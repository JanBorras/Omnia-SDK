#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_uart.h"

#include "port_stm32.h"
#include "omnia_port.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * Implementació del port STM32 per a Omnia (STM32H7).
 *
 * Objectiu del prototip:
 *  - ADC: adc_read_n() blocking recolzat per DMA circular (estable a 2kHz).
 *  - UART: ASYNC sempre:
 *      * TX: DMA + cua ring de paquets (no bloqueja mai)
 *      * RX: IT 1-byte + ring buffer (no bloqueja mai)
 *
 * IMPORTANT:
 *  - Config HW per ADC (TIM3 TRGO @2kHz, ADC1 trigger extern, DMA circular) via CubeMX.
 *  - Config HW per UART3 @921600 via CubeMX.
 *  - UART3 TX DMA habilitat via CubeMX (USART3_TX DMA).
 *
 * CACHE/DMA (M7):
 *  - Si actives DCache: buffers DMA han d'estar en MPU no-cacheable o fer invalidate/clean.
 */

/* ------------------------------------------------------------------------- */
/* Helpers: secció crítica curta                                              */
/* ------------------------------------------------------------------------- */

static inline uint32_t omnia_irq_save(void)
{
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  return prim;
}

static inline void omnia_irq_restore(uint32_t prim)
{
  if (!prim) __enable_irq();
}

/* ------------------------------------------------------------------------- */
/* GPIO pool opac                                                             */
/* ------------------------------------------------------------------------- */

#define OMNIA_STM32_MAX_PINS 16

typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
} stm32_gpio_desc_t;

static stm32_gpio_desc_t gpio_pool[OMNIA_STM32_MAX_PINS];
static size_t            gpio_pool_used = 0;

/* ------------------------------------------------------------------------- */
/* Temps: delay_us via DWT + millis via HAL_GetTick                           */
/* ------------------------------------------------------------------------- */

static uint32_t dwt_ticks_per_us = 0;

static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#if defined(DWT_LAR)
  DWT->LAR = 0xC5ACCE55;
#endif

  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  dwt_ticks_per_us = SystemCoreClock / 1000000U;
  if (dwt_ticks_per_us == 0U) dwt_ticks_per_us = 1U;
}

static void delay_us_impl(uint32_t us)
{
  if (us == 0U) return;
  if (dwt_ticks_per_us == 0U) dwt_delay_init();

  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * dwt_ticks_per_us;

  while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
    __NOP();
  }
}

static uint64_t millis_impl(void)
{
  return (uint64_t)HAL_GetTick();
}

/* ------------------------------------------------------------------------- */
/* GPIO                                                                       */
/* ------------------------------------------------------------------------- */

static omnia_status_t gpio_write_impl(omnia_gpio_t pin_handle, int level)
{
  if (pin_handle == NULL) return OMNIA_EINVAL;
  stm32_gpio_desc_t *g = (stm32_gpio_desc_t *)pin_handle;

  HAL_GPIO_WritePin(g->port, g->pin, (level ? GPIO_PIN_SET : GPIO_PIN_RESET));
  return OMNIA_OK;
}

static omnia_status_t gpio_mode_impl(omnia_gpio_t pin_handle, int is_output, int pull)
{
  if (pin_handle == NULL) return OMNIA_EINVAL;
  stm32_gpio_desc_t *g = (stm32_gpio_desc_t *)pin_handle;

  GPIO_InitTypeDef init = {0};
  init.Pin   = g->pin;
  init.Mode  = is_output ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  switch ((omnia_gpio_pull_t)pull) {
    case OMNIA_GPIO_PULL_NONE: init.Pull = GPIO_NOPULL;   break;
    case OMNIA_GPIO_PULL_UP:   init.Pull = GPIO_PULLUP;   break;
    case OMNIA_GPIO_PULL_DOWN: init.Pull = GPIO_PULLDOWN; break;
    default: return OMNIA_EINVAL;
  }

  HAL_GPIO_Init(g->port, &init);
  return OMNIA_OK;
}

static int gpio_read_impl(omnia_gpio_t pin_handle)
{
  if (pin_handle == NULL) return -1;
  stm32_gpio_desc_t *g = (stm32_gpio_desc_t *)pin_handle;

  GPIO_PinState s = HAL_GPIO_ReadPin(g->port, g->pin);
  return (s == GPIO_PIN_SET) ? 1 : 0;
}

/* ------------------------------------------------------------------------- */
/* SPI                                                                        */
/* ------------------------------------------------------------------------- */

static omnia_status_t spi_tx_impl(omnia_spi_t spi, const uint8_t *buf, size_t n)
{
  if (spi == NULL || buf == NULL || n == 0U) return OMNIA_EINVAL;

  SPI_HandleTypeDef *h = (SPI_HandleTypeDef *)spi;

  h->ErrorCode = HAL_SPI_ERROR_NONE;
  h->State     = HAL_SPI_STATE_READY;

  HAL_StatusTypeDef st = HAL_SPI_Transmit(h, (uint8_t *)buf, (uint16_t)n, 1000U);
  if (st != HAL_OK) {
    __BKPT(1);
    return OMNIA_EIO;
  }
  return OMNIA_OK;
}

static omnia_status_t spi_txrx_impl(omnia_spi_t spi,
                                   const uint8_t *tx,
                                   uint8_t *rx,
                                   size_t n)
{
  if (spi == NULL || tx == NULL || rx == NULL || n == 0U) return OMNIA_EINVAL;

  SPI_HandleTypeDef *h = (SPI_HandleTypeDef *)spi;

  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(h,
                                                (uint8_t *)tx,
                                                rx,
                                                (uint16_t)n,
                                                1000U);
  if (st != HAL_OK) {
    __BKPT(2);
    return OMNIA_EIO;
  }
  return OMNIA_OK;
}

/* ------------------------------------------------------------------------- */
/* UART async (USART3 VCP): TX DMA + queue, RX IT + ring                      */
/* ------------------------------------------------------------------------- */

/*
 * Contracte:
 *  - uart_write(u,data,len): NO bloqueja mai.
 *      OMNIA_OK   => encolat (o enviant)
 *      OMNIA_EBUSY=> cua plena (drop si és stream)
 *      OMNIA_EIO  => error HAL
 *
 *  - uart_read(u,data,len,&out): NO bloqueja.
 *      OMNIA_OK   => out_nread > 0
 *      OMNIA_EBUSY=> no hi ha dades
 */

#define STM32_UART_TX_SLOTS      8u
#define STM32_UART_TX_SLOT_SZ    256u   /* suficient per paquets EMG típics (ajusta si cal) */

typedef struct {
  UART_HandleTypeDef* huart;

  /* TX queue (ring of buffers) */
  uint8_t  tx_slots[STM32_UART_TX_SLOTS][STM32_UART_TX_SLOT_SZ];
  uint16_t tx_len[STM32_UART_TX_SLOTS];
  volatile uint8_t  tx_busy;
  volatile uint8_t  tx_head;  /* next to send */
  volatile uint8_t  tx_tail;  /* next free slot */

  /* RX ring (IT one byte) */
  uint8_t  rx_buf[1024];
  volatile uint16_t rx_wr;
  volatile uint16_t rx_rd;
  uint8_t  rx_byte;

} stm32_uart_ctx_t;

static stm32_uart_ctx_t g_uart3_ctx = {0};

static inline uint8_t ring_next_u8(uint8_t v, uint8_t mod)
{
  v++;
  if (v >= mod) v = 0;
  return v;
}

static void stm32_uart3_kick_tx(void)
{
  /* Assumeix IRQs enabled (no cridar dins secció crítica llarga) */
  if (!g_uart3_ctx.huart) return;
  if (g_uart3_ctx.tx_busy) return;

  if (g_uart3_ctx.tx_head == g_uart3_ctx.tx_tail) {
    /* cua buida */
    return;
  }

  uint8_t idx = g_uart3_ctx.tx_head;

  g_uart3_ctx.tx_busy = 1;

  if (HAL_UART_Transmit_DMA(g_uart3_ctx.huart,
                            g_uart3_ctx.tx_slots[idx],
                            g_uart3_ctx.tx_len[idx]) != HAL_OK) {
    /* si falla, allibera i marca I/O error via busy=0, però la cua queda amb l'element.
       L'app veurà EIO només quan intenti enviar; aquí fem BKPT per debug. */
    g_uart3_ctx.tx_busy = 0;
  }
}

static int stm32_uart_tx_busy_impl(omnia_uart_t u)
{
  (void)u;
  /* busy o cua no buida */
  if (g_uart3_ctx.tx_busy) return 1;
  return (g_uart3_ctx.tx_head != g_uart3_ctx.tx_tail) ? 1 : 0;
}

static omnia_status_t stm32_uart_write_async_impl(omnia_uart_t u,
                                                 const uint8_t* data,
                                                 size_t len)
{
  (void)u;
  if (!g_uart3_ctx.huart || !data || len == 0) return OMNIA_EINVAL;
  if (len > STM32_UART_TX_SLOT_SZ) return OMNIA_EINVAL;

  uint32_t prim = omnia_irq_save();

  uint8_t next_tail = ring_next_u8(g_uart3_ctx.tx_tail, STM32_UART_TX_SLOTS);

  if (next_tail == g_uart3_ctx.tx_head) {
    /* cua plena */
    omnia_irq_restore(prim);
    return OMNIA_EBUSY;
  }

  uint8_t slot = g_uart3_ctx.tx_tail;
  memcpy(g_uart3_ctx.tx_slots[slot], data, len);
  g_uart3_ctx.tx_len[slot] = (uint16_t)len;
  g_uart3_ctx.tx_tail = next_tail;

  omnia_irq_restore(prim);

  /* kick DMA si estava idle */
  stm32_uart3_kick_tx();

  return OMNIA_OK;
}

static size_t stm32_uart_rx_available_impl(omnia_uart_t u)
{
  (void)u;
  uint16_t wr = g_uart3_ctx.rx_wr;
  uint16_t rd = g_uart3_ctx.rx_rd;
  if (wr >= rd) return (size_t)(wr - rd);
  return (size_t)(sizeof(g_uart3_ctx.rx_buf) - (rd - wr));
}

static omnia_status_t stm32_uart_read_async_impl(omnia_uart_t u,
                                                uint8_t* data,
                                                size_t len,
                                                size_t* out_nread)
{
  (void)u;
  if (!data || len == 0 || !out_nread) return OMNIA_EINVAL;
  *out_nread = 0;

  size_t avail = stm32_uart_rx_available_impl(u);
  if (avail == 0) return OMNIA_EBUSY;

  size_t n = (avail < len) ? avail : len;

  uint32_t prim = omnia_irq_save();

  for (size_t i = 0; i < n; ++i) {
    data[i] = g_uart3_ctx.rx_buf[g_uart3_ctx.rx_rd];
    g_uart3_ctx.rx_rd++;
    if (g_uart3_ctx.rx_rd >= (uint16_t)sizeof(g_uart3_ctx.rx_buf)) {
      g_uart3_ctx.rx_rd = 0;
    }
  }

  omnia_irq_restore(prim);

  *out_nread = n;
  return OMNIA_OK;
}

static void stm32_uart_rx_start_it(void)
{
  if (!g_uart3_ctx.huart) return;
  (void)HAL_UART_Receive_IT(g_uart3_ctx.huart, &g_uart3_ctx.rx_byte, 1);
}

/* HAL callbacks: cal que aquests símbols siguin únics al projecte.
   Si tens altres mòduls amb callbacks, centralitza-ho o fes weak override. */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == g_uart3_ctx.huart) {
    /* envia següent element de cua */
    uint32_t prim = omnia_irq_save();

    /* hem acabat d'enviar el head */
    g_uart3_ctx.tx_head = ring_next_u8(g_uart3_ctx.tx_head, STM32_UART_TX_SLOTS);
    g_uart3_ctx.tx_busy = 0;

    omnia_irq_restore(prim);

    /* kick següent si n'hi ha */
    stm32_uart3_kick_tx();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == g_uart3_ctx.huart) {
    /* error UART: marca idle i intenta continuar. */
    g_uart3_ctx.tx_busy = 0;
    stm32_uart3_kick_tx();

    /* RX: rearm */
    stm32_uart_rx_start_it();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == g_uart3_ctx.huart) {

    uint16_t wr = g_uart3_ctx.rx_wr;
    g_uart3_ctx.rx_buf[wr] = g_uart3_ctx.rx_byte;

    wr++;
    if (wr >= (uint16_t)sizeof(g_uart3_ctx.rx_buf)) wr = 0;

    g_uart3_ctx.rx_wr = wr;

    /* overflow: drop-oldest */
    if (g_uart3_ctx.rx_wr == g_uart3_ctx.rx_rd) {
      uint16_t rd = g_uart3_ctx.rx_rd;
      rd++;
      if (rd >= (uint16_t)sizeof(g_uart3_ctx.rx_buf)) rd = 0;
      g_uart3_ctx.rx_rd = rd;
    }

    /* rearm */
    (void)HAL_UART_Receive_IT(g_uart3_ctx.huart, &g_uart3_ctx.rx_byte, 1);
  }
}

/* ------------------------------------------------------------------------- */
/* ADC: DMA circular context                                                  */
/* ------------------------------------------------------------------------- */

typedef struct {
  ADC_HandleTypeDef* hadc;

  uint16_t*          dma_buf;
  uint32_t           dma_len;      // total length (2*block_n)
  uint32_t           block_n;      // half size

  volatile uint32_t  available;   // samples ready (0..dma_len)
  uint32_t           rd_idx;       // read index in ring
  volatile uint8_t   dma_started;  // 0/1
} stm32_adc_ctx_t;

/* Prototip curt: ADC1 únic per EMG */
#define STM32_ADC_BLOCK_N   32U
#define STM32_ADC_DMA_LEN  (2U * STM32_ADC_BLOCK_N)

/* Buffer DMA circular */
static uint16_t g_adc1_dma_buf[STM32_ADC_DMA_LEN]
  __attribute__((section(".RAM_D1"), aligned(32)));

static stm32_adc_ctx_t g_adc1_ctx = {
  .hadc        = NULL,
  .dma_buf     = g_adc1_dma_buf,
  .dma_len     = STM32_ADC_DMA_LEN,
  .block_n     = STM32_ADC_BLOCK_N,
  .available   = 0,
  .rd_idx      = 0,
  .dma_started = 0,
};

static inline uint32_t stm32_adc_wr_idx(const stm32_adc_ctx_t* ctx)
{
  if (!ctx || !ctx->hadc) return 0;

  DMA_HandleTypeDef* hdma = ctx->hadc->DMA_Handle;
  if (!hdma) return 0;

  uint32_t ndtr = __HAL_DMA_GET_COUNTER(hdma); // elements que queden
  uint32_t wr = (ctx->dma_len - ndtr) % ctx->dma_len;
  return wr;
}

/* Política d'overflow: drop-oldest -> enganxa't al present */
static inline void stm32_adc_handle_overflow(stm32_adc_ctx_t* ctx)
{
  if (!ctx) return;
  ctx->available = ctx->dma_len;
  ctx->rd_idx = stm32_adc_wr_idx(ctx);
}

static omnia_status_t stm32_adc_dma_start(stm32_adc_ctx_t* ctx)
{
  if (!ctx || !ctx->hadc) return OMNIA_EINVAL;

  ctx->available   = 0;
  ctx->rd_idx      = 0;
  ctx->dma_started = 0;

  HAL_StatusTypeDef st = HAL_ADC_Start_DMA(ctx->hadc,
                                          (uint32_t*)ctx->dma_buf,
                                          ctx->dma_len);
  if (st == HAL_OK) {
    ctx->dma_started = 1;
    return OMNIA_OK;
  }
  return OMNIA_EIO;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == g_adc1_ctx.hadc && g_adc1_ctx.dma_started) {
    uint32_t new_av = g_adc1_ctx.available + g_adc1_ctx.block_n;
    g_adc1_ctx.available = new_av;

    if (g_adc1_ctx.available > g_adc1_ctx.dma_len) {
      stm32_adc_handle_overflow(&g_adc1_ctx);
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == g_adc1_ctx.hadc && g_adc1_ctx.dma_started) {
    uint32_t new_av = g_adc1_ctx.available + g_adc1_ctx.block_n;
    g_adc1_ctx.available = new_av;

    if (g_adc1_ctx.available > g_adc1_ctx.dma_len) {
      stm32_adc_handle_overflow(&g_adc1_ctx);
    }
  }
}

/* ------------------------------------------------------------------------- */
/* ADC API via vtable                                                         */
/* ------------------------------------------------------------------------- */

static omnia_status_t adc_read_impl(omnia_adc_t dev, uint16_t *out)
{
  if (dev == NULL || out == NULL) return OMNIA_EINVAL;

  if (g_adc1_ctx.dma_started && (ADC_HandleTypeDef*)dev == g_adc1_ctx.hadc) {

    while (g_adc1_ctx.available < 1U) { __NOP(); }

    __DMB();

    uint32_t prim = omnia_irq_save();

    *out = g_adc1_ctx.dma_buf[g_adc1_ctx.rd_idx];

    g_adc1_ctx.rd_idx++;
    if (g_adc1_ctx.rd_idx >= g_adc1_ctx.dma_len) g_adc1_ctx.rd_idx = 0;

    g_adc1_ctx.available -= 1U;

    omnia_irq_restore(prim);

    return OMNIA_OK;
  }

  /* fallback single-shot */
  ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)dev;

  if (HAL_ADC_Start(hadc) != HAL_OK) return OMNIA_EIO;

  if (HAL_ADC_PollForConversion(hadc, 10U) != HAL_OK) {
    (void)HAL_ADC_Stop(hadc);
    return OMNIA_ETIMEDOUT;
  }

  uint32_t value = HAL_ADC_GetValue(hadc);
  (void)HAL_ADC_Stop(hadc);

  *out = (uint16_t)value;
  return OMNIA_OK;
}

static omnia_status_t adc_read_n_impl(omnia_adc_t dev, uint16_t *buf, size_t n)
{
  if (dev == NULL || buf == NULL || n == 0U) return OMNIA_EINVAL;

  stm32_adc_ctx_t* ctx = &g_adc1_ctx;

  if (!ctx->dma_started || (ADC_HandleTypeDef*)dev != ctx->hadc) {
    for (size_t i = 0; i < n; ++i) {
      omnia_status_t st = adc_read_impl(dev, &buf[i]);
      if (st != OMNIA_OK) return st;
    }
    return OMNIA_OK;
  }

  while (ctx->available < (uint32_t)n) { __NOP(); }

  __DMB();

  uint32_t prim = omnia_irq_save();

  for (size_t i = 0; i < n; ++i) {
    buf[i] = ctx->dma_buf[ctx->rd_idx];

    ctx->rd_idx++;
    if (ctx->rd_idx >= ctx->dma_len) ctx->rd_idx = 0;
  }

  ctx->available -= (uint32_t)n;

  omnia_irq_restore(prim);

  return OMNIA_OK;
}

/* ------------------------------------------------------------------------- */
/* Vtable STM32                                                               */
/* ------------------------------------------------------------------------- */

static const omnia_port_vtable_t STM32_PORT_VTABLE = {

  /* GPIO */
  .gpio_mode  = gpio_mode_impl,
  .gpio_write = gpio_write_impl,
  .gpio_read  = gpio_read_impl,

  /* SPI */
  .spi_tx   = spi_tx_impl,
  .spi_tx16 = NULL,
  .spi_txrx = spi_txrx_impl,

  /* ADC */
  .adc_read   = adc_read_impl,
  .adc_read_n = adc_read_n_impl,

  /* UART async */
  .uart_write        = stm32_uart_write_async_impl,
  .uart_read         = stm32_uart_read_async_impl,
  .uart_tx_busy      = stm32_uart_tx_busy_impl,
  .uart_rx_available = stm32_uart_rx_available_impl,

  /* Temps */
  .delay_us = delay_us_impl,
  .millis   = millis_impl,

  /* Concurrència (ara mateix no implementat) */
  .mutex_create  = NULL,
  .mutex_lock    = NULL,
  .mutex_unlock  = NULL,
  .mutex_destroy = NULL,

  /* Memòria (opcional) */
  .malloc_fn = NULL,
  .free_fn   = NULL,

  /* Log (opcional) */
  .log = NULL,

  .user_ctx = NULL,
};

/* ------------------------------------------------------------------------- */
/* API pública del port                                                       */
/* ------------------------------------------------------------------------- */

void omnia_port_stm32_init(void)
{
  extern ADC_HandleTypeDef hadc1;
  extern UART_HandleTypeDef huart3;

  /* ADC1 context */
  g_adc1_ctx.hadc = &hadc1;

  /* UART3 context */
  memset(&g_uart3_ctx, 0, sizeof(g_uart3_ctx));
  g_uart3_ctx.huart = &huart3;
  g_uart3_ctx.tx_busy = 0;
  g_uart3_ctx.tx_head = 0;
  g_uart3_ctx.tx_tail = 0;

  g_uart3_ctx.rx_wr = 0;
  g_uart3_ctx.rx_rd = 0;

  stm32_uart_rx_start_it();

  (void)omnia_port_register(&STM32_PORT_VTABLE);
}

omnia_gpio_t omnia_mkpin(void *port, uint16_t pin)
{
  if (gpio_pool_used >= OMNIA_STM32_MAX_PINS) return NULL;

  gpio_pool[gpio_pool_used].port = (GPIO_TypeDef *)port;
  gpio_pool[gpio_pool_used].pin  = pin;

  return (omnia_gpio_t)&gpio_pool[gpio_pool_used++];
}

omnia_status_t omnia_port_stm32_adc1_dma_start(void)
{
  omnia_status_t st = stm32_adc_dma_start(&g_adc1_ctx);
  if (st != OMNIA_OK) {
    __BKPT(3);
  }
  return st;
}
