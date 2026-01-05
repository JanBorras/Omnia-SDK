// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (c) 2025 Jan Borràs Ros
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file port_stm32.c
 * @brief STM32H7 Omnia port implementation (GPIO/SPI/UART/ADC/time).
 *
 * This module binds STM32 HAL peripherals to the Omnia port contract
 * (omnia_port_vtable_t). It provides:
 * - GPIO: read/write + mode configuration.
 * - Time: delay_us() via DWT CYCCNT, millis() via HAL_GetTick().
 * - SPI: blocking TX/TXRX (HAL_SPI_Transmit / TransmitReceive).
 * - UART: non-blocking contract (TX via DMA + ring queue, RX via IT + ring buffer).
 * - ADC: adc_read_n() blocking over circular DMA (stable at kHz rates).
 *
 * Hardware configuration assumptions (CubeMX):
 * - ADC1: external trigger (e.g., TIM3 TRGO @ 2 kHz), DMA in circular mode.
 * - USART3: configured at the desired baud rate (e.g., 921600), TX DMA enabled.
 *
 * Notes for Cortex-M7:
 * - If DCache is enabled, DMA buffers must be placed in non-cacheable memory
 *   or managed with clean/invalidate operations (MPU strongly recommended).
 */

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_uart.h"

#include "port_stm32.h"
#include "omnia_port.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Short critical sections                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Disable IRQs and return previous PRIMASK state.
 *
 * This helper is used to protect short ring-buffer updates shared with IRQ
 * context. Keep the protected sections minimal.
 *
 * @return Previous PRIMASK value.
 */
static inline uint32_t omnia_irq_save(void)
{
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  return prim;
}

/**
 * @brief Restore IRQ state based on a saved PRIMASK value.
 *
 * @param prim PRIMASK value returned by omnia_irq_save().
 */
static inline void omnia_irq_restore(uint32_t prim)
{
  if (!prim) __enable_irq();
}

/* -------------------------------------------------------------------------- */
/* Opaque GPIO pool                                                            */
/* -------------------------------------------------------------------------- */

/** @brief Maximum number of opaque GPIO handles that can be created. */
#define OMNIA_STM32_MAX_PINS 16

/**
 * @brief Internal representation of an opaque GPIO handle.
 *
 * The public API exposes omnia_gpio_t as a void*; the STM32 port maps it
 * to a (GPIO port, pin) pair.
 */
typedef struct {
  GPIO_TypeDef* port;
  uint16_t      pin;
} stm32_gpio_desc_t;

static stm32_gpio_desc_t gpio_pool[OMNIA_STM32_MAX_PINS];
static size_t            gpio_pool_used = 0;

/* -------------------------------------------------------------------------- */
/* Time: delay_us via DWT, millis via HAL                                      */
/* -------------------------------------------------------------------------- */

static uint32_t dwt_ticks_per_us = 0;

/**
 * @brief Initialize DWT CYCCNT for microsecond busy-wait delays.
 *
 * This enables the cycle counter and computes ticks-per-microsecond from
 * SystemCoreClock. It is initialized lazily on first delay_us() call.
 */
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

/**
 * @brief Omnia port delay_us() implementation using DWT CYCCNT.
 *
 * @param us Delay duration in microseconds.
 */
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

/**
 * @brief Omnia port millis() implementation.
 *
 * Uses HAL_GetTick() (typically 1 ms SysTick).
 *
 * @return Monotonic milliseconds since boot (HAL tick).
 */
static uint64_t millis_impl(void)
{
  return (uint64_t)HAL_GetTick();
}

/* -------------------------------------------------------------------------- */
/* GPIO                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Omnia port gpio_write() implementation.
 *
 * @param pin_handle Opaque pin handle (from omnia_mkpin()).
 * @param level      Logic level (0/1).
 *
 * @return OMNIA_OK on success, OMNIA_EINVAL on invalid handle.
 */
static omnia_status_t gpio_write_impl(omnia_gpio_t pin_handle, int level)
{
  if (pin_handle == NULL) return OMNIA_EINVAL;
  stm32_gpio_desc_t* g = (stm32_gpio_desc_t*)pin_handle;

  HAL_GPIO_WritePin(g->port, g->pin, (level ? GPIO_PIN_SET : GPIO_PIN_RESET));
  return OMNIA_OK;
}

/**
 * @brief Omnia port gpio_mode() implementation.
 *
 * @param pin_handle Opaque pin handle (from omnia_mkpin()).
 * @param is_output  1 = output push-pull, 0 = input.
 * @param pull       omnia_gpio_pull_t value.
 *
 * @return OMNIA_OK on success, OMNIA_EINVAL on invalid parameters.
 */
static omnia_status_t gpio_mode_impl(omnia_gpio_t pin_handle, int is_output, int pull)
{
  if (pin_handle == NULL) return OMNIA_EINVAL;
  stm32_gpio_desc_t* g = (stm32_gpio_desc_t*)pin_handle;

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

/**
 * @brief Omnia port gpio_read() implementation.
 *
 * @param pin_handle Opaque pin handle (from omnia_mkpin()).
 *
 * @return 1 if set, 0 if reset, -1 on invalid handle.
 */
static int gpio_read_impl(omnia_gpio_t pin_handle)
{
  if (pin_handle == NULL) return -1;
  stm32_gpio_desc_t* g = (stm32_gpio_desc_t*)pin_handle;

  GPIO_PinState s = HAL_GPIO_ReadPin(g->port, g->pin);
  return (s == GPIO_PIN_SET) ? 1 : 0;
}

/* -------------------------------------------------------------------------- */
/* SPI                                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Omnia port spi_tx() implementation (blocking).
 *
 * @param spi SPI handle (SPI_HandleTypeDef* cast to omnia_spi_t).
 * @param buf Bytes to transmit.
 * @param n   Number of bytes.
 *
 * @return OMNIA_OK on success, error status otherwise.
 */
static omnia_status_t spi_tx_impl(omnia_spi_t spi, const uint8_t* buf, size_t n)
{
  if (spi == NULL || buf == NULL || n == 0U) return OMNIA_EINVAL;

  SPI_HandleTypeDef* h = (SPI_HandleTypeDef*)spi;

  /* Ensure predictable behavior if HAL state was left dirty by user code. */
  h->ErrorCode = HAL_SPI_ERROR_NONE;
  h->State     = HAL_SPI_STATE_READY;

  HAL_StatusTypeDef st = HAL_SPI_Transmit(h, (uint8_t*)buf, (uint16_t)n, 1000U);
  if (st != HAL_OK) {
    __BKPT(1);
    return OMNIA_EIO;
  }
  return OMNIA_OK;
}

/**
 * @brief Omnia port spi_txrx() implementation (blocking).
 *
 * @param spi SPI handle (SPI_HandleTypeDef* cast to omnia_spi_t).
 * @param tx  Bytes to transmit.
 * @param rx  Receive buffer.
 * @param n   Number of bytes.
 *
 * @return OMNIA_OK on success, error status otherwise.
 */
static omnia_status_t spi_txrx_impl(omnia_spi_t spi,
                                   const uint8_t* tx,
                                   uint8_t* rx,
                                   size_t n)
{
  if (spi == NULL || tx == NULL || rx == NULL || n == 0U) return OMNIA_EINVAL;

  SPI_HandleTypeDef* h = (SPI_HandleTypeDef*)spi;

  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(h,
                                                (uint8_t*)tx,
                                                rx,
                                                (uint16_t)n,
                                                1000U);
  if (st != HAL_OK) {
    __BKPT(2);
    return OMNIA_EIO;
  }
  return OMNIA_OK;
}

/* -------------------------------------------------------------------------- */
/* UART async (USART3): TX DMA + queue, RX IT + ring                           */
/* -------------------------------------------------------------------------- */

/**
 * UART contract mapping:
 * - uart_write(): never blocks. Returns OMNIA_EBUSY if queue is full.
 * - uart_read(): never blocks. Returns OMNIA_EBUSY if no data is available.
 */

#define STM32_UART_TX_SLOTS   8u
#define STM32_UART_TX_SLOT_SZ 256u

/**
 * @brief UART runtime context for a single UART instance.
 *
 * TX:
 * - Ring of fixed-size slots holding whole frames/packets.
 * - DMA sends one slot at a time; TxCpltCallback advances the head.
 *
 * RX:
 * - One-byte interrupt re-armed continuously into a circular buffer.
 * - On overflow, oldest bytes are dropped (drop-oldest policy).
 */
typedef struct {
  UART_HandleTypeDef* huart;

  uint8_t  tx_slots[STM32_UART_TX_SLOTS][STM32_UART_TX_SLOT_SZ];
  uint16_t tx_len[STM32_UART_TX_SLOTS];
  volatile uint8_t  tx_busy;
  volatile uint8_t  tx_head;
  volatile uint8_t  tx_tail;

  uint8_t  rx_buf[1024];
  volatile uint16_t rx_wr;
  volatile uint16_t rx_rd;
  uint8_t  rx_byte;
} stm32_uart_ctx_t;

static stm32_uart_ctx_t g_uart3_ctx = {0};

/**
 * @brief Ring increment helper for small u8 rings.
 *
 * @param v   Current value.
 * @param mod Ring size.
 * @return Next value in [0..mod-1].
 */
static inline uint8_t ring_next_u8(uint8_t v, uint8_t mod)
{
  v++;
  if (v >= mod) v = 0;
  return v;
}

/**
 * @brief Start a DMA TX transfer if idle and queue is not empty.
 *
 * This function does not block. It is safe to call after enqueuing data,
 * and from TX complete callback to chain transfers.
 */
static void stm32_uart3_kick_tx(void)
{
  if (!g_uart3_ctx.huart) return;
  if (g_uart3_ctx.tx_busy) return;

  if (g_uart3_ctx.tx_head == g_uart3_ctx.tx_tail) {
    return; /* queue empty */
  }

  uint8_t idx = g_uart3_ctx.tx_head;
  g_uart3_ctx.tx_busy = 1;

  if (HAL_UART_Transmit_DMA(g_uart3_ctx.huart,
                           g_uart3_ctx.tx_slots[idx],
                           g_uart3_ctx.tx_len[idx]) != HAL_OK) {
    /* Keep element in the queue; allow retry on next kick. */
    g_uart3_ctx.tx_busy = 0;
  }
}

/**
 * @brief Omnia port uart_tx_busy() implementation.
 *
 * @return 1 if DMA is busy or queue is not empty, 0 otherwise.
 */
static int stm32_uart_tx_busy_impl(omnia_uart_t u)
{
  (void)u;
  if (g_uart3_ctx.tx_busy) return 1;
  return (g_uart3_ctx.tx_head != g_uart3_ctx.tx_tail) ? 1 : 0;
}

/**
 * @brief Omnia port uart_write() implementation (non-blocking).
 *
 * Copies the frame into an internal TX slot and triggers DMA if idle.
 *
 * @param u    Opaque UART handle (ignored; single-instance mapping).
 * @param data Frame data.
 * @param len  Frame length (must fit into STM32_UART_TX_SLOT_SZ).
 *
 * @return OMNIA_OK if enqueued, OMNIA_EBUSY if queue full, or OMNIA_EINVAL.
 */
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
    omnia_irq_restore(prim);
    return OMNIA_EBUSY; /* queue full */
  }

  uint8_t slot = g_uart3_ctx.tx_tail;
  memcpy(g_uart3_ctx.tx_slots[slot], data, len);
  g_uart3_ctx.tx_len[slot] = (uint16_t)len;
  g_uart3_ctx.tx_tail = next_tail;

  omnia_irq_restore(prim);

  stm32_uart3_kick_tx();
  return OMNIA_OK;
}

/**
 * @brief Omnia port uart_rx_available() implementation.
 *
 * @return Number of bytes currently available in the RX ring.
 */
static size_t stm32_uart_rx_available_impl(omnia_uart_t u)
{
  (void)u;
  uint16_t wr = g_uart3_ctx.rx_wr;
  uint16_t rd = g_uart3_ctx.rx_rd;
  if (wr >= rd) return (size_t)(wr - rd);
  return (size_t)(sizeof(g_uart3_ctx.rx_buf) - (rd - wr));
}

/**
 * @brief Omnia port uart_read() implementation (non-blocking).
 *
 * Copies up to @p len bytes from the RX ring.
 *
 * @param u         Opaque UART handle (ignored; single-instance mapping).
 * @param data      Output buffer.
 * @param len       Maximum bytes to read.
 * @param out_nread Number of bytes actually read.
 *
 * @return OMNIA_OK if at least one byte was read, OMNIA_EBUSY if none available.
 */
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

/**
 * @brief Arm continuous RX via 1-byte interrupt.
 *
 * This must be called once during init and is re-armed in the RX callback.
 */
static void stm32_uart_rx_start_it(void)
{
  if (!g_uart3_ctx.huart) return;
  (void)HAL_UART_Receive_IT(g_uart3_ctx.huart, &g_uart3_ctx.rx_byte, 1);
}

/* HAL callbacks: these symbols must be unique in the final link.
 * If multiple modules define them, consolidate into a single translation unit.
 */

/**
 * @brief HAL UART TX complete callback (DMA).
 *
 * Advances the TX queue head and chains the next pending transfer.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == g_uart3_ctx.huart) {
    uint32_t prim = omnia_irq_save();

    g_uart3_ctx.tx_head = ring_next_u8(g_uart3_ctx.tx_head, STM32_UART_TX_SLOTS);
    g_uart3_ctx.tx_busy = 0;

    omnia_irq_restore(prim);

    stm32_uart3_kick_tx();
  }
}

/**
 * @brief HAL UART error callback.
 *
 * Best-effort recovery:
 * - Mark TX idle and attempt to resume DMA chain.
 * - Re-arm RX interrupt.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  if (huart == g_uart3_ctx.huart) {
    g_uart3_ctx.tx_busy = 0;
    stm32_uart3_kick_tx();
    stm32_uart_rx_start_it();
  }
}

/**
 * @brief HAL UART RX complete callback (1-byte IT).
 *
 * Pushes the received byte into the RX ring. On overflow, drops the oldest byte.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == g_uart3_ctx.huart) {

    uint16_t wr = g_uart3_ctx.rx_wr;
    g_uart3_ctx.rx_buf[wr] = g_uart3_ctx.rx_byte;

    wr++;
    if (wr >= (uint16_t)sizeof(g_uart3_ctx.rx_buf)) wr = 0;

    g_uart3_ctx.rx_wr = wr;

    /* Overflow policy: drop-oldest. */
    if (g_uart3_ctx.rx_wr == g_uart3_ctx.rx_rd) {
      uint16_t rd = g_uart3_ctx.rx_rd;
      rd++;
      if (rd >= (uint16_t)sizeof(g_uart3_ctx.rx_buf)) rd = 0;
      g_uart3_ctx.rx_rd = rd;
    }

    (void)HAL_UART_Receive_IT(g_uart3_ctx.huart, &g_uart3_ctx.rx_byte, 1);
  }
}

/* -------------------------------------------------------------------------- */
/* ADC: circular DMA context                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief ADC circular DMA reader context.
 *
 * The design is optimized for a stable fixed sample rate using DMA circular mode
 * and half/full completion callbacks. Consumer reads from a ring in a blocking
 * manner (suitable for deterministic acquisition pipelines).
 */
typedef struct {
  ADC_HandleTypeDef* hadc;

  uint16_t*          dma_buf;
  uint32_t           dma_len;
  uint32_t           block_n;

  volatile uint32_t  available;
  uint32_t           rd_idx;
  volatile uint8_t   dma_started;
} stm32_adc_ctx_t;

#define STM32_ADC_BLOCK_N  32U
#define STM32_ADC_DMA_LEN  (2U * STM32_ADC_BLOCK_N)

/**
 * @brief ADC1 DMA circular buffer.
 *
 * Aligned and placed in D1 RAM as a typical STM32H7 choice. If DCache is enabled,
 * ensure this section is configured as non-cacheable in the MPU, or add explicit
 * cache maintenance around DMA operations.
 */
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

/**
 * @brief Compute DMA write index in the circular buffer.
 *
 * Uses DMA NDTR to infer the current write position.
 */
static inline uint32_t stm32_adc_wr_idx(const stm32_adc_ctx_t* ctx)
{
  if (!ctx || !ctx->hadc) return 0;

  DMA_HandleTypeDef* hdma = ctx->hadc->DMA_Handle;
  if (!hdma) return 0;

  uint32_t ndtr = __HAL_DMA_GET_COUNTER(hdma);
  uint32_t wr = (ctx->dma_len - ndtr) % ctx->dma_len;
  return wr;
}

/**
 * @brief Overflow policy: drop-oldest and resync consumer to current write index.
 */
static inline void stm32_adc_handle_overflow(stm32_adc_ctx_t* ctx)
{
  if (!ctx) return;
  ctx->available = ctx->dma_len;
  ctx->rd_idx = stm32_adc_wr_idx(ctx);
}

/**
 * @brief Start ADC circular DMA acquisition.
 *
 * @param ctx ADC context.
 * @return OMNIA_OK on success, OMNIA_EIO on HAL failure.
 */
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

/**
 * @brief HAL ADC half-transfer callback.
 *
 * Marks half-buffer worth of samples as available.
 */
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

/**
 * @brief HAL ADC full-transfer callback.
 *
 * Marks the other half-buffer worth of samples as available.
 */
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

/* -------------------------------------------------------------------------- */
/* ADC API via vtable                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Omnia port adc_read() implementation.
 *
 * If ADC1 DMA circular acquisition is running, reads from the ring buffer.
 * Otherwise, falls back to a single-shot HAL ADC conversion.
 *
 * @param dev ADC device handle (ADC_HandleTypeDef* cast to omnia_adc_t).
 * @param out Output raw sample.
 *
 * @return OMNIA_OK on success, or error code on failure.
 */
static omnia_status_t adc_read_impl(omnia_adc_t dev, uint16_t* out)
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

  /* Fallback: single-shot conversion. */
  ADC_HandleTypeDef* hadc = (ADC_HandleTypeDef*)dev;

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

/**
 * @brief Omnia port adc_read_n() implementation (blocking).
 *
 * If DMA circular acquisition is running for ADC1, waits until @p n samples
 * are available and copies them from the ring. Otherwise, uses repeated
 * adc_read() calls.
 *
 * @param dev ADC device handle.
 * @param buf Output buffer.
 * @param n   Number of samples to read.
 *
 * @return OMNIA_OK on success, or error code on failure.
 */
static omnia_status_t adc_read_n_impl(omnia_adc_t dev, uint16_t* buf, size_t n)
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

/* -------------------------------------------------------------------------- */
/* STM32 port vtable                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Static vtable instance exported to the Omnia port core.
 *
 * Fields not supported in this prototype are left as NULL and will be
 * detected via omnia_port_has_*() feature helpers.
 */
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

  /* UART (async) */
  .uart_write        = stm32_uart_write_async_impl,
  .uart_read         = stm32_uart_read_async_impl,
  .uart_tx_busy      = stm32_uart_tx_busy_impl,
  .uart_rx_available = stm32_uart_rx_available_impl,

  /* Time */
  .delay_us = delay_us_impl,
  .millis   = millis_impl,

  /* Concurrency (not implemented in this prototype) */
  .mutex_create  = NULL,
  .mutex_lock    = NULL,
  .mutex_unlock  = NULL,
  .mutex_destroy = NULL,

  /* Memory (optional) */
  .malloc_fn = NULL,
  .free_fn   = NULL,

  /* Log (optional) */
  .log = NULL,

  .user_ctx = NULL,
};

/* -------------------------------------------------------------------------- */
/* Public STM32 port API                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize STM32 port bindings and register the Omnia vtable.
 *
 * This function:
 * - Binds external CubeMX handles (hadc1, huart3) to internal contexts.
 * - Arms UART RX interrupt for continuous reception.
 * - Registers STM32_PORT_VTABLE through omnia_port_register().
 *
 * The function is intended to be called once at boot time.
 */
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

/**
 * @brief Create an opaque GPIO handle for a given STM32 port/pin.
 *
 * The handle is backed by a small static pool. If the pool is exhausted,
 * this function returns NULL.
 *
 * @param port GPIO port pointer (GPIO_TypeDef*).
 * @param pin  GPIO pin mask (GPIO_PIN_x).
 *
 * @return Opaque omnia_gpio_t handle, or NULL if pool is full.
 */
omnia_gpio_t omnia_mkpin(void* port, uint16_t pin)
{
  if (gpio_pool_used >= OMNIA_STM32_MAX_PINS) return NULL;

  gpio_pool[gpio_pool_used].port = (GPIO_TypeDef*)port;
  gpio_pool[gpio_pool_used].pin  = pin;

  return (omnia_gpio_t)&gpio_pool[gpio_pool_used++];
}

/**
 * @brief Start ADC1 DMA circular sampling (ring-based acquisition).
 *
 * @return OMNIA_OK on success, or an error code if HAL startup fails.
 */
omnia_status_t omnia_port_stm32_adc1_dma_start(void)
{
  omnia_status_t st = stm32_adc_dma_start(&g_adc1_ctx);
  if (st != OMNIA_OK) {
    __BKPT(3);
  }
  return st;
}
