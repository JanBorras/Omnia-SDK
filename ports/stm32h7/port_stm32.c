// ports/stm32h755/port_stm32.c
#include "omnia_port.h"
#include "stm32h7xx_hal.h"

extern SPI_HandleTypeDef hspi1;

static omnia_status_t spi_tx_impl(omnia_spi_t bus, const uint8_t* buf, size_t n){
  (void)bus;
  return (HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, (uint16_t)n, 1000) == HAL_OK) ? OMNIA_OK : OMNIA_EIO;
}

static void delay_us_impl(uint32_t us){
  // si tens DWT us, o converteix a HAL_Delay(ms) quan >1000
  if (us >= 1000) HAL_Delay(us/1000);
  else { /* dwt cycle counter o busy-wait curt */ }
}

static const omnia_port_vtable_t V = {
  .gpio_write = /* HAL_GPIO_WritePin wrapper */,
  .gpio_read  = /* HAL_GPIO_ReadPin wrapper */,
  .gpio_mode  = /* HAL_GPIO_Init wrapper */,
  .spi_tx     = spi_tx_impl,
  .spi_tx16   = NULL,
  .spi_txrx   = /* HAL_SPI_TransmitReceive */,
  .delay_us   = delay_us_impl,
  .millis     = /* HAL_GetTick */,
  .mutex_create = NULL, .mutex_lock=NULL, .mutex_unlock=NULL, .mutex_destroy=NULL,
  .malloc_fn = NULL, .free_fn=NULL,
  .log = /* via ITM/UART o no-op */,
  .user_ctx = NULL
};

void omnia_port_stm32_init(void){ omnia_port_register(&V); }
