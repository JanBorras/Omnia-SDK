// ports/esp32/port_esp32.c
#include "omnia_port.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static omnia_status_t spi_tx_impl(omnia_spi_t bus, const uint8_t* buf, size_t n){
  spi_device_handle_t dev = (spi_device_handle_t)bus;
  spi_transaction_t t = { .length = n * 8, .tx_buffer = buf };
  return (spi_device_transmit(dev, &t) == ESP_OK) ? OMNIA_OK : OMNIA_EIO;
}

static void delay_us_impl(uint32_t us){ ets_delay_us(us); }

static void* malloc_impl(size_t n){ return heap_caps_malloc(n, MALLOC_CAP_8BIT); }
static void  free_impl(void* p){ heap_caps_free(p); }

static const omnia_port_vtable_t V = {
  .gpio_write = /* gpio_set_level */,
  .gpio_read  = /* gpio_get_level */,
  .gpio_mode  = /* gpio_config */,
  .spi_tx     = spi_tx_impl,
  .spi_tx16   = NULL, .spi_txrx = NULL,
  .delay_us   = delay_us_impl,
  .millis     = /* esp_timer_get_time()/1000 */,
  .mutex_create = /* xSemaphoreCreateMutex */, .mutex_lock=/* xSemaphoreTake */, .mutex_unlock=/* xSemaphoreGive */, .mutex_destroy=/* vSemaphoreDelete */,
  .malloc_fn = malloc_impl, .free_fn = free_impl,
  .log = /* ESP_LOGx wrapper */,
  .user_ctx = NULL
};

void omnia_port_esp32_init(void){ omnia_port_register(&V); }
