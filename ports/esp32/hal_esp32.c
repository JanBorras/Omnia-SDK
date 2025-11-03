/*
 * ESP32 HAL binding (skeleton).
 * In a real project, include <driver/gpio.h>, <driver/i2c.h>, <esp_timer.h>, etc.
 */
#include "edge/hal/gpio.h"
#include "edge/hal/i2c.h"
#include "edge/hal/time.h"

int edge_gpio_init(uint32_t pin, edge_gpio_mode_t mode){
  // TODO: map to ESP-IDF gpio_config + direction
  (void)pin; (void)mode; return 0;
}

int edge_gpio_write(uint32_t pin, int level){
  // TODO: gpio_set_level(pin, level)
  (void)pin; (void)level; return 0;
}

int edge_gpio_read(uint32_t pin){
  // TODO: gpio_get_level(pin)
  (void)pin; return 0;
}

int edge_i2c_init(edge_i2c_cfg_t cfg){
  // TODO: create I2C driver with cfg.bus and cfg.freq_hz
  (void)cfg; return 0;
}

int edge_i2c_write(uint8_t bus, uint8_t addr7, const uint8_t* data, uint16_t len){
  // TODO: i2c_master_write_to_device(...)
  (void)bus;(void)addr7;(void)data;(void)len; return 0;
}

int edge_i2c_read(uint8_t bus, uint8_t addr7, uint8_t* data, uint16_t len){
  // TODO: i2c_master_read_from_device(...)
  (void)bus;(void)addr7;(void)data;(void)len; return 0;
}

void edge_delay_ms(uint32_t ms){
  // TODO: vTaskDelay(pdMS_TO_TICKS(ms)) or ets_delay_us
  (void)ms;
}

uint64_t edge_millis(void){
  // TODO: esp_timer_get_time()/1000
  return 0;
}
