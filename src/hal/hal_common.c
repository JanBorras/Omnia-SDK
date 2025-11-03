#include <stdint.h>
#include <stdio.h>
#include "edge/hal/gpio.h"
#include "edge/hal/i2c.h"
#include "edge/hal/time.h"

// Provide weak fallbacks so the app links in host tests.
__attribute__((weak)) int edge_gpio_init(uint32_t pin, edge_gpio_mode_t mode){ (void)pin;(void)mode; return 0; }
__attribute__((weak)) int edge_gpio_write(uint32_t pin, int level){ (void)pin;(void)level; return 0; }
__attribute__((weak)) int edge_gpio_read(uint32_t pin){ (void)pin; return 0; }

__attribute__((weak)) int edge_i2c_init(edge_i2c_cfg_t cfg){ (void)cfg; return 0; }
__attribute__((weak)) int edge_i2c_write(uint8_t bus, uint8_t addr7, const uint8_t* data, uint16_t len){
  (void)bus;(void)addr7;(void)data;(void)len; return 0;
}
__attribute__((weak)) int edge_i2c_read(uint8_t bus, uint8_t addr7, uint8_t* data, uint16_t len){
  (void)bus;(void)addr7;(void)data;(void)len; return 0;
}

__attribute__((weak)) void edge_delay_ms(uint32_t ms){ (void)ms; }
__attribute__((weak)) uint64_t edge_millis(void){ return 0; }
