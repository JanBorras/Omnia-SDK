/*
 * STM32H7 HAL binding (skeleton).
 * In a real project, include "stm32h7xx_hal.h" and configure GPIO/I2C handles.
 */
#include "edge/hal/gpio.h"
#include "edge/hal/i2c.h"
#include "edge/hal/time.h"

int edge_gpio_init(uint32_t pin, edge_gpio_mode_t mode){
  // TODO: HAL_GPIO_Init(...)
  (void)pin; (void)mode; return 0;
}

int edge_gpio_write(uint32_t pin, int level){
  // TODO: HAL_GPIO_WritePin(...)
  (void)pin; (void)level; return 0;
}

int edge_gpio_read(uint32_t pin){
  // TODO: HAL_GPIO_ReadPin(...)
  (void)pin; return 0;
}

int edge_i2c_init(edge_i2c_cfg_t cfg){
  // TODO: MX_I2C_Init with timing derived from cfg.freq_hz
  (void)cfg; return 0;
}

int edge_i2c_write(uint8_t bus, uint8_t addr7, const uint8_t* data, uint16_t len){
  // TODO: HAL_I2C_Master_Transmit(...)
  (void)bus;(void)addr7;(void)data;(void)len; return 0;
}

int edge_i2c_read(uint8_t bus, uint8_t addr7, uint8_t* data, uint16_t len){
  // TODO: HAL_I2C_Master_Receive(...)
  (void)bus;(void)addr7;(void)data;(void)len; return 0;
}

void edge_delay_ms(uint32_t ms){
  // TODO: HAL_Delay(ms)
  (void)ms;
}

uint64_t edge_millis(void){
  // TODO: tick counter
  return 0;
}
