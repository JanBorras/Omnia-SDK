#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" { 
#endif

typedef enum { EDGE_GPIO_INPUT=0, EDGE_GPIO_OUTPUT=1 } edge_gpio_mode_t;

int edge_gpio_init(uint32_t pin, edge_gpio_mode_t mode);
int edge_gpio_write(uint32_t pin, int level);
int edge_gpio_read(uint32_t pin);

#ifdef __cplusplus
}
#endif
