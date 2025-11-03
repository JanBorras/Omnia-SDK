#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" { 
#endif

typedef struct {
  uint8_t bus;     // logical bus id
  uint32_t freq_hz; // 100k / 400k
} edge_i2c_cfg_t;

int edge_i2c_init(edge_i2c_cfg_t cfg);
int edge_i2c_write(uint8_t bus, uint8_t addr7, const uint8_t* data, uint16_t len);
int edge_i2c_read(uint8_t bus, uint8_t addr7, uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif
