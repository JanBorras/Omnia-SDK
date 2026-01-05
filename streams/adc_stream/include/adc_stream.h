#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*adc_block_cb_t)(const uint16_t* samples, size_t n, void* user);

typedef struct {
  uint32_t       fs_hz;      // 2000
  size_t         block_n;    // 32 o 64
  adc_block_cb_t on_block;   // callback quan tens un bloc
  void*          user;
} adc_stream_cfg_t;

void adc_stream_init(const adc_stream_cfg_t* cfg);
int  adc_stream_start(void);
void adc_stream_stop(void);

#ifdef __cplusplus
}
#endif
