#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" { 
#endif

void edge_delay_ms(uint32_t ms);
uint64_t edge_millis(void); // uptime in ms

#ifdef __cplusplus
}
#endif
