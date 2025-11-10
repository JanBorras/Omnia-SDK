// include/omnia_port_ex.h
#ifndef OMNIA_PORT_EX_H
#define OMNIA_PORT_EX_H
#include "omnia_port.h"
#include "omnia_rtos.h"

typedef struct {
  const omnia_port_vtable_t* io;     // el de sempre
  const omnia_rtos_ops_t*    rtos;   // opcional
  uint32_t caps; // bitmask (ex: bit0=RTOS, bit1=SPI_DMA, bit2=ISR_safe_tx)
  void*     user_ctx;
} omnia_port_ex_t;

// Helpers (interns o públics, com prefereixis)
const omnia_rtos_ops_t* omnia_port_get_rtos(omnia_port_t* p);
uint32_t                omnia_port_caps(omnia_port_t* p);

#endif
