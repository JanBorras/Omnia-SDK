// core/omnia_port_ex.c (exemple)
#include "omnia_port_ex.h"

static omnia_port_ex_t g_port_ex;

omnia_status_t omnia_port_register_ex(const omnia_port_vtable_t* io,
                                      const omnia_rtos_ops_t* rtos,
                                      uint32_t caps, void* user_ctx)
{
  if (!io) return OMNIA_EINVAL;
  g_port_ex.io   = io;
  g_port_ex.rtos = rtos;   // pot ser NULL
  g_port_ex.caps = caps;
  g_port_ex.user_ctx = user_ctx;
  // també mantens la via antiga per compatibilitat:
  return omnia_port_register(io);
}

const omnia_rtos_ops_t* omnia_port_get_rtos(omnia_port_t* p){
  (void)p; return g_port_ex.rtos;
}
uint32_t omnia_port_caps(omnia_port_t* p){
  (void)p; return g_port_ex.caps;
}
