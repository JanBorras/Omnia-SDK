// core/omnia_port_default.c
#include "omnia_port.h"
#include <string.h>

static omnia_port_t g_port;
static int g_inited = 0;

omnia_status_t omnia_port_register(const omnia_port_vtable_t* v) {
  if (!v) return OMNIA_EINVAL;
  g_port.v = v;
  g_inited = 1;
  return OMNIA_OK;
}
omnia_port_t* omnia_port_get(void) {
  return g_inited ? &g_port : NULL;
}
