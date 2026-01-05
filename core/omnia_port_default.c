#include "omnia_port.h"

/*
 * Nucli del contracte de plataforma (Omnia Port):
 * - Manté un únic port actiu (seleccionat a build-time via enllaç/CMake)
 * - Registra la vtable a runtime (binding trivial d’inicialització)
 * - Proporciona helpers de capacitats per a drivers/apps
 */

static omnia_port_t g_port;
static int g_inited = 0;

omnia_status_t omnia_port_validate(const omnia_port_vtable_t* v)
{
  if (!v) return OMNIA_EINVAL;

  /*
   * Mínim obligatori per considerar el port “usable”:
   * - GPIO write/mode: permet control bàsic de perifèrics (CS/DC/RST, LEDs, etc.)
   * - delay_us + millis: sincronització i temporització portables
   *
   * Tot el que no és universal (SPI/UART/ADC, concurrència, heap, logging) és opcional.
   */
  if (!v->gpio_write) return OMNIA_EINVAL;
  if (!v->gpio_mode)  return OMNIA_EINVAL;
  if (!v->delay_us)   return OMNIA_EINVAL;
  if (!v->millis)     return OMNIA_EINVAL;

  return OMNIA_OK;
}

omnia_status_t omnia_port_register(const omnia_port_vtable_t* v)
{
  if (!v) return OMNIA_EINVAL;

  omnia_status_t st = omnia_port_validate(v);
  if (st != OMNIA_OK) return st;

  g_port.v = v;
  g_inited = 1;
  return OMNIA_OK;
}

omnia_port_t* omnia_port_get(void)
{
  return g_inited ? &g_port : NULL;
}

/* ---------- Helpers de capacitats (feature detection) ---------- */

static const omnia_port_vtable_t* port_v(void)
{
  omnia_port_t* p = omnia_port_get();
  return (p && p->v) ? p->v : NULL;
}

int omnia_port_has_adc(void)
{
  const omnia_port_vtable_t* v = port_v();
  /* ADC “present” si almenys es pot llegir una mostra */
  return (v && v->adc_read != NULL);
}

int omnia_port_has_spi(void)
{
  const omnia_port_vtable_t* v = port_v();
  /* SPI “present” si hi ha TX bàsic */
  return (v && v->spi_tx != NULL);
}

int omnia_port_has_uart(void)
{
  const omnia_port_vtable_t* v = port_v();
  /* UART “present” si hi ha read + write (contracte async) */
  return (v && v->uart_write != NULL && v->uart_read != NULL);
}

int omnia_port_has_mutex(void)
{
  const omnia_port_vtable_t* v = port_v();
  /* Concurrència “present” només si el conjunt és complet */
  return (v &&
          v->mutex_create  != NULL &&
          v->mutex_lock    != NULL &&
          v->mutex_unlock  != NULL &&
          v->mutex_destroy != NULL);
}

int omnia_port_has_malloc(void)
{
  const omnia_port_vtable_t* v = port_v();
  /* Heap “present” només si malloc+free existeixen */
  return (v && v->malloc_fn != NULL && v->free_fn != NULL);
}

int omnia_port_has_log(void)
{
  const omnia_port_vtable_t* v = port_v();
  return (v && v->log != NULL);
}
