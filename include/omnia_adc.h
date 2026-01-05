#ifndef OMNIA_ADC_H
#define OMNIA_ADC_H

#include "omnia_port.h"

#ifdef __cplusplus
extern "C" {
#endif

// Descripció lògica d’un canal ADC dins del SDK
typedef struct {
  omnia_adc_t  dev;             // Handle opac del port (p.ex. ADC_HandleTypeDef*)
  uint8_t      resolution_bits; // p.ex. 12 per STM32
  uint32_t     vref_mv;         // referència en mV (p.ex. 3300)
} omnia_adc_handle_t;

// Llegeix una mostra RAW via vtable
static inline omnia_status_t
omnia_adc_read_raw(const omnia_adc_handle_t* h, uint16_t* out_raw)
{
  if (!h || !out_raw) {
    return OMNIA_EINVAL;
  }

  omnia_port_t* port = omnia_port_get();
  if (!port || !port->v || !port->v->adc_read) {
    return OMNIA_ENOTSUP;  // el port actual no dona servei d'ADC
  }

  return port->v->adc_read(h->dev, out_raw);
}

// Converteix RAW -> volts (float)
// Nota: fem servir mV per no obligar a floats a tot arreu si no cal.
static inline float
omnia_adc_raw_to_volts(const omnia_adc_handle_t* h, uint16_t raw)
{
  if (!h || h->resolution_bits == 0) {
    return 0.0f;
  }

  uint32_t max_code = (1u << h->resolution_bits) - 1u;
  float vref = (float)h->vref_mv / 1000.0f;

  return ((float)raw / (float)max_code) * vref;
}

// Helper: RAW directament a volts a un sol call
static inline omnia_status_t
omnia_adc_read_volts(const omnia_adc_handle_t* h, float* out_volts)
{
  if (!h || !out_volts) {
    return OMNIA_EINVAL;
  }

  uint16_t raw = 0;
  omnia_status_t st = omnia_adc_read_raw(h, &raw);
  if (st != OMNIA_OK) {
    return st;
  }

  *out_volts = omnia_adc_raw_to_volts(h, raw);
  return OMNIA_OK;
}

// Llegeix N mostres RAW via vtable (si està implementat)
static inline omnia_status_t
omnia_adc_read_raw_n(const omnia_adc_handle_t* h, uint16_t* buf, size_t n)
{
  if (!h || !buf || n == 0) return OMNIA_EINVAL;

  omnia_port_t* port = omnia_port_get();
  if (!port || !port->v || !port->v->adc_read_n) return OMNIA_ENOTSUP;

  return port->v->adc_read_n(h->dev, buf, n);
}


#ifdef __cplusplus
}
#endif
#endif // OMNIA_ADC_H
