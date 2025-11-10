// include/omnia_port.h
#ifndef OMNIA_PORT_H
#define OMNIA_PORT_H

#include "omnia_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pins opacs (la plataforma decideix què contenen)
typedef void* omnia_gpio_t;
typedef void* omnia_spi_t;
typedef void* omnia_mutex_t;

typedef struct {
  // GPIO
  omnia_status_t (*gpio_write)(omnia_gpio_t pin, int level);
  int            (*gpio_read)(omnia_gpio_t pin);
  omnia_status_t (*gpio_mode)(omnia_gpio_t pin, int is_output, int pull);

  // SPI (8/16 bits via flags o paraules)
  omnia_status_t (*spi_tx)(omnia_spi_t bus, const uint8_t* buf, size_t n);
  omnia_status_t (*spi_tx16)(omnia_spi_t bus, const uint16_t* buf, size_t n);
  omnia_status_t (*spi_txrx)(omnia_spi_t bus, const uint8_t* tx, uint8_t* rx, size_t n);

  // Temps
  void           (*delay_us)(uint32_t us);
  uint64_t       (*millis)(void);

  // Concurrència (opcional)
  omnia_mutex_t  (*mutex_create)(void);
  void           (*mutex_lock)(omnia_mutex_t m);
  void           (*mutex_unlock)(omnia_mutex_t m);
  void           (*mutex_destroy)(omnia_mutex_t m);

  // Memòria (permets injectar allocs per RTOS/regions)
  void*          (*malloc_fn)(size_t);
  void           (*free_fn)(void*);

  // Log (nivells simples)
  void           (*log)(int level, const char* fmt, ...);

  // Context específic de plataforma si cal
  void* user_ctx;
} omnia_port_vtable_t;

// Handle del port en temps d’execució
struct omnia_port_s {
  const omnia_port_vtable_t* v;
};

// API pública
// 1) El port s’ha de registrar abans d’usar drivers
omnia_status_t omnia_port_register(const omnia_port_vtable_t* v);
// 2) Obtenir el port global (o variants si vols multi-port)
omnia_port_t*  omnia_port_get(void);

#ifdef __cplusplus
}
#endif
#endif
