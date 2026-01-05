// include/omnia_port.h
#ifndef OMNIA_PORT_H
#define OMNIA_PORT_H

#include "omnia_types.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Pins opacs (la plataforma decideix què contenen)
typedef void* omnia_gpio_t;
typedef void* omnia_spi_t;
typedef void* omnia_mutex_t;
typedef void* omnia_adc_t;
typedef void* omnia_uart_t;

typedef enum {
  OMNIA_GPIO_PULL_NONE = 0,
  OMNIA_GPIO_PULL_UP   = 1,
  OMNIA_GPIO_PULL_DOWN = 2,
} omnia_gpio_pull_t;

/*
 * Contracte del port (vtable)
 *
 * UART (async always):
 *  - uart_write(): NO bloqueja mai.
 *      OMNIA_OK     => encolat/enviant
 *      OMNIA_EBUSY  => cua plena / TX ocupat (no acceptat)
 *      OMNIA_EIO    => error HW
 *
 *  - uart_read(): NO bloqueja mai.
 *      OMNIA_OK     => out_nread > 0
 *      OMNIA_EBUSY  => no hi ha dades disponibles
 *      OMNIA_EIO    => error HW
 *
 * ADC:
 *  - adc_read()/adc_read_n() poden ser blocking (DMA ring / polling)
 */
typedef struct {
  /* GPIO */
  omnia_status_t (*gpio_write)(omnia_gpio_t pin, int level);
  int            (*gpio_read)(omnia_gpio_t pin);
  omnia_status_t (*gpio_mode)(omnia_gpio_t pin, int is_output, omnia_gpio_pull_t pull);

  /* SPI */
  omnia_status_t (*spi_tx)(omnia_spi_t bus, const uint8_t* buf, size_t n);
  omnia_status_t (*spi_tx16)(omnia_spi_t bus, const uint16_t* buf, size_t n);
  omnia_status_t (*spi_txrx)(omnia_spi_t bus, const uint8_t* tx, uint8_t* rx, size_t n);

  /* ADC */
  omnia_status_t (*adc_read)(omnia_adc_t dev, uint16_t* out);
  omnia_status_t (*adc_read_n)(omnia_adc_t dev, uint16_t* buf, size_t n);

  /* UART (async always) */
  omnia_status_t (*uart_write)(omnia_uart_t uart, const uint8_t* data, size_t len);
  omnia_status_t (*uart_read)(omnia_uart_t uart, uint8_t* data, size_t len, size_t* out_nread);

  /* UART status helpers */
  int            (*uart_tx_busy)(omnia_uart_t uart);        /* 0/1 */
  size_t         (*uart_rx_available)(omnia_uart_t uart);   /* bytes available */

  /* Time */
  void           (*delay_us)(uint32_t us);
  uint64_t       (*millis)(void);

  /* Concurrency (optional) */
  omnia_mutex_t  (*mutex_create)(void);
  void           (*mutex_lock)(omnia_mutex_t m);
  void           (*mutex_unlock)(omnia_mutex_t m);
  void           (*mutex_destroy)(omnia_mutex_t m);

  /* Memory (optional) */
  void*          (*malloc_fn)(size_t);
  void           (*free_fn)(void*);

  /* Log (optional) */
  void           (*log)(int level, const char* fmt, ...);

  /* Platform-specific context */
  void*          user_ctx;

} omnia_port_vtable_t;

/* Handle del port en temps d’execució */
struct omnia_port_s {
  const omnia_port_vtable_t* v;
};

/* API pública */
omnia_status_t omnia_port_register(const omnia_port_vtable_t* v);
omnia_status_t omnia_port_validate(const omnia_port_vtable_t* v);
omnia_port_t*  omnia_port_get(void);

int omnia_port_has_adc(void);
int omnia_port_has_spi(void);
int omnia_port_has_uart(void);
int omnia_port_has_mutex(void);
int omnia_port_has_malloc(void);
int omnia_port_has_log(void);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_PORT_H */
