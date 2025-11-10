// ports/msp430fr2xx/port_msp430.c
#include "omnia_port.h"
#include <msp430.h>  // el món real és aquí, amagat del driver

static omnia_status_t gpio_write_impl(omnia_gpio_t pin, int level){
  // pin apunta a una estructura pròpia (p.ex. {volatile uint8_t* out; uint8_t mask;})
  // … escriu al registre OUT
  return OMNIA_OK;
}

static void delay_us_impl(uint32_t us){
  // pots basar-te en un timer o busy-wait calibrat
}

static omnia_status_t spi_tx_impl(omnia_spi_t bus, const uint8_t* buf, size_t n){
  // escriu a UCBxTXBUF i espera flags TXIFG/ RXIFG segons correspongui
  return OMNIA_OK;
}

// opcional: log amb UART o no-op
static void log_impl(int level, const char* fmt, ...){ (void)level;(void)fmt; }

static const omnia_port_vtable_t V = {
  .gpio_write = gpio_write_impl,
  .gpio_read  = NULL,         // si no cal, pot ser NULL i el driver no l’usarà
  .gpio_mode  = NULL,
  .spi_tx     = spi_tx_impl,
  .spi_tx16   = NULL,
  .spi_txrx   = NULL,
  .delay_us   = delay_us_impl,
  .millis     = NULL,
  .mutex_create = NULL, .mutex_lock=NULL, .mutex_unlock=NULL, .mutex_destroy=NULL,
  .malloc_fn  = NULL, .free_fn = NULL,
  .log        = log_impl,
  .user_ctx   = NULL
};

void omnia_port_msp430_init(void){
  omnia_port_register(&V);
}
