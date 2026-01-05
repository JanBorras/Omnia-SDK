#ifndef OMNIA_UART_H
#define OMNIA_UART_H

#include "omnia_port.h"

#ifdef __cplusplus
extern "C" {
#endif

// Descriptor lògic d’un canal UART dins el SDK
typedef struct {
  omnia_uart_t dev;     // Handle opac del port (p.ex. UART_HandleTypeDef*)
  uint32_t     baud;    // Baudrate configurat (informatiu)
} omnia_uart_handle_t;

static inline omnia_status_t
omnia_uart_write(const omnia_uart_handle_t* h, const uint8_t* data, size_t len)
{
  if (!h || !data || len == 0) return OMNIA_EINVAL;
  omnia_port_t* port = omnia_port_get();
  if (!port || !port->v || !port->v->uart_write) return OMNIA_ENOTSUP;
  return port->v->uart_write(h->dev, data, len);
}

static inline omnia_status_t
omnia_uart_read(const omnia_uart_handle_t* h, uint8_t* data, size_t len, size_t* out_nread)
{
  if (!h || !data || len == 0 || !out_nread) return OMNIA_EINVAL;
  *out_nread = 0;
  omnia_port_t* port = omnia_port_get();
  if (!port || !port->v || !port->v->uart_read) return OMNIA_ENOTSUP;
  return port->v->uart_read(h->dev, data, len, out_nread);
}

static inline int
omnia_uart_tx_busy(const omnia_uart_handle_t* h)
{
  if (!h) return 1;
  omnia_port_t* port = omnia_port_get();
  if (!port || !port->v || !port->v->uart_tx_busy) return 1;
  return port->v->uart_tx_busy(h->dev);
}

static inline size_t
omnia_uart_rx_available(const omnia_uart_handle_t* h)
{
  if (!h) return 0;
  omnia_port_t* port = omnia_port_get();
  if (!port || !port->v || !port->v->uart_rx_available) return 0;
  return port->v->uart_rx_available(h->dev);
}

#ifdef __cplusplus
}
#endif
#endif // OMNIA_UART_H
