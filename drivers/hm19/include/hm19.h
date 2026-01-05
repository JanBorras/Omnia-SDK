#ifndef HM19_H
#define HM19_H

#include "omnia_port.h"
#include "omnia_uart.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  HM19_MODE_DATA = 0,
  HM19_MODE_AT
} hm19_mode_t;

typedef enum {
  HM19_AT_EOL_LF = 0,   // "\n"
  HM19_AT_EOL_CRLF      // "\r\n"
} hm19_at_eol_t;

typedef enum {
  HM19_EN_ACTIVE_HIGH = 0,
  HM19_EN_ACTIVE_LOW
} hm19_en_polarity_t;

typedef enum {
  HM19_F_USE_MUTEX             = 1u << 0,
  HM19_F_DRAIN_RX_BEFORE_AT    = 1u << 1,
  HM19_F_CHUNK_TX              = 1u << 2,
  HM19_F_CHECK_CONNECTED       = 1u << 3,
  HM19_F_DROP_IF_NOT_CONNECTED = 1u << 4,
} hm19_flags_t;

typedef struct {
  uint32_t flags;

  hm19_at_eol_t      at_eol;         // default: HM19_AT_EOL_CRLF
  hm19_en_polarity_t en_pol;         // default: HM19_EN_ACTIVE_HIGH
  uint32_t           boot_delay_ms;  // default: 100

  uint32_t           drain_budget_ms; // default: 20

  uint16_t           tx_chunk_size;   // default: 64

  uint16_t           max_at_line;     // default: 128
} hm19_config_t;

typedef struct {
  const omnia_uart_handle_t* uart;
  omnia_gpio_t               pin_en;
  omnia_gpio_t               pin_state;

  hm19_mode_t                mode;
  hm19_config_t              cfg;

  omnia_mutex_t              lock;
} hm19_t;

omnia_status_t hm19_init(hm19_t* dev,
                         const omnia_uart_handle_t* uart,
                         omnia_gpio_t pin_en,
                         omnia_gpio_t pin_state,
                         const hm19_config_t* cfg);

void hm19_deinit(hm19_t* dev);

void hm19_set_at_eol(hm19_t* dev, hm19_at_eol_t eol);
void hm19_set_en_polarity(hm19_t* dev, hm19_en_polarity_t pol);
void hm19_set_boot_delay_ms(hm19_t* dev, uint32_t ms);
void hm19_set_flags(hm19_t* dev, uint32_t flags);
void hm19_set_tx_chunk(hm19_t* dev, uint16_t chunk_size);
void hm19_set_drain_budget(hm19_t* dev, uint32_t budget_ms);

int hm19_is_connected(hm19_t* dev);

omnia_status_t hm19_send_raw(hm19_t* dev,
                             const uint8_t* data,
                             size_t len,
                             uint32_t timeout_ms);

omnia_status_t hm19_send_line(hm19_t* dev,
                              const char* line,
                              uint32_t timeout_ms);

omnia_status_t hm19_send_at(hm19_t* dev,
                            const char* cmd,
                            char* resp,
                            size_t resp_size,
                            uint32_t timeout_ms);

static omnia_status_t hm19_send_at_raw_noeol(hm19_t* dev,
                                            const char* cmd,
                                            char* resp,
                                            size_t resp_size,
                                            uint32_t timeout_ms);

int hm19_send_at_multi(hm19_t* dev,
                       const char* cmd,
                       char* out_buf,
                       size_t out_size,
                       uint8_t max_lines,
                       uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // HM19_H
