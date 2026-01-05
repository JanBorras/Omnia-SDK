#ifndef APP_EMG_STREAM_H
#define APP_EMG_STREAM_H

#include <stdint.h>
#include <stddef.h>

#include "app_emg.h"
#include "app_plot.h"
#include "emg_packet.h"
#include "omnia_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  app_emg_t*           emg;
  app_plot_t*          plot;      // pot ser NULL si no vols UI
  omnia_uart_handle_t  uart;

  uint16_t             sample_rate_hz;
  uint8_t              adc_channel;     // informatiu

  /* seq de protocol */
  uint8_t              seq;

  /* CFG retry */
  uint8_t              cfg_pending;
  emg_cfg_payload_t    cfg_cache;

  /* TX buf temporal (construcció paquet) */
  uint8_t              tx_buf[256];

  /* UI throttling extra (a part del throttle intern d'app_plot) */
  uint32_t             plot_period_ms;
  uint32_t             last_plot_ms;

  /* Stats de stream */
  uint32_t             dropped_raw;
  uint32_t             sent_raw;
  uint32_t             sent_cfg;

} app_emg_stream_t;

void app_emg_stream_init(app_emg_stream_t* s,
                         app_emg_t* emg,
                         app_plot_t* plot,
                         const omnia_uart_handle_t* uart,
                         uint16_t sample_rate_hz,
                         uint8_t adc_channel);

/* Crida-ho quan hi ha flanc de botó CAL */
void app_emg_stream_start_calibration(app_emg_stream_t* s);

/* Crida-ho a cada bloc ADC (n = 16/32...) */
void app_emg_stream_process_block(app_emg_stream_t* s,
                                  const uint16_t* raw,
                                  size_t n,
                                  uint32_t time_ms);

/* Força enviament CFG (p.ex. a boot, o quan vols refrescar context al PC) */
void app_emg_stream_request_cfg(app_emg_stream_t* s, uint32_t time_ms);

#ifdef __cplusplus
}
#endif
#endif
