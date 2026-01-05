#include "app_emg_stream.h"
#include "omnia_port.h"
#include <string.h>

/* converteix volts -> raw aproximat, per omplir baseline_raw del CFG */
static uint16_t volts_to_raw(const omnia_adc_handle_t* adc, float volts)
{
  if (!adc || adc->resolution_bits == 0) return 0;

  const uint32_t max_code = (1u << adc->resolution_bits) - 1u;
  const float vref = (float)adc->vref_mv / 1000.0f;
  if (vref <= 0.0f) return 0;

  float x = volts / vref;
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  uint32_t raw = (uint32_t)(x * (float)max_code + 0.5f);
  if (raw > max_code) raw = max_code;
  return (uint16_t)raw;
}

static void build_cfg_from_state(app_emg_stream_t* s)
{
  /* Aquest CFG és el “context” del stream perquè el PC pugui interpretar raw */
  memset(&s->cfg_cache, 0, sizeof(s->cfg_cache));

  s->cfg_cache.resolution_bits = s->emg->sen.adc.resolution_bits;
  s->cfg_cache.vref_mv         = (uint16_t)s->emg->sen.adc.vref_mv;
  s->cfg_cache.adc_channel     = s->adc_channel;
  s->cfg_cache.sample_rate_hz  = s->sample_rate_hz;

  /* baseline_raw: derivat de v_offset (en volts) */
  s->cfg_cache.baseline_raw    = volts_to_raw(&s->emg->sen.adc, s->emg->sen.v_offset);

  /* gain_q15: opcional. Si no el fas servir al PC, deixa 0 */
  s->cfg_cache.gain_q15        = 0;
}

static void try_send_cfg(app_emg_stream_t* s, uint32_t time_ms)
{
  if (!s->cfg_pending) return;

  size_t n = emg_build_cfg_packet(s->seq++, time_ms,
                                 &s->cfg_cache,
                                 s->tx_buf, sizeof(s->tx_buf));
  if (n == 0) return;

  omnia_status_t st = omnia_uart_write(&s->uart, s->tx_buf, n);
  if (st == OMNIA_OK) {
    s->cfg_pending = 0;
    s->sent_cfg++;
  }
  /* si EBUSY, no passa res: ho reintentarem al següent bloc */
}

static void send_raw_stream(app_emg_stream_t* s,
                            const uint16_t* raw,
                            size_t n,
                            uint32_t time_ms)
{
  /* El teu protocol limita a 32 samples per paquet.
     Si el bloc n > 32, el tallem en “chunks”.
   */
  while (n > 0) {
    uint8_t ns = (n > EMG_PKT_MAX_SAMPLES) ? (uint8_t)EMG_PKT_MAX_SAMPLES : (uint8_t)n;

    size_t pkt_n = emg_build_raw_u16_packet(s->seq++, time_ms,
                                           raw, ns,
                                           s->tx_buf, sizeof(s->tx_buf));
    if (pkt_n == 0) return;

    omnia_status_t st = omnia_uart_write(&s->uart, s->tx_buf, pkt_n);
    if (st == OMNIA_OK) {
      s->sent_raw++;
    } else if (st == OMNIA_EBUSY) {
      /* stream: drop i segueix, perquè no bloquegi mai */
      s->dropped_raw++;
    } else {
      /* OMNIA_EIO -> aquí pots comptar errors, BKPT, etc. */
    }

    raw += ns;
    n   -= ns;
  }
}

void app_emg_stream_init(app_emg_stream_t* s,
                         app_emg_t* emg,
                         app_plot_t* plot,
                         const omnia_uart_handle_t* uart,
                         uint16_t sample_rate_hz,
                         uint8_t adc_channel)
{
  if (!s || !emg || !uart) return;

  memset(s, 0, sizeof(*s));
  s->emg           = emg;
  s->plot          = plot;
  s->uart          = *uart;
  s->sample_rate_hz= sample_rate_hz;
  s->adc_channel   = adc_channel;

  s->seq           = 0;

  /* UI: 30 Hz és raonable; canvia-ho si vols */
  s->plot_period_ms = 33;
  s->last_plot_ms   = 0;

  /* Envia CFG al boot (pending fins que UART el pugui acceptar) */
  build_cfg_from_state(s);
  s->cfg_pending = 1;
}

void app_emg_stream_request_cfg(app_emg_stream_t* s, uint32_t time_ms)
{
  if (!s) return;
  build_cfg_from_state(s);
  s->cfg_pending = 1;
  try_send_cfg(s, time_ms);
}

void app_emg_stream_start_calibration(app_emg_stream_t* s)
{
  if (!s || !s->emg) return;

  app_emg_start_calibration(s->emg);

  /* UI immediata (si tens plot) */
  if (s->plot) {
    app_plot_draw_info(s->plot, 0, 0.0f, 0.0f, 0.0f, 0, 1, 0);
  }

  /* CFG: durant calibratge també pot ser útil enviar-lo (baseline encara no) */
  build_cfg_from_state(s);
  s->cfg_pending = 1;
}

void app_emg_stream_process_block(app_emg_stream_t* s,
                                  const uint16_t* raw,
                                  size_t n,
                                  uint32_t time_ms)
{
  if (!s || !s->emg || !raw || n == 0) return;

  /* 1) Processa EMG (baseline / norm / stats) */
  /* Abans de processar, guardem estat per detectar “acabo d’obtenir baseline” */
  uint8_t had_baseline_before = (uint8_t)app_emg_has_baseline(s->emg);

  app_emg_process_block_raw(s->emg, raw, n);

  uint8_t has_baseline_now = (uint8_t)app_emg_has_baseline(s->emg);
  uint8_t is_cal = (uint8_t)app_emg_is_calibrating(s->emg);

  /* 2) Si acabem de sortir de calibratge i ara tenim baseline:
        envia CFG actualitzat (baseline_raw real) */
  if (!had_baseline_before && has_baseline_now) {
    build_cfg_from_state(s);
    s->cfg_pending = 1;
  }

  /* 3) Intenta enviar CFG si està pending (no bloqueja) */
  try_send_cfg(s, time_ms);

  /* 4) Stream RAW cap al PC (sempre, fins i tot calibrant si vols dataset de repòs) */
  send_raw_stream(s, raw, n, time_ms);

  /* 5) UI (a ritme humà) */
  if (s->plot) {
    /* draw() és car: limitem a plot_period_ms */
    if ((time_ms - s->last_plot_ms) >= s->plot_period_ms) {
      s->last_plot_ms = time_ms;

      if (has_baseline_now && !is_cal) {
        app_plot_draw(s->plot, s->emg->norm, s->emg->saturated);
      }

      /* draw_info() ja té throttle intern (info_period_ms) */
      app_plot_draw_info(s->plot,
                         s->emg->raw,
                         s->emg->volts,
                         s->emg->centered,
                         s->emg->norm,
                         s->emg->saturated,
                         is_cal ? 1 : 0,
                         has_baseline_now ? 1 : 0);
    }
  }
}
