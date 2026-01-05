#include "emg_stream.h"
#include "omnia_port.h"
#include <string.h>

/* -------- time helpers -------- */

static uint64_t now_ms(void)
{
  omnia_port_t* p = omnia_port_get();
  if (!p || !p->v || !p->v->millis) return 0;
  return p->v->millis();
}

static uint32_t advance_time_ms(const emg_stream_t* s, uint32_t base_ms, uint32_t n_samples)
{
  if (!s || s->cfg.sample_period_us == 0) return base_ms;
  uint64_t us = (uint64_t)n_samples * (uint64_t)s->cfg.sample_period_us;
  return base_ms + (uint32_t)(us / 1000u);
}

/* -------- ring helpers -------- */

static inline uint32_t ring_next(uint32_t i, uint32_t cap) { return (i + 1u) % cap; }

static int ring_push(emg_stream_t* s, uint16_t v)
{
  if (!s || !s->buf || s->cap == 0) return 0;

  if (s->count >= s->cap) {
    if (!s->cfg.drop_oldest_on_full) {
      s->dropped_samples++;
      return 0; // drop newest
    }

    // drop oldest
    s->tail = ring_next(s->tail, s->cap);
    s->count--;
    s->dropped_samples++;

    // IMPORTANT: si hem tret 1 mostra del front, t0 ha d’avançar 1 mostra
    if (s->t0_valid) {
      s->t0_ms = advance_time_ms(s, s->t0_ms, 1u);
    }
  }

  s->buf[s->head] = v;
  s->head = ring_next(s->head, s->cap);
  s->count++;
  return 1;
}

static int ring_pop(emg_stream_t* s, uint16_t* out)
{
  if (!s || !out || s->count == 0) return 0;
  *out = s->buf[s->tail];
  s->tail = ring_next(s->tail, s->cap);
  s->count--;
  return 1;
}

/* -------- tx policy -------- */

static int connected_now(emg_stream_t* s)
{
  if (!s || !s->ble) return -1;
  return hm19_is_connected(s->ble); // 1/0/-1
}

static int should_send_now(emg_stream_t* s)
{
  if (!s || !s->ble) return 0;

  if (!s->cfg.require_connected) return 1;

  int c = connected_now(s);
  return (c == 1) ? 1 : 0;
}

/* -------- packet senders -------- */

static int try_send_cfg(emg_stream_t* s, uint32_t t_ms)
{
  if (!s || !s->ble) return 0;
  if (!s->cfg_valid) return 0;

  uint8_t tx_buf[EMG_PKT_COMMON_HDR_LEN + EMG_PKT_CFG_PAYLOAD_LEN + EMG_PKT_CRC_LEN];

  size_t pkt_len = emg_build_cfg_packet(s->seq, t_ms, &s->cfg_payload, tx_buf, sizeof(tx_buf));
  if (pkt_len == 0) {
    s->cfg_errors++;
    return 0;
  }

  omnia_status_t st = hm19_send_raw(s->ble, tx_buf, pkt_len, s->cfg.tx_timeout_ms);
  if (st == OMNIA_OK) {
    s->seq++;
    s->cfg_sent = 1;
    s->sent_cfg++;
    return 1;
  }

  s->cfg_errors++;
  return 0;
}

static int try_send_raw_frame(emg_stream_t* s, const uint16_t* frame, uint8_t n, uint32_t t_ms)
{
  if (!s || !s->ble || !frame || n == 0) return 0;

  uint8_t tx_buf[EMG_PKT_COMMON_HDR_LEN + EMG_PKT_RAW_HDR_LEN + (EMG_PKT_MAX_SAMPLES * 2u) + EMG_PKT_CRC_LEN];

  size_t pkt_len = emg_build_raw_u16_packet(s->seq, t_ms, frame, n, tx_buf, sizeof(tx_buf));
  if (pkt_len == 0) {
    s->tx_errors++;
    return 0;
  }

  omnia_status_t st = hm19_send_raw(s->ble, tx_buf, pkt_len, s->cfg.tx_timeout_ms);
  if (st == OMNIA_OK) {
    s->seq++;
    s->sent_packets++;
    return 1;
  }

  s->tx_errors++;
  return 0;
}

/* -------- queue frame ops -------- */

static void enqueue_frame(emg_stream_t* s, const uint16_t* frame, uint8_t n, uint32_t t_ms_first)
{
  if (!s || !frame || n == 0) return;

  if (s->count == 0) {
    s->t0_ms = t_ms_first;
    s->t0_valid = 1;
  } else if (!s->t0_valid) {
    s->t0_ms = t_ms_first;
    s->t0_valid = 1;
  }

  for (uint8_t i = 0; i < n; ++i) {
    (void)ring_push(s, frame[i]);
  }

  s->enqueued_frames++;
}

static uint8_t dequeue_frame(emg_stream_t* s, uint16_t* out_frame, uint32_t* out_t_ms_first)
{
  if (!s || !out_frame || s->count == 0) return 0;

  uint32_t t_first = s->t0_ms;
  if (out_t_ms_first) *out_t_ms_first = t_first;

  uint8_t want = s->cfg.frame_samples;
  if (want == 0) want = 16;
  if (want > EMG_PKT_MAX_SAMPLES) want = EMG_PKT_MAX_SAMPLES;

  uint8_t n = 0;
  while (n < want) {
    uint16_t v;
    if (!ring_pop(s, &v)) break;
    out_frame[n++] = v;
  }

  if (s->count == 0) {
    s->t0_valid = 0;
  } else {
    s->t0_ms = advance_time_ms(s, t_first, (uint32_t)n);
  }

  s->dequeued_frames++;
  return n;
}

/* -------- public API -------- */

emg_stream_err_t emg_stream_init(emg_stream_t* s,
                                 hm19_t* ble,
                                 const emg_stream_config_t* cfg,
                                 uint16_t* ring_buf_samples,
                                 uint32_t cap_samples)
{
  if (!s || !ble || !cfg || !ring_buf_samples || cap_samples == 0) return EMG_STREAM_EINVAL;
  if (cfg->frame_samples == 0 || cfg->frame_samples > EMG_PKT_MAX_SAMPLES) return EMG_STREAM_EINVAL;
  if (cfg->sample_rate_hz == 0 && cfg->sample_period_us == 0) return EMG_STREAM_EINVAL;

  memset(s, 0, sizeof(*s));
  s->ble = ble;
  s->cfg = *cfg;

  if (s->cfg.sample_period_us == 0) {
    s->cfg.sample_period_us = 1000000u / s->cfg.sample_rate_hz;
  }

  s->buf = ring_buf_samples;
  s->cap = cap_samples;
  s->head = s->tail = s->count = 0;
  s->t0_valid = 0;

  s->frame_len = 0;
  s->frame_t0_valid = 0;

  s->pending_valid = 0;

  s->seq = 0;

  s->cfg_valid = 0;
  s->cfg_sent = 0;
  s->last_cfg_try_ms = 0;

  s->last_connected = -1;

  return EMG_STREAM_OK;
}

void emg_stream_set_cfg(emg_stream_t* s, const emg_cfg_payload_t* cfg)
{
  if (!s || !cfg) return;
  s->cfg_payload = *cfg;
  s->cfg_valid = 1;
  s->cfg_sent = 0; // força reenviament
}

void emg_stream_update_baseline_raw(emg_stream_t* s, uint16_t baseline_raw, uint8_t force_resend)
{
  if (!s) return;
  s->cfg_payload.baseline_raw = baseline_raw;
  s->cfg_valid = 1;
  if (force_resend) s->cfg_sent = 0;
}

void emg_stream_push_sample(emg_stream_t* s, uint16_t sample_raw, uint32_t t_ms)
{
  if (!s) return;

  s->pushed_samples++;

  if (s->frame_len == 0) {
    s->frame_t0_ms = t_ms;
    s->frame_t0_valid = 1;
  }

  s->frame_tmp[s->frame_len++] = sample_raw;

  if (s->frame_len < s->cfg.frame_samples) {
    return;
  }

  uint8_t n = s->frame_len;
  s->frame_len = 0;

  uint32_t t_first = s->frame_t0_valid ? s->frame_t0_ms : t_ms;
  s->frame_t0_valid = 0;

  // No desordenis: si hi ha cua o pending, encola.
  if (s->count == 0 && !s->pending_valid && should_send_now(s)) {

    // Si volem CFG abans de RAW, assegurem-nos que s’ha enviat (si activat)
    if (s->cfg.send_cfg_on_connect && s->cfg_valid && !s->cfg_sent) {
      // prova enviar CFG (si falla, encola RAW igualment)
      (void)try_send_cfg(s, t_first);
    }

    if (try_send_raw_frame(s, s->frame_tmp, n, t_first)) {
      return;
    }
  }

  enqueue_frame(s, s->frame_tmp, n, t_first);
}

void emg_stream_push_block(emg_stream_t* s,
                           const uint16_t* samples_raw,
                           uint32_t n,
                           uint32_t t0_ms)
{
  if (!s || !samples_raw || n == 0) return;

  for (uint32_t i = 0; i < n; ++i) {
    uint32_t t_ms = advance_time_ms(s, t0_ms, i);
    emg_stream_push_sample(s, samples_raw[i], t_ms);
  }
}

/* Decideix si toca intentar enviar CFG ara:
 * - Si send_cfg_on_connect=1: quan detectes connectat (flanc) o si cfg_resend_ms expira.
 * - Si require_connected=0: també ho pots enviar sense STATE, però només si should_send_now()==1.
 */
static void maybe_send_cfg(emg_stream_t* s)
{
  if (!s) return;
  if (!s->cfg.send_cfg_on_connect) return;
  if (!s->cfg_valid) return;
  if (s->cfg_sent) return;

  int c = connected_now(s);
  uint32_t t_ms = (uint32_t)now_ms();

  // Detecta flanc a connectat (0->1 o -1->1)
  int rising = (c == 1) && (s->last_connected != 1);

  // Resend per temps
  int time_ok = 0;
  if (s->cfg.cfg_resend_ms != 0) {
    uint32_t last = s->last_cfg_try_ms;
    if (last == 0 || (t_ms >= last && (t_ms - last) >= s->cfg.cfg_resend_ms)) {
      time_ok = 1;
    }
  }

  s->last_connected = c;

  // Si exigeixes connectat: només tries quan c==1
  if (s->cfg.require_connected) {
    if (c != 1) return;
    if (!rising && !time_ok) return;
  } else {
    // no exigeix connectat: pots provar periòdicament igualment
    if (!rising && !time_ok) return;
  }

  s->last_cfg_try_ms = t_ms;
  (void)try_send_cfg(s, t_ms);
}

void emg_stream_tick(emg_stream_t* s)
{
  if (!s) return;

  // CFG pot anar abans del flush
  maybe_send_cfg(s);

  // Si exigeix connexió i no hi és, no fem flush
  if (s->cfg.require_connected) {
    int c = connected_now(s);
    s->last_connected = c;
    if (c != 1) return;
  }

  uint64_t t_start = now_ms();
  uint8_t sent_now = 0;

  // Pending primer
  if (s->pending_valid) {
    if (try_send_raw_frame(s, s->pending_frame, s->pending_len, s->pending_t_ms)) {
      s->pending_valid = 0;
    } else {
      return; // link mort: no toquis la cua
    }
  }

  while (s->count > 0) {

    if (s->cfg.max_flush_packets_per_tick && sent_now >= s->cfg.max_flush_packets_per_tick) {
      break;
    }

    if (s->cfg.flush_budget_ms) {
      uint64_t t_now = now_ms();
      if (t_now >= t_start && (t_now - t_start) >= s->cfg.flush_budget_ms) {
        break;
      }
    }

    uint32_t t_ms_first = 0;
    uint8_t n = dequeue_frame(s, s->pending_frame, &t_ms_first);
    if (n == 0) break;

    s->pending_len   = n;
    s->pending_t_ms  = t_ms_first;
    s->pending_valid = 1;

    // Abans d’enviar RAW, prova CFG si està pendent (no fatal)
    if (s->cfg.send_cfg_on_connect && s->cfg_valid && !s->cfg_sent) {
      (void)try_send_cfg(s, t_ms_first);
    }

    if (try_send_raw_frame(s, s->pending_frame, s->pending_len, s->pending_t_ms)) {
      s->pending_valid = 0;
      sent_now++;
      continue;
    }

    // falla: manté pending i surt (no duplica, no re-encola)
    return;
  }
}
