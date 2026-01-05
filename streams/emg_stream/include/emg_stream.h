#ifndef EMG_STREAM_H
#define EMG_STREAM_H

#include <stdint.h>
#include <stddef.h>
#include "omnia_types.h"
#include "hm19.h"
#include "emg_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  EMG_STREAM_OK = 0,

  EMG_STREAM_EINVAL = -1,
  EMG_STREAM_ENOMEM = -2,
  EMG_STREAM_EIO    = -3,
} emg_stream_err_t;

typedef struct {
  // Sampling
  uint32_t sample_rate_hz;     // ex: 2000
  uint32_t sample_period_us;   // ex: 1000000 / sample_rate_hz

  // Framing
  uint8_t  frame_samples;      // ex: 16 (8..32)
  uint32_t tx_timeout_ms;      // ex: 20..50

  // Policy
  uint8_t  require_connected;  // si 1: només envia quan hm19_is_connected()==1

  // Flush budget (monocore friendly)
  uint8_t  max_flush_packets_per_tick; // ex: 2..8
  uint32_t flush_budget_ms;            // ex: 2..10 (0=ignore)

  // Ring full
  uint8_t  drop_oldest_on_full; // 0 drop newest, 1 drop oldest

  // CFG policy
  uint8_t  send_cfg_on_connect; // 1 = envia CFG quan detecta connectat
  uint32_t cfg_resend_ms;       // reintenta CFG cada X ms si no s’ha pogut (0=desactiva)

} emg_stream_config_t;

typedef struct {
  // Ring buffer RAW (mostres)
  uint16_t* buf;
  uint32_t  cap;
  uint32_t  head;
  uint32_t  tail;
  uint32_t  count;

  // Timing base de la cua (timestamp del sample a tail)
  uint32_t  t0_ms;
  uint32_t  t0_valid;

  // Frame builder (RAM calenta)
  uint16_t  frame_tmp[EMG_PKT_MAX_SAMPLES];
  uint8_t   frame_len;

  // Timestamp del primer sample del frame en construcció
  uint32_t  frame_t0_ms;
  uint8_t   frame_t0_valid;

  // Pending frame (flush robust)
  uint16_t  pending_frame[EMG_PKT_MAX_SAMPLES];
  uint8_t   pending_len;
  uint32_t  pending_t_ms;
  uint8_t   pending_valid;

  // Seq del protocol
  uint8_t   seq;

  // Driver BLE
  hm19_t*   ble;

  // Config
  emg_stream_config_t cfg;

  // CFG payload (context) + estat
  emg_cfg_payload_t cfg_payload;
  uint8_t   cfg_valid;        // s’ha configurat payload?
  uint8_t   cfg_sent;         // ja s’ha enviat correctament?
  uint32_t  last_cfg_try_ms;  // per resend

  // Connexió (per detectar flanc de connexió)
  int       last_connected;   // -1 desconegut, 0 no, 1 sí

  // Stats
  uint32_t pushed_samples;
  uint32_t dropped_samples;
  uint32_t sent_packets;
  uint32_t tx_errors;
  uint32_t enqueued_frames;
  uint32_t dequeued_frames;

  uint32_t sent_cfg;
  uint32_t cfg_errors;

} emg_stream_t;

emg_stream_err_t emg_stream_init(emg_stream_t* s,
                                 hm19_t* ble,
                                 const emg_stream_config_t* cfg,
                                 uint16_t* ring_buf_samples,
                                 uint32_t cap_samples);

/* Defineix el context (CFG) que s’enviarà.
 * Pots cridar-ho després de calibratge (baseline_raw actual) i abans de començar streaming.
 */
void emg_stream_set_cfg(emg_stream_t* s, const emg_cfg_payload_t* cfg);

/* Si has recalibrat baseline, pots actualitzar només baseline_raw i forçar resend */
void emg_stream_update_baseline_raw(emg_stream_t* s, uint16_t baseline_raw, uint8_t force_resend);

/* Push d’una mostra RAW (counts) */
void emg_stream_push_sample(emg_stream_t* s, uint16_t sample_raw, uint32_t t_ms);

/* Push d’un bloc RAW (ex: DMA block) */
void emg_stream_push_block(emg_stream_t* s,
                           const uint16_t* samples_raw,
                           uint32_t n,
                           uint32_t t0_ms);

/* Tick: intenta enviar CFG si toca i fer flush de frames en cua */
void emg_stream_tick(emg_stream_t* s);

static inline uint32_t emg_stream_queued_samples(const emg_stream_t* s)
{
  return s ? s->count : 0;
}

#ifdef __cplusplus
}
#endif

#endif // EMG_STREAM_H
