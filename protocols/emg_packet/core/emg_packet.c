#include "emg_packet.h"
#include <string.h>

/* Helpers LE */
static inline void write_u16_le(uint8_t* buf, uint16_t v)
{
  buf[0] = (uint8_t)(v & 0xFFu);
  buf[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline void write_i16_le(uint8_t* buf, int16_t v)
{
  write_u16_le(buf, (uint16_t)v);
}

static inline void write_u32_le(uint8_t* buf, uint32_t v)
{
  buf[0] = (uint8_t)(v & 0xFFu);
  buf[1] = (uint8_t)((v >> 8) & 0xFFu);
  buf[2] = (uint8_t)((v >> 16) & 0xFFu);
  buf[3] = (uint8_t)((v >> 24) & 0xFFu);
}

uint16_t emg_crc16_ccitt(const uint8_t* data, size_t len)
{
  uint16_t crc = 0xFFFFu;
  const uint16_t poly = 0x1021u;

  for (size_t i = 0; i < len; ++i) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ poly) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* CRC sempre sobre [TYPE..darrera dada], excloent CRC */
static inline uint16_t compute_crc_over_len_region(const uint8_t* out_buf, uint16_t len_field)
{
  // out_buf: [SYNC0][SYNC1][LENL][LENH][TYPE..CRC]
  // region CRC = TYPE..(últim byte abans CRC)
  const uint8_t* start = out_buf + 4u;            // TYPE comença aquí
  const size_t   n     = (size_t)len_field - 2u;  // treu CRC(2)
  return emg_crc16_ccitt(start, n);
}

size_t emg_build_raw_u16_packet(uint8_t         seq,
                                uint32_t        time_ms,
                                const uint16_t* samples,
                                uint8_t         nsamples,
                                uint8_t*        out_buf,
                                size_t          out_buf_size)
{
  if (!samples || !out_buf) return 0;
  if (nsamples == 0 || nsamples > EMG_PKT_MAX_SAMPLES) return 0;

  const uint16_t len_field = emg_pkt_len_raw(nsamples);
  const size_t total_size = emg_pkt_total_from_len(len_field);
  if (out_buf_size < total_size) return 0;

  uint8_t* p = out_buf;

  /* SYNC */
  *p++ = EMG_PKT_SYNC0;
  *p++ = EMG_PKT_SYNC1;

  /* LEN */
  write_u16_le(p, len_field);
  p += 2;

  /* TYPE */
  *p++ = (uint8_t)EMG_PKT_TYPE_RAW_U16;

  /* SEQ */
  *p++ = seq;

  /* TIME */
  write_u32_le(p, time_ms);
  p += 4;

  /* NS */
  *p++ = nsamples;

  /* DATA: uint16 LE */
  for (uint8_t i = 0; i < nsamples; ++i) {
    write_u16_le(p, samples[i]);
    p += 2;
  }

  /* CRC */
  {
    uint16_t crc = compute_crc_over_len_region(out_buf, len_field);
    write_u16_le(p, crc);
    p += 2;
  }

  return ((size_t)(p - out_buf) == total_size) ? total_size : 0;
}

size_t emg_build_cfg_packet(uint8_t                  seq,
                            uint32_t                 time_ms,
                            const emg_cfg_payload_t* cfg,
                            uint8_t*                 out_buf,
                            size_t                   out_buf_size)
{
  if (!cfg || !out_buf) return 0;

  const uint16_t len_field = emg_pkt_len_cfg();
  const size_t total_size = emg_pkt_total_from_len(len_field);
  if (out_buf_size < total_size) return 0;

  uint8_t* p = out_buf;

  /* SYNC */
  *p++ = EMG_PKT_SYNC0;
  *p++ = EMG_PKT_SYNC1;

  /* LEN */
  write_u16_le(p, len_field);
  p += 2;

  /* TYPE */
  *p++ = (uint8_t)EMG_PKT_TYPE_CFG;

  /* SEQ */
  *p++ = seq;

  /* TIME */
  write_u32_le(p, time_ms);
  p += 4;

  /* CFG payload (fix 10 bytes) */
  *p++ = cfg->resolution_bits;
  write_u16_le(p, cfg->vref_mv);        p += 2;
  *p++ = cfg->adc_channel;
  write_u16_le(p, cfg->sample_rate_hz); p += 2;
  write_u16_le(p, cfg->baseline_raw);   p += 2;
  write_i16_le(p, cfg->gain_q15);       p += 2;

  /* CRC */
  {
    uint16_t crc = compute_crc_over_len_region(out_buf, len_field);
    write_u16_le(p, crc);
    p += 2;
  }

  return ((size_t)(p - out_buf) == total_size) ? total_size : 0;
}
