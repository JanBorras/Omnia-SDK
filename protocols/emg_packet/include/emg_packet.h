#ifndef EMG_PACKET_H
#define EMG_PACKET_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sync */
#define EMG_PKT_SYNC0  0xAAu
#define EMG_PKT_SYNC1  0x55u

/* Tipus de paquet */
typedef enum {
  EMG_PKT_TYPE_RAW_U16 = 0x00u,  // samples = uint16 (raw ADC counts)
  EMG_PKT_TYPE_CFG     = 0x01u,  // context/metadades
} emg_pkt_type_t;

/* Limits */
#define EMG_PKT_MAX_SAMPLES  32u

/* CRC-16-CCITT: poly=0x1021, init=0xFFFF, no reflectit */
uint16_t emg_crc16_ccitt(const uint8_t* data, size_t len);

/* -----------------------------
 * Estructura CFG (fixa)
 * -----------------------------
 * Dissenyada per ser simple de parsejar al PC.
 *
 * Tots els enters en little-endian dins del paquet.
 */
typedef struct {
  uint8_t  resolution_bits;   // ex: 12
  uint16_t vref_mv;           // ex: 3300
  uint8_t  adc_channel;       // ex: 3 (informatiu)
  uint16_t sample_rate_hz;    // ex: 2000
  uint16_t baseline_raw;      // ex: baseline en counts (mitjana repòs)
  int16_t  gain_q15;          // opcional: escala Q15 (0 si no s’usa)
} emg_cfg_payload_t;

/* Mides per còmput */
#define EMG_PKT_COMMON_HDR_LEN   (2u /*SYNC*/ + 2u /*LEN*/ + 1u /*TYPE*/ + 1u /*SEQ*/ + 4u /*TIME*/)
#define EMG_PKT_CRC_LEN          (2u)
#define EMG_PKT_RAW_HDR_LEN      (1u /*NS*/)
#define EMG_PKT_CFG_PAYLOAD_LEN  (1u + 2u + 1u + 2u + 2u + 2u)  /* = 10 bytes */

/* LEN (camp intern) = TYPE + SEQ + TIME + PAYLOAD + CRC */
static inline uint16_t emg_pkt_len_raw(uint8_t nsamples)
{
  // TYPE(1)+SEQ(1)+TIME(4)+NS(1)+DATA(ns*2)+CRC(2)
  return (uint16_t)(1u + 1u + 4u + 1u + (uint16_t)nsamples * 2u + 2u);
}

static inline uint16_t emg_pkt_len_cfg(void)
{
  // TYPE(1)+SEQ(1)+TIME(4)+CFG(10)+CRC(2)
  return (uint16_t)(1u + 1u + 4u + EMG_PKT_CFG_PAYLOAD_LEN + 2u);
}

/* Total bytes del paquet = SYNC(2)+LEN(2)+LEN */
static inline size_t emg_pkt_total_from_len(uint16_t len_field)
{
  return (size_t)(4u + (size_t)len_field);
}

static inline size_t emg_pkt_total_raw(uint8_t nsamples)
{
  return emg_pkt_total_from_len(emg_pkt_len_raw(nsamples));
}

static inline size_t emg_pkt_total_cfg(void)
{
  return emg_pkt_total_from_len(emg_pkt_len_cfg());
}

/* Builders:
 * - Retornen bytes totals escrits a out_buf, o 0 si error.
 */
size_t emg_build_raw_u16_packet(uint8_t         seq,
                                uint32_t        time_ms,
                                const uint16_t* samples,
                                uint8_t         nsamples,
                                uint8_t*        out_buf,
                                size_t          out_buf_size);

size_t emg_build_cfg_packet(uint8_t                 seq,
                            uint32_t                time_ms,
                            const emg_cfg_payload_t* cfg,
                            uint8_t*                out_buf,
                            size_t                  out_buf_size);

#ifdef __cplusplus
}
#endif

#endif // EMG_PACKET_H
