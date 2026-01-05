#ifndef APP_EMG_H
#define APP_EMG_H

#include "sen0240.h"
#include "omnia_adc.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_EMG_STATE_RUN = 0,
  APP_EMG_STATE_CALIBRATING
} app_emg_state_t;

typedef struct {
  sen0240_t        sen;

  /* “Latest sample” (per UI / debug) */
  uint16_t         raw;
  float            volts;
  float            centered;
  float            norm;
  uint8_t          saturated;

  app_emg_state_t  state;

  /* baseline vàlida? */
  uint8_t          has_baseline;

  /* Calibratge per mostres reals */
  uint32_t         calib_samples;
  uint32_t         calib_target;
  float            calib_accum;     // suma de volts

  /* Guany per normalitzar */
  float            gain_volts;

  /* (Opcional) Resum del darrer bloc per UI */
  float            last_block_rms;
  float            last_block_peak;
} app_emg_t;

/* Config per defecte del calibratge (a 2 kHz, 12000 mostres = 6s) */
#ifndef APP_EMG_CALIB_SAMPLES
#define APP_EMG_CALIB_SAMPLES 15000U
#endif

void app_emg_init(app_emg_t*  a,
                  const omnia_adc_handle_t* adc,
                  float v_offset_initial,
                  float gain_volts);

void app_emg_start_calibration(app_emg_t* a);

/* Mode antic (1 mostra). Es manté per compatibilitat, però NO és el camí bo a 2kHz. */
void app_emg_step(app_emg_t* a);

/* Nou: processa un bloc de mostres RAW ja capturat per DMA.
 * - Actualitza baseline si estàs calibrant
 * - Si tens baseline: calcula centered/norm per cada mostra i et deixa “latest”
 * - Calcula last_block_rms/peak (opcional però útil)
 */
void app_emg_process_block_raw(app_emg_t* a,
                               const uint16_t* raw,
                               size_t n);

/* Helpers */
static inline int app_emg_is_calibrating(const app_emg_t* a) {
  return a && (a->state == APP_EMG_STATE_CALIBRATING);
}

static inline int app_emg_has_baseline(const app_emg_t* a) {
  return a && (a->has_baseline != 0);
}

#ifdef __cplusplus
}
#endif
#endif
