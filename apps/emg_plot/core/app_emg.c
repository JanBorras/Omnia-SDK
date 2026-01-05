#include "app_emg.h"
#include "omnia_port.h"
#include <math.h>

static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void app_emg_init(app_emg_t* a,
                  const omnia_adc_handle_t* adc,
                  float v_offset_initial,
                  float gain_volts)
{
  if (!a || !adc) return;

  (void)sen0240_init(&a->sen, adc, v_offset_initial, gain_volts);

  a->raw       = 0;
  a->volts     = 0.0f;
  a->centered  = 0.0f;
  a->norm      = 0.0f;
  a->saturated = 0;

  a->state        = APP_EMG_STATE_RUN;
  a->has_baseline = 0;

  a->calib_samples = 0;
  a->calib_target  = APP_EMG_CALIB_SAMPLES;
  a->calib_accum   = 0.0f;

  a->gain_volts    = (gain_volts > 0.0f) ? gain_volts : 1.0f;

  a->last_block_rms  = 0.0f;
  a->last_block_peak = 0.0f;
}

void app_emg_start_calibration(app_emg_t* a)
{
  if (!a) return;

  a->state         = APP_EMG_STATE_CALIBRATING;
  a->calib_samples = 0;
  a->calib_accum   = 0.0f;
  a->has_baseline  = 0;

  a->centered  = 0.0f;
  a->norm      = 0.0f;
  a->saturated = 0;

  a->last_block_rms  = 0.0f;
  a->last_block_peak = 0.0f;
}

/* Compatibilitat: una mostra */
void app_emg_step(app_emg_t* a)
{
  if (!a) return;

  uint16_t raw = 0;
  if (sen0240_read_raw(&a->sen, &raw) != OMNIA_OK) {
    return;
  }

  app_emg_process_block_raw(a, &raw, 1);
}

/* Nou motor: bloc de RAW (capturat per DMA) */
void app_emg_process_block_raw(app_emg_t* a,
                               const uint16_t* raw,
                               size_t n)
{
  if (!a || !raw || n == 0) return;

  /* Conversions */
  float peak = 0.0f;
  float acc2 = 0.0f;

  for (size_t i = 0; i < n; ++i) {
    const uint16_t r = raw[i];
    const float v   = omnia_adc_raw_to_volts(&a->sen.adc, r);

    /* publish “latest” raw/volts sempre */
    a->raw   = r;
    a->volts = v;

    /* CALIBRATING: baseline = mitjana de volts */
    if (a->state == APP_EMG_STATE_CALIBRATING) {
      a->calib_accum   += v;
      a->calib_samples += 1U;

      if (a->calib_samples >= a->calib_target) {
        const float new_offset = a->calib_accum / (float)a->calib_samples;
        a->sen.v_offset  = new_offset;
        a->state         = APP_EMG_STATE_RUN;
        a->has_baseline  = 1;
      }

      /* durant calibratge: no exposem senyal “real” */
      a->centered  = 0.0f;
      a->norm      = 0.0f;
      a->saturated = 0;
      continue;
    }

    /* si no tens baseline, no generis senyal */
    if (!a->has_baseline) {
      a->centered  = 0.0f;
      a->norm      = 0.0f;
      a->saturated = 0;
      continue;
    }

    /* RUN: centered/norm */
    const float centered = v - a->sen.v_offset;
    float norm = centered / a->gain_volts;
    norm = clampf(norm, -1.0f, 1.0f);

    a->centered  = centered;
    a->norm      = norm;
    a->saturated = (fabsf(norm) > 0.98f) ? 1 : 0;

    /* Stats del bloc (opcionales però útils per UI) */
    const float an = fabsf(norm);
    if (an > peak) peak = an;
    acc2 += norm * norm;
  }

  /* Resum del bloc si estàvem en RUN i tenim baseline.
   * (Si estàvem calibrant o sense baseline, queda 0.)
   */
  if (a->state == APP_EMG_STATE_RUN && a->has_baseline) {
    a->last_block_peak = peak;
    a->last_block_rms  = (n > 0) ? sqrtf(acc2 / (float)n) : 0.0f;
  } else {
    a->last_block_peak = 0.0f;
    a->last_block_rms  = 0.0f;
  }
}
