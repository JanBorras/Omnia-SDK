#ifndef SEN0240_H
#define SEN0240_H

#include <stdint.h>
#include "omnia_types.h"
#include "omnia_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle lògic per al mòdul EMG SEN0240.
 *
 * No sap res d'ADC1 ni d'STM32; només veu un omnia_adc_handle_t
 * ja configurat (canal, vref, resolució...).
 */
typedef struct {
  omnia_adc_handle_t adc;  // ADC associat al SEN0240
  float              v_offset;     // volts de baseline (~1.5 V)
  float              gain_volts;   // opcional: escala per normalitzar (p.ex. vref/2)
} sen0240_t;

/**
 * @brief Inicialitza l'estructura lògica del sensor.
 *
 * No configura el hardware; assumeix que l'ADC ja està configurat
 * (MX_ADCx_Init, canal correcte, sampling time, etc.).
 *
 * @param s       Punter a l'estructura del sensor.
 * @param adc     Handle ADC preconfigurat (còpia per valor).
 * @param v_offset Baseline en volts (normalment 1.5f).
 * @param gain_volts Factor per normalitzar (ex: vref/2, o amplitud esperada).
 */
omnia_status_t sen0240_init(sen0240_t* s,
                            const omnia_adc_handle_t* adc,
                            float v_offset,
                            float gain_volts);

/**
 * @brief Llegeix una mostra RAW (conteig ADC cru).
 */
omnia_status_t sen0240_read_raw(sen0240_t* s, uint16_t* out_raw);

/**
 * @brief Llegeix una mostra en volts absoluts (0..vref).
 */
omnia_status_t sen0240_read_volts(sen0240_t* s, float* out_volts);

/**
 * @brief Llegeix una mostra centrada respecte la baseline (volts).
 *
 * Exemple: si v_offset=1.5V i la mesura és 1.55V → out = +0.05V
 */
omnia_status_t sen0240_read_centered(sen0240_t* s, float* out_centered);

/**
 * @brief Llegeix una mostra centrada i normalitzada.
 *
 * centered_norm = (v - v_offset) / gain_volts
 *
 * Si gain_volts = vref/2 → idees per tenir rang aproximat [-1, +1].
 */
omnia_status_t sen0240_read_centered_norm(sen0240_t* s, float* out_norm);

/**
 * @brief Calibra la baseline del SEN0240 fent múltiples lectures.
 *
 * Aquesta funció llegeix n_samples mostres consecutives de l'ADC
 * amb un delay_ms entre cada lectura, i calcula la mitjana per
 * establir una nova baseline (v_offset).
 *
 * Útil per ajustar la baseline a les condicions reals d'ús
 * (sense activitat muscular) després de la inicialització.
 *
 * @param s          Punter al context del sensor.
 * @param n_samples  Nombre de mostres a llegir per a la mitjana.
 * @param delay_ms    Retard en mil·lisegons entre mostres.
 */
omnia_status_t sen0240_calibrate_baseline(sen0240_t* s,
                                          uint32_t n_samples,
                                          uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif // SEN0240_H
