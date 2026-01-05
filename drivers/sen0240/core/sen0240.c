#include "sen0240.h"
#include "omnia_port.h" // per omnia_port_has_adc()

/**
 * @brief Inicialitza el context lògic del sensor EMG SEN0240.
 *
 * Aquesta funció NO configura el maquinari de l'ADC. Assumeix que:
 *   - L'ADC ja està inicialitzat i configurat (canal, sampling time, vref, etc.).
 *   - El handle omnia_adc_handle_t reflecteix correctament aquesta configuració.
 *
 * El que fa és:
 *   - Validar els punters d'entrada.
 *   - Comprovar que el port actual disposa de suport d'ADC.
 *   - Guardar una còpia del handle ADC i dels paràmetres de calibratge
 *     (offset i gain) dins de l'estructura sen0240_t.
 *
 * @param s          Punter a l'estructura del sensor (context intern).
 * @param adc        Punter a un handle ADC ja configurat (es copia per valor).
 * @param v_offset   Baseline en volts del SEN0240 (normalment ~1.5 V).
 * @param gain_volts Factor d'escala per normalitzar el senyal centrat
 *                   (per exemple, vref/2 o amplitud màxima esperada).
 *
 * @return OMNIA_OK           si tot és correcte.
 *         OMNIA_EINVAL       si s o adc són NULL.
 *         OMNIA_EUNSUPPORTED si el port actual no anuncia suport d'ADC.
 */
omnia_status_t sen0240_init(sen0240_t* s,
                            const omnia_adc_handle_t* adc,
                            float v_offset,
                            float gain_volts)
{
  // Validació bàsica de punters d’entrada
  if (!s || !adc) {
    return OMNIA_EINVAL;
  }

  // Comprovem que el port actual declara suport d'ADC.
  // Això garanteix que hi ha una implementació de adc_read al vtable.
  if (!omnia_port_has_adc()) {
    return OMNIA_ENOTSUP;
  }

  // Guardem una còpia del handle ADC (per valor, per independitzar-nos
  // de la vida del paràmetre extern).
  s->adc = *adc;

  // Guardem la baseline (offset) en volts. Aquest valor es farà servir
  // per centrar el senyal al voltant de 0 V.
  s->v_offset = v_offset;

  // Guardem el factor d'escala. Si el caller passa un valor no vàlid,
  // fem servir 1.0f per evitar divisions per zero.
  s->gain_volts = (gain_volts > 0.0f) ? gain_volts : 1.0f;

  return OMNIA_OK;
}

/**
 * @brief Llegeix una mostra crua (RAW) del SEN0240 via ADC.
 *
 * Aquesta funció no aplica cap calibratge ni conversió a volts. Simplement:
 *   - Demana al backend d'ADC una mostra.
 *   - Retorna el valor de conteig (p.ex. 0..4095 per 12 bits).
 *
 * @param s        Punter al context del sensor.
 * @param out_raw  Punter on es desarà el valor cru de l'ADC.
 *
 * @return OMNIA_OK     si la lectura és correcta.
 *         OMNIA_EINVAL si s o out_raw són NULL.
 *         Altres codis d'error provinents de omnia_adc_read_raw().
 */
omnia_status_t sen0240_read_raw(sen0240_t* s, uint16_t* out_raw)
{
  if (!s || !out_raw) {
    return OMNIA_EINVAL;
  }

  // Deleguem la lectura crua a l'helper genèric d'ADC del SDK.
  return omnia_adc_read_raw(&s->adc, out_raw);
}

/**
 * @brief Llegeix una mostra en volts absoluts (respecte vref).
 *
 * Aquesta funció:
 *   - Llegeix una mostra RAW de l'ADC.
 *   - Converteix el valor de conteig a volts utilitzant la resolució i vref
 *     emmagatzemades a omnia_adc_handle_t.
 *
 * No treu l'offset del SEN0240: és la tensió "tal qual" mesurada al pin.
 *
 * @param s          Punter al context del sensor.
 * @param out_volts  Punter on es desarà la tensió en volts.
 *
 * @return OMNIA_OK     si la lectura i conversió són correctes.
 *         OMNIA_EINVAL si s o out_volts són NULL.
 *         Altres codis d'error provinents de omnia_adc_read_volts().
 */
omnia_status_t sen0240_read_volts(sen0240_t* s, float* out_volts)
{
  if (!s || !out_volts) {
    return OMNIA_EINVAL;
  }

  // Fem servir directament l'helper que llegeix RAW i converteix a volts.
  return omnia_adc_read_volts(&s->adc, out_volts);
}

/**
 * @brief Llegeix una mostra centrada respecte la baseline del SEN0240.
 *
 * Flux:
 *   1. Llegeix una mostra en volts absoluts.
 *   2. Resta la baseline (v_offset) configurada a sen0240_init().
 *
 * Exemple:
 *   - v_offset = 1.5 V
 *   - mesura = 1.55 V
 *   - out_centered = +0.05 V
 *
 * Això prepara el senyal per tractar l'EMG com una amplitud al voltant de 0 V.
 *
 * @param s             Punter al context del sensor.
 * @param out_centered  Punter on es desarà el valor centrat en volts.
 *
 * @return OMNIA_OK     si tot és correcte.
 *         OMNIA_EINVAL si s o out_centered són NULL.
 *         Altres codis d'error si la lectura ADC falla.
 */
omnia_status_t sen0240_read_centered(sen0240_t* s, float* out_centered)
{
  if (!s || !out_centered) {
    return OMNIA_EINVAL;
  }

  float v = 0.0f;

  // Primer, obtenim la tensió absoluta via l'ADC.
  omnia_status_t st = omnia_adc_read_volts(&s->adc, &v);
  if (st != OMNIA_OK) {
    return st;
  }

  // Després, centrem el senyal restant la baseline configurada.
  *out_centered = v - s->v_offset;
  return OMNIA_OK;
}

/**
 * @brief Llegeix una mostra centrada i normalitzada.
 *
 * Aquesta funció combina:
 *   - La centrada del senyal (treure v_offset).
 *   - Una normalització per un factor gain_volts.
 *
 * centered_norm = (v - v_offset) / gain_volts
 *
 * Útil per preparar dades per a models de ML:
 *   - Si gain_volts = vref/2, el rang esperat queda aproximadament dins [-1, +1].
 *
 * @param s        Punter al context del sensor.
 * @param out_norm Punter on es desarà el valor centrat i normalitzat.
 *
 * @return OMNIA_OK     si tot és correcte.
 *         OMNIA_EINVAL si s o out_norm són NULL.
 *         Altres codis d'error si la lectura ADC o la centrada fallen.
 */
omnia_status_t sen0240_read_centered_norm(sen0240_t* s, float* out_norm)
{
  if (!s || !out_norm) {
    return OMNIA_EINVAL;
  }

  float centered = 0.0f;

  // Reutilitzem la funció que ja fa la lectura en volts i la resta d'offset.
  omnia_status_t st = sen0240_read_centered(s, &centered);
  if (st != OMNIA_OK) {
    return st;
  }

  // Normalització pel gain configurat a l'init. Evitem divisió per zero
  // gràcies a la protecció a sen0240_init (gain_volts >= 1.0f).
  *out_norm = centered / s->gain_volts;
  return OMNIA_OK;
}

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
 * @param n_samples  Nombre de mostres a llegir per a la calibració.
 * @param delay_ms   Delay en mil·lisegons entre cada lectura (0 per cap delay).
 *
 * @return OMNIA_OK           si la calibració és correcta.
 *         OMNIA_EINVAL       si s és NULL o n_samples és 0.
 *         OMNIA_ENOTSUP      si el port actual no implementa delay_us().
 *         Altres codis d'error si alguna lectura ADC falla.
 */
omnia_status_t sen0240_calibrate_baseline(sen0240_t* s,
                                          uint32_t n_samples,
                                          uint32_t delay_ms)
{
  if (!s || n_samples == 0U) {
    return OMNIA_EINVAL;
  }

  // Obtenim el port actual per fer servir delay_us()
  omnia_port_t* port = omnia_port_get();
  if (!port || !port->v || !port->v->delay_us) {
    // El port actual no implementa timing de microsegons
    return OMNIA_ENOTSUP;
  }

  float acc = 0.0f;

  for (uint32_t i = 0; i < n_samples; ++i) {
    float v = 0.0f;

    // Fem servir la pròpia API del driver per llegir volts absoluts
    omnia_status_t st = sen0240_read_volts(s, &v);
    if (st != OMNIA_OK) {
      return st;  // Propaguen l'error d'ADC o del port
    }

    acc += v;

    // Delay entre mostres, si s'ha demanat
    if (delay_ms > 0U) {
      port->v->delay_us(delay_ms * 1000U);
    }
  }

  // Mitjana de la tensió observada en repòs -> nova línia base
  s->v_offset = acc / (float)n_samples;

  return OMNIA_OK;
}