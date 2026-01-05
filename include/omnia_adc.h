// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (c) 2025 Jan Borràs Ros
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file omnia_adc.h
 * @brief Platform-agnostic ADC abstraction for the Omnia SDK.
 *
 * This header defines a minimal, portable abstraction for ADC access
 * built on top of the Omnia port contract.
 *
 * Key properties:
 * - The ADC hardware is owned and configured by the platform.
 * - The SDK only sees a logical ADC handle plus metadata.
 * - All access is routed through the port vtable at runtime.
 *
 * This design allows the same sensor and application code to run
 * unchanged across different MCUs and HALs.
 */

#ifndef OMNIA_ADC_H
#define OMNIA_ADC_H

#include <stdint.h>
#include <stddef.h>

#include "omnia_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Logical ADC channel descriptor.
 *
 * This structure represents an ADC channel from the SDK point of view.
 * It does not expose any vendor-specific details.
 *
 * The @ref dev field is an opaque handle understood only by the
 * platform implementation (e.g. a HAL ADC handle).
 */
typedef struct {
    /** Opaque ADC handle owned by the platform. */
    omnia_adc_t  dev;

    /** ADC resolution in bits (e.g. 12 for a 12-bit ADC). */
    uint8_t      resolution_bits;

    /** ADC reference voltage in millivolts (e.g. 3300). */
    uint32_t     vref_mv;
} omnia_adc_handle_t;

/* -------------------------------------------------------------------------- */
/* Raw access                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a single raw ADC sample.
 *
 * This function delegates the actual read operation to the active
 * Omnia port implementation.
 *
 * @param h        ADC handle.
 * @param out_raw  Output buffer for the raw ADC code.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return OMNIA_ENOTSUP if the active port does not provide ADC support.
 * @return Other error codes propagated from the platform.
 */
static inline omnia_status_t
omnia_adc_read_raw(const omnia_adc_handle_t* h, uint16_t* out_raw)
{
    if (!h || !out_raw) {
        return OMNIA_EINVAL;
    }

    omnia_port_t* port = omnia_port_get();
    if (!port || !port->v || !port->v->adc_read) {
        return OMNIA_ENOTSUP;
    }

    return port->v->adc_read(h->dev, out_raw);
}

/**
 * @brief Read multiple raw ADC samples in a single call.
 *
 * This operation is optional and may not be implemented by all ports.
 * Typical implementations use DMA-backed ADC reads.
 *
 * @param h    ADC handle.
 * @param buf  Output buffer for raw samples.
 * @param n    Number of samples to read.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return OMNIA_ENOTSUP if the active port does not implement bulk ADC reads.
 * @return Other error codes propagated from the platform.
 */
static inline omnia_status_t
omnia_adc_read_raw_n(const omnia_adc_handle_t* h, uint16_t* buf, size_t n)
{
    if (!h || !buf || n == 0) {
        return OMNIA_EINVAL;
    }

    omnia_port_t* port = omnia_port_get();
    if (!port || !port->v || !port->v->adc_read_n) {
        return OMNIA_ENOTSUP;
    }

    return port->v->adc_read_n(h->dev, buf, n);
}

/* -------------------------------------------------------------------------- */
/* Conversion helpers                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Convert a raw ADC code to volts.
 *
 * The conversion uses the resolution and reference voltage stored
 * in the ADC handle.
 *
 * @param h    ADC handle.
 * @param raw  Raw ADC code.
 *
 * @return Converted voltage in volts.
 */
static inline float
omnia_adc_raw_to_volts(const omnia_adc_handle_t* h, uint16_t raw)
{
    if (!h || h->resolution_bits == 0) {
        return 0.0f;
    }

    uint32_t max_code = (1u << h->resolution_bits) - 1u;
    float vref = (float)h->vref_mv / 1000.0f;

    return ((float)raw / (float)max_code) * vref;
}

/**
 * @brief Read a single ADC sample and convert it to volts.
 *
 * This is a convenience helper combining
 * @ref omnia_adc_read_raw and @ref omnia_adc_raw_to_volts.
 *
 * @param h          ADC handle.
 * @param out_volts  Output voltage in volts.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return Other error codes propagated from the ADC backend.
 */
static inline omnia_status_t
omnia_adc_read_volts(const omnia_adc_handle_t* h, float* out_volts)
{
    if (!h || !out_volts) {
        return OMNIA_EINVAL;
    }

    uint16_t raw = 0;
    omnia_status_t st = omnia_adc_read_raw(h, &raw);
    if (st != OMNIA_OK) {
        return st;
    }

    *out_volts = omnia_adc_raw_to_volts(h, raw);
    return OMNIA_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_ADC_H */
