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

#ifndef OMNIA_AI_TFLM_BACKEND_H
#define OMNIA_AI_TFLM_BACKEND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "omnia_ai.h"

/**
 * @file omnia_ai_tflm_backend.h
 * @brief TensorFlow Lite Micro backend entry point for the Omnia AI runtime.
 *
 * This header exposes the factory function that returns the backend vtable
 * implementing the Omnia AI backend interface using TensorFlow Lite Micro (TFLM).
 *
 * Typical usage:
 * - Applications do not include this header directly.
 * - The platform or core wiring layer selects and registers the backend
 *   when initializing an @ref omnia_ai_ctx_t.
 *
 * No backend-specific state or configuration is exposed through this interface.
 */

/**
 * @brief Get the TensorFlow Lite Micro backend vtable.
 *
 * The returned vtable provides function pointers for backend initialization,
 * inference invocation, tensor access and timing queries.
 *
 * @return Pointer to the TFLM backend vtable.
 */
const omnia_ai_backend_vtbl_t* omnia_ai_backend_tflm(void);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_AI_TFLM_BACKEND_H */
