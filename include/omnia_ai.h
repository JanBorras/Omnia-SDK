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
 * @file omnia_ai.h
 * @brief Omnia AI contract (public API): backend-agnostic inference interface.
 *
 * This header defines the public AI interface of the Omnia SDK.
 * Applications and higher-level modules depend only on this contract:
 * - No vendor/runtime headers are exposed here.
 * - Backends are plugged via a vtable.
 * - Memory is caller-owned: the runtime never allocates from the heap by default.
 *
 * The optional Omnia port pointer allows a backend and the wrapper to use
 * platform services (timing/log) through the port vtable without leaking HALs.
 */

#ifndef OMNIA_AI_H
#define OMNIA_AI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "omnia_port.h" /* omnia_port_t for optional time/log hooks */

/* -------------------------------------------------------------------------- */
/* Opaque context                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Opaque AI runtime context.
 *
 * The concrete definition is private to the core implementation.
 * Backends and apps treat it as an opaque handle.
 */
typedef struct omnia_ai_ctx omnia_ai_ctx_t;

/* -------------------------------------------------------------------------- */
/* Status codes                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Status codes returned by the Omnia AI API.
 */
typedef enum {
    OMNIA_AI_OK = 0,              /**< Operation completed successfully. */
    OMNIA_AI_ERR_INVALID,         /**< Invalid argument or contract violation. */
    OMNIA_AI_ERR_NO_MEMORY,       /**< Arena too small or required memory unavailable. */
    OMNIA_AI_ERR_UNSUPPORTED,     /**< Unsupported model/runtime feature. */
    OMNIA_AI_ERR_BACKEND,         /**< Backend-specific failure (invoke/init/etc.). */
    OMNIA_AI_ERR_NOT_INITIALIZED, /**< Context not initialized (init not called or failed). */
} omnia_ai_status_t;

/* -------------------------------------------------------------------------- */
/* Tensor types                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Tensor element data types supported by the API.
 *
 * Backends may expose additional internal types, but only these are part
 * of the public contract.
 */
typedef enum {
    OMNIA_AI_DTYPE_UNKNOWN = 0,
    OMNIA_AI_DTYPE_INT8,
    OMNIA_AI_DTYPE_UINT8,
    OMNIA_AI_DTYPE_INT16,
    OMNIA_AI_DTYPE_UINT16,
    OMNIA_AI_DTYPE_INT32,
    OMNIA_AI_DTYPE_UINT32,
    OMNIA_AI_DTYPE_FLOAT32,
} omnia_ai_dtype_t;

/** @brief Maximum number of tensor dimensions supported by the descriptor. */
#define OMNIA_AI_MAX_DIMS 8u

/**
 * @brief Quantization parameters for affine quantized tensors.
 *
 * When @ref valid is false, the remaining fields must be ignored.
 */
typedef struct {
    bool    valid;       /**< Whether quantization parameters are valid. */
    float   scale;       /**< Quantization scale. */
    int32_t zero_point;  /**< Quantization zero-point. */
} omnia_ai_quant_t;

/**
 * @brief Backend-agnostic tensor descriptor.
 *
 * This descriptor is used to expose input/output tensors without leaking
 * vendor-specific tensor structures. The underlying memory is owned by
 * the backend runtime (typically inside the arena).
 */
typedef struct {
    omnia_ai_dtype_t dtype;                      /**< Element data type. */

    uint32_t dims[OMNIA_AI_MAX_DIMS];            /**< Tensor shape (up to OMNIA_AI_MAX_DIMS). */
    uint32_t dims_count;                         /**< Number of valid entries in @ref dims. */

    size_t   bytes;                              /**< Total byte size of the tensor buffer. */
    void*    data;                               /**< Pointer to tensor buffer memory. */

    omnia_ai_quant_t quant;                      /**< Optional quantization parameters. */
} omnia_ai_tensor_desc_t;

/**
 * @brief Model blob descriptor.
 *
 * The model data is treated as a read-only byte buffer. The format is
 * backend-dependent (e.g. TFLite FlatBuffer for TFLM).
 */
typedef struct {
    const uint8_t* data;  /**< Pointer to model bytes. */
    size_t         size;  /**< Size of model bytes in bytes. */
} omnia_ai_model_t;

/* -------------------------------------------------------------------------- */
/* Config                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief AI runtime configuration.
 *
 * The caller owns the arena. The runtime must not call malloc/free by default.
 * The optional port pointer provides access to timing/log hooks via the
 * Omnia port contract (e.g. millis(), log()).
 */
typedef struct {
    /** Required: tensor arena provided by the caller. */
    uint8_t* arena;

    /** Required: size of @ref arena in bytes. */
    size_t   arena_size;

    /**
     * Required: requested alignment for arena allocations.
     * Typical value is 16.
     */
    size_t   arena_align;

    /**
     * Optional: Omnia port handle for timing/logging hooks.
     * Recommended for profiling and diagnostics, but not mandatory.
     */
    const omnia_port_t* port;

    /**
     * Backend tuning flag: enable optimized kernels when available
     * (e.g. CMSIS-NN on Cortex-M).
     */
    bool enable_optimized_kernels;

    /**
     * Backend tuning flag: enable profiling instrumentation if supported.
     */
    bool enable_profiler;

    /**
     * Optional user pointer forwarded to the backend/core as needed.
     * The contract does not interpret this pointer.
     */
    void* user;
} omnia_ai_config_t;

/* -------------------------------------------------------------------------- */
/* Backend vtable                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Backend operations vtable.
 *
 * A backend (e.g. TFLM, CMSIS-NN wrapper, Mock) implements these operations.
 *
 * Contract requirements:
 * - No heap allocations by default: use only @ref omnia_ai_config_t::arena.
 * - Use @ref omnia_ai_config_t::port for time/log when provided.
 * - Keep all vendor/runtime headers private to the backend implementation.
 */
typedef struct omnia_ai_backend_vtbl {
    /** Backend name identifier (e.g. "tflm"). */
    const char* name;

    /**
     * @brief Initialize the backend runtime and bind it to the context.
     *
     * Implementations should place all runtime state into the provided arena.
     *
     * @param ctx   AI context previously bound with omnia_ai_bind().
     * @param model Model blob.
     * @param cfg   Runtime configuration (arena, optional port, tuning flags).
     * @return Status code.
     */
    omnia_ai_status_t (*init)(
        omnia_ai_ctx_t* ctx,
        const omnia_ai_model_t* model,
        const omnia_ai_config_t* cfg
    );

    /**
     * @brief Deinitialize the backend runtime.
     *
     * The arena remains owned by the caller. Backends must not free it.
     *
     * @param ctx AI context.
     */
    void (*deinit)(omnia_ai_ctx_t* ctx);

    /**
     * @brief Invoke inference.
     *
     * @param ctx AI context.
     * @return Status code.
     */
    omnia_ai_status_t (*invoke)(omnia_ai_ctx_t* ctx);

    /**
     * @brief Get a descriptor for an input tensor.
     *
     * The returned descriptor points to backend-owned memory (typically in arena).
     *
     * @param ctx   AI context.
     * @param index Input tensor index.
     * @param out   Output descriptor.
     * @return Status code.
     */
    omnia_ai_status_t (*get_input)(
        omnia_ai_ctx_t* ctx,
        uint32_t index,
        omnia_ai_tensor_desc_t* out
    );

    /**
     * @brief Get a descriptor for an output tensor.
     *
     * The returned descriptor points to backend-owned memory (typically in arena).
     *
     * @param ctx   AI context.
     * @param index Output tensor index.
     * @param out   Output descriptor.
     * @return Status code.
     */
    omnia_ai_status_t (*get_output)(
        omnia_ai_ctx_t* ctx,
        uint32_t index,
        omnia_ai_tensor_desc_t* out
    );

    /**
     * @brief Optional: get the last inference time in microseconds.
     *
     * If the backend provides a fine-grained timer, it should return a non-zero
     * value. If only millis() is available, returning (ms * 1000) is acceptable
     * but coarse.
     *
     * If not supported, this pointer may be NULL.
     *
     * @param ctx AI context.
     * @return Last inference time in microseconds, or 0 if unavailable.
     */
    uint32_t (*get_last_inference_us)(omnia_ai_ctx_t* ctx);

} omnia_ai_backend_vtbl_t;

/* -------------------------------------------------------------------------- */
/* Binding                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Bind a backend vtable to a context shell.
 *
 * This must be called before omnia_ai_init(). Binding selects which backend
 * implementation will service subsequent init/invoke/tensor calls.
 *
 * @param ctx     Context to bind.
 * @param backend Backend vtable implementation.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_bind(
    omnia_ai_ctx_t* ctx,
    const omnia_ai_backend_vtbl_t* backend
);

/* -------------------------------------------------------------------------- */
/* Public API (backend-agnostic)                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize an AI context with a model and configuration.
 *
 * The backend will use the caller-provided arena for all runtime allocations.
 *
 * @param ctx   AI context previously bound with omnia_ai_bind().
 * @param model Model blob.
 * @param cfg   Runtime configuration.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_init(
    omnia_ai_ctx_t* ctx,
    const omnia_ai_model_t* model,
    const omnia_ai_config_t* cfg
);

/**
 * @brief Deinitialize an AI context.
 *
 * After this call, the context becomes inert until re-initialized.
 *
 * @param ctx AI context.
 */
void omnia_ai_deinit(omnia_ai_ctx_t* ctx);

/**
 * @brief Run inference.
 *
 * @param ctx AI context.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_invoke(omnia_ai_ctx_t* ctx);

/**
 * @brief Get a descriptor for an input tensor.
 *
 * @param ctx   AI context.
 * @param index Input tensor index.
 * @param out   Output descriptor.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_get_input(
    omnia_ai_ctx_t* ctx,
    uint32_t index,
    omnia_ai_tensor_desc_t* out
);

/**
 * @brief Get a descriptor for an output tensor.
 *
 * @param ctx   AI context.
 * @param index Output tensor index.
 * @param out   Output descriptor.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_get_output(
    omnia_ai_ctx_t* ctx,
    uint32_t index,
    omnia_ai_tensor_desc_t* out
);

/**
 * @brief Get the last inference time in microseconds.
 *
 * If the backend provides a measurement via the vtable, it is preferred.
 * Otherwise, the core may return a coarse wrapper-level estimate.
 *
 * @param ctx AI context.
 * @return Inference time in microseconds, or 0 if unavailable.
 */
uint32_t omnia_ai_get_last_inference_us(omnia_ai_ctx_t* ctx);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_AI_H */
