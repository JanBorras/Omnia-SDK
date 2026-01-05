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
 * @file omnia_ai_tflm_backend.cc
 * @brief TensorFlow Lite Micro backend implementation for the Omnia AI runtime.
 *
 * This backend instantiates a TFLM MicroInterpreter inside a user-provided arena
 * and exposes a backend vtable compatible with the Omnia AI core.
 *
 * Key properties:
 * - No dynamic heap usage: backend state and the MicroInterpreter are placed into the arena.
 * - Minimal dependency surface: the core interacts only through the backend vtable and
 *   internal helper accessors (no context layout assumptions).
 * - Optional timing: inference latency is measured using the platform port millis() hook.
 */

 // core/ai/backends/tflm/omnia_ai_tflm_backend.cc

#include "omnia_ai_tflm_backend.h"
#include "core/ai/omnia_ai_internal.h" /* internal helpers (no ctx layout hacks) */

/* TFLM headers (C++) */
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"

#include <cstring>
#include <new>

/**
 * @brief Backend private state stored in the user arena.
 *
 * The state is allocated inside the arena and referenced from the Omnia AI context
 * through internal helper functions. The arena remains owned by the caller.
 */
struct tflm_state_t {
    /** Parsed TFLM model view (points into model->data). */
    const tflite::Model* model;

    /** TFLM error reporter instance. */
    tflite::MicroErrorReporter error_reporter;

    /** Operation resolver (Phase 1: include all ops for bring-up). */
    tflite::AllOpsResolver all_ops_resolver;

    /** Interpreter instance (placement-new inside arena). */
    tflite::MicroInterpreter* interpreter;

    /** Cached last inference time in microseconds (best-effort). */
    uint32_t last_us;
};

/**
 * @brief Map a TFLM tensor into an Omnia tensor descriptor.
 *
 * This function translates type, dimensions, data pointer and quantization info.
 * It does not allocate memory and the returned descriptor points to TFLM-owned data.
 *
 * @param t   TFLM tensor pointer.
 * @param out Output tensor descriptor.
 * @return Status code.
 */
static omnia_ai_status_t tensor_to_desc(const TfLiteTensor* t, omnia_ai_tensor_desc_t* out)
{
    if (!t || !out) return OMNIA_AI_ERR_INVALID;
    std::memset(out, 0, sizeof(*out));

    switch (t->type) {
        case kTfLiteInt8:    out->dtype = OMNIA_AI_DTYPE_INT8; break;
        case kTfLiteUInt8:   out->dtype = OMNIA_AI_DTYPE_UINT8; break;
        case kTfLiteInt16:   out->dtype = OMNIA_AI_DTYPE_INT16; break;
        case kTfLiteInt32:   out->dtype = OMNIA_AI_DTYPE_INT32; break;
        case kTfLiteFloat32: out->dtype = OMNIA_AI_DTYPE_FLOAT32; break;
        default:             out->dtype = OMNIA_AI_DTYPE_UNKNOWN; break;
    }

    if (t->dims && t->dims->size > 0) {
        uint32_t n = (uint32_t)t->dims->size;
        if (n > OMNIA_AI_MAX_DIMS) n = OMNIA_AI_MAX_DIMS;
        out->dims_count = n;
        for (uint32_t i = 0; i < n; ++i) {
            out->dims[i] = (uint32_t)t->dims->data[i];
        }
    }

    out->data  = (void*)t->data.raw;
    out->bytes = (size_t)t->bytes;

    /* Populate affine quantization parameters (first scale/zero-point only). */
    if (t->quantization.type == kTfLiteAffineQuantization && t->quantization.params) {
        auto* aq = (TfLiteAffineQuantization*)t->quantization.params;
        if (aq->scale && aq->zero_point && aq->scale->size > 0 && aq->zero_point->size > 0) {
            out->quant.valid = true;
            out->quant.scale = aq->scale->data[0];
            out->quant.zero_point = aq->zero_point->data[0];
        }
    }

    return OMNIA_AI_OK;
}

/**
 * @brief Allocate the backend state inside the provided arena.
 *
 * The state is aligned to @p align. The returned pointer refers to memory inside
 * @p arena and must not be freed.
 *
 * @param arena      Arena base pointer.
 * @param arena_size Arena size in bytes.
 * @param align      Requested alignment (defaults to 16 if zero).
 * @param out_offset Optional output: offset from arena base to the state.
 * @return Pointer to aligned state, or nullptr on failure.
 */
static tflm_state_t* arena_alloc_state(uint8_t* arena,
                                      size_t arena_size,
                                      size_t align,
                                      size_t* out_offset)
{
    if (!arena || arena_size < sizeof(tflm_state_t)) return nullptr;
    if (align == 0) align = 16;

    uintptr_t base = reinterpret_cast<uintptr_t>(arena);
    uintptr_t aligned = (base + (align - 1)) & ~(uintptr_t)(align - 1);

    size_t offset = (size_t)(aligned - base);
    if (offset + sizeof(tflm_state_t) > arena_size) return nullptr;

    if (out_offset) *out_offset = offset;
    return reinterpret_cast<tflm_state_t*>(arena + offset);
}

/* -------------------------- Backend operations --------------------------- */

/**
 * @brief Initialize the TFLM backend.
 *
 * This function:
 * - Allocates backend state and the MicroInterpreter object inside the arena.
 * - Validates model schema compatibility.
 * - Calls AllocateTensors() to prepare the interpreter.
 *
 * @param ctx   Omnia AI context.
 * @param model Model descriptor (points to TFLite flatbuffer).
 * @param cfg   Backend configuration (arena, port, alignment).
 * @return Status code.
 */
static omnia_ai_status_t tflm_init(omnia_ai_ctx_t* ctx,
                                  const omnia_ai_model_t* model,
                                  const omnia_ai_config_t* cfg)
{
    if (!ctx || !model || !cfg || !cfg->arena || cfg->arena_size == 0) {
        return OMNIA_AI_ERR_INVALID;
    }

    const omnia_port_t* port = cfg->port;

    size_t state_off = 0;
    tflm_state_t* st = arena_alloc_state(cfg->arena, cfg->arena_size, cfg->arena_align, &state_off);
    if (!st) {
        if (port && port->v && port->v->log) {
            port->v->log(3, "TFLM: arena too small for backend state");
        }
        return OMNIA_AI_ERR_NO_MEMORY;
    }
    std::memset(st, 0, sizeof(*st));

    st->model = tflite::GetModel(model->data);
    if (!st->model) {
        if (port && port->v && port->v->log) {
            port->v->log(3, "TFLM: invalid model pointer");
        }
        return OMNIA_AI_ERR_BACKEND;
    }

    if (st->model->version() != TFLITE_SCHEMA_VERSION) {
        if (port && port->v && port->v->log) {
            port->v->log(3, "TFLM: schema mismatch (model=%d runtime=%d)",
                         (int)st->model->version(), (int)TFLITE_SCHEMA_VERSION);
        }
        return OMNIA_AI_ERR_UNSUPPORTED;
    }

    /* Interpreter is created in the arena via placement new (no heap usage). */
    uint8_t* arena_rest = cfg->arena + state_off + sizeof(tflm_state_t);
    size_t   arena_rest_size = cfg->arena_size - (state_off + sizeof(tflm_state_t));

    if (arena_rest_size < sizeof(tflite::MicroInterpreter) + 16) {
        if (port && port->v && port->v->log) {
            port->v->log(3, "TFLM: arena too small for interpreter object");
        }
        return OMNIA_AI_ERR_NO_MEMORY;
    }

    void* interp_mem = (void*)arena_rest;
    arena_rest += sizeof(tflite::MicroInterpreter);
    arena_rest_size -= sizeof(tflite::MicroInterpreter);

    st->interpreter = new (interp_mem) tflite::MicroInterpreter(
        st->model,
        st->all_ops_resolver, /* Phase 1 bring-up */
        arena_rest,
        arena_rest_size,
        &st->error_reporter
    );

    TfLiteStatus ts = st->interpreter->AllocateTensors();
    if (ts != kTfLiteOk) {
        if (port && port->v && port->v->log) {
            port->v->log(3, "TFLM: AllocateTensors failed");
        }
        return OMNIA_AI_ERR_NO_MEMORY;
    }

    omnia_ai__set_backend_state(ctx, st);
    return OMNIA_AI_OK;
}

/**
 * @brief Deinitialize the TFLM backend.
 *
 * The arena remains owned by the caller. This function optionally calls
 * the MicroInterpreter destructor and clears the backend state pointer.
 *
 * @param ctx Omnia AI context.
 */
static void tflm_deinit(omnia_ai_ctx_t* ctx)
{
    if (!ctx) return;

    auto* st = (tflm_state_t*)omnia_ai__get_backend_state(ctx);
    if (!st) return;

    if (st->interpreter) {
        st->interpreter->~MicroInterpreter();
        st->interpreter = nullptr;
    }

    omnia_ai__set_backend_state(ctx, nullptr);
}

/**
 * @brief Invoke inference.
 *
 * Inference latency is measured using the platform millis() hook when available.
 * The measured value is stored in the backend state and mirrored into the core
 * cache as a fallback access path.
 *
 * @param ctx Omnia AI context.
 * @return Status code.
 */
static omnia_ai_status_t tflm_invoke(omnia_ai_ctx_t* ctx)
{
    if (!ctx) return OMNIA_AI_ERR_INVALID;

    auto* st = (tflm_state_t*)omnia_ai__get_backend_state(ctx);
    if (!st || !st->interpreter) return OMNIA_AI_ERR_NOT_INITIALIZED;

    const omnia_port_t* port = omnia_ai__get_port(ctx);

    uint64_t t0 = 0, t1 = 0;
    bool have_ms = (port && port->v && port->v->millis);
    if (have_ms) t0 = port->v->millis();

    TfLiteStatus ts = st->interpreter->Invoke();

    if (have_ms) {
        t1 = port->v->millis();
        uint64_t dt_ms = (t1 >= t0) ? (t1 - t0) : 0;
        st->last_us = (uint32_t)(dt_ms * 1000u);
    } else {
        st->last_us = 0;
    }

    /* Keep wrapper cache in sync as a fallback access path. */
    omnia_ai__set_last_inference_us_cached(ctx, st->last_us);

    return (ts == kTfLiteOk) ? OMNIA_AI_OK : OMNIA_AI_ERR_BACKEND;
}

/**
 * @brief Get an input tensor descriptor.
 *
 * The descriptor points into TFLM-managed memory and remains valid as long as
 * the interpreter is alive and tensors are allocated.
 *
 * @param ctx   Omnia AI context.
 * @param index Input tensor index.
 * @param out   Output tensor descriptor.
 * @return Status code.
 */
static omnia_ai_status_t tflm_get_input(omnia_ai_ctx_t* ctx,
                                       uint32_t index,
                                       omnia_ai_tensor_desc_t* out)
{
    if (!ctx || !out) return OMNIA_AI_ERR_INVALID;

    auto* st = (tflm_state_t*)omnia_ai__get_backend_state(ctx);
    if (!st || !st->interpreter) return OMNIA_AI_ERR_NOT_INITIALIZED;

    TfLiteTensor* t = st->interpreter->input((int)index);
    if (!t) return OMNIA_AI_ERR_INVALID;

    return tensor_to_desc(t, out);
}

/**
 * @brief Get an output tensor descriptor.
 *
 * The descriptor points into TFLM-managed memory and remains valid as long as
 * the interpreter is alive and tensors are allocated.
 *
 * @param ctx   Omnia AI context.
 * @param index Output tensor index.
 * @param out   Output tensor descriptor.
 * @return Status code.
 */
static omnia_ai_status_t tflm_get_output(omnia_ai_ctx_t* ctx,
                                        uint32_t index,
                                        omnia_ai_tensor_desc_t* out)
{
    if (!ctx || !out) return OMNIA_AI_ERR_INVALID;

    auto* st = (tflm_state_t*)omnia_ai__get_backend_state(ctx);
    if (!st || !st->interpreter) return OMNIA_AI_ERR_NOT_INITIALIZED;

    TfLiteTensor* t = st->interpreter->output((int)index);
    if (!t) return OMNIA_AI_ERR_INVALID;

    return tensor_to_desc(t, out);
}

/**
 * @brief Get the last measured inference time in microseconds.
 *
 * Note: timing is best-effort and depends on the availability and resolution
 * of the platform time source.
 *
 * @param ctx Omnia AI context.
 * @return Last inference duration in microseconds, or 0 if unavailable.
 */
static uint32_t tflm_get_last_us(omnia_ai_ctx_t* ctx)
{
    if (!ctx) return 0;
    auto* st = (tflm_state_t*)omnia_ai__get_backend_state(ctx);
    if (!st) return 0;
    return st->last_us;
}

/* ------------------------------ Vtable ----------------------------------- */

/**
 * @brief Backend vtable for TensorFlow Lite Micro.
 *
 * The core selects and uses this backend through the vtable interface.
 */
static const omnia_ai_backend_vtbl_t g_tflm_vtbl = {
    "tflm",
    tflm_init,
    tflm_deinit,
    tflm_invoke,
    tflm_get_input,
    tflm_get_output,
    tflm_get_last_us
};

/**
 * @brief Get the TFLM backend vtable.
 *
 * @return Pointer to the backend vtable.
 */
extern "C" const omnia_ai_backend_vtbl_t* omnia_ai_backend_tflm(void)
{
    return &g_tflm_vtbl;
}
