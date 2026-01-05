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
 * @file omnia_ai.c
 * @brief Omnia AI runtime: backend binding, lifecycle and tensor access.
 *
 * This module implements the public Omnia AI API defined in @ref omnia_ai.h and
 * provides internal helper accessors used by backends (see @ref omnia_ai_internal.h).
 *
 * Architectural intent:
 * - The public context type is opaque to users.
 * - A backend is "plugged in" via a vtable and owns its private state.
 * - All predictable memory is provided by the user via an arena (no heap required).
 * - Timing and logging are accessed through the Omnia port contract when available.
 */

 // core/ai/omnia_ai.c

#include "omnia_ai.h"
#include "omnia_ai_internal.h"

#include <string.h>

/**
 * @brief Private context definition (opaque to users).
 *
 * The context is a sealed container for:
 * - Backend vtable selection.
 * - Optional platform port pointer for timing/log.
 * - Initialization guard.
 * - Wrapper-level coarse profiling fallback.
 * - Backend-private opaque state pointer.
 */
struct omnia_ai_ctx {
    /** Selected backend operations table. */
    const omnia_ai_backend_vtbl_t* vtbl;

    /** Optional platform port (timing/log hooks). */
    const omnia_port_t* port;

    /** Initialization guard (backend successfully initialized). */
    bool initialized;

    /** Coarse inference timing fallback (microseconds derived from millis()). */
    uint32_t last_inference_us_cached;

    /** Backend-private state pointer (opaque). */
    void* backend_state;
};

/* -------------------------------------------------------------------------- */
/* Internal helpers implementation                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get backend-private state from the context.
 *
 * @param ctx AI context.
 * @return Backend state pointer, or NULL if unavailable.
 */
void* omnia_ai__get_backend_state(omnia_ai_ctx_t* ctx)
{
    if (!ctx) return NULL;
    return ctx->backend_state;
}

/**
 * @brief Set backend-private state in the context.
 *
 * @param ctx   AI context.
 * @param state Backend state pointer (opaque).
 */
void omnia_ai__set_backend_state(omnia_ai_ctx_t* ctx, void* state)
{
    if (!ctx) return;
    ctx->backend_state = state;
}

/**
 * @brief Get the platform port associated with the context.
 *
 * @param ctx AI context.
 * @return Port pointer, or NULL if not configured.
 */
const omnia_port_t* omnia_ai__get_port(const omnia_ai_ctx_t* ctx)
{
    if (!ctx) return NULL;
    return ctx->port;
}

/**
 * @brief Get the cached wrapper-level inference time estimate.
 *
 * @param ctx AI context.
 * @return Cached inference time in microseconds.
 */
uint32_t omnia_ai__get_last_inference_us_cached(const omnia_ai_ctx_t* ctx)
{
    if (!ctx) return 0;
    return ctx->last_inference_us_cached;
}

/**
 * @brief Set the cached wrapper-level inference time estimate.
 *
 * @param ctx AI context.
 * @param us  Inference time estimate in microseconds.
 */
void omnia_ai__set_last_inference_us_cached(omnia_ai_ctx_t* ctx, uint32_t us)
{
    if (!ctx) return;
    ctx->last_inference_us_cached = us;
}

/* -------------------------------------------------------------------------- */
/* Binding                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Bind a backend vtable to a context.
 *
 * This function validates the backend's mandatory operations and resets the
 * context to a known state. A successful bind is required before @ref omnia_ai_init.
 *
 * @param ctx     AI context to bind.
 * @param backend Backend vtable.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_bind(omnia_ai_ctx_t* ctx, const omnia_ai_backend_vtbl_t* backend)
{
    if (!ctx || !backend || !backend->init || !backend->invoke ||
        !backend->get_input || !backend->get_output) {
        return OMNIA_AI_ERR_INVALID;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->vtbl = backend;

    return OMNIA_AI_OK;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the AI context and backend.
 *
 * The configuration requires a valid arena to enforce predictable memory usage.
 * The port pointer is optional and is used for timing/log if provided.
 *
 * @param ctx   Bound AI context.
 * @param model Model descriptor (flatbuffer).
 * @param cfg   Runtime configuration (arena, alignment, optional port).
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_init(omnia_ai_ctx_t* ctx,
                               const omnia_ai_model_t* model,
                               const omnia_ai_config_t* cfg)
{
    if (!ctx || !ctx->vtbl || !model || !model->data || model->size == 0 || !cfg) {
        return OMNIA_AI_ERR_INVALID;
    }

    /* Predictable memory rule: arena must exist and be non-empty. */
    if (!cfg->arena || cfg->arena_size == 0) {
        return OMNIA_AI_ERR_NO_MEMORY;
    }

    ctx->port = cfg->port; /* optional */
    ctx->initialized = false;
    ctx->last_inference_us_cached = 0;
    ctx->backend_state = NULL;

    omnia_ai_status_t st = ctx->vtbl->init(ctx, model, cfg);
    if (st != OMNIA_AI_OK) {
        return st;
    }

    ctx->initialized = true;
    return OMNIA_AI_OK;
}

/**
 * @brief Deinitialize the AI context and backend.
 *
 * If the backend provides a deinit operation, it will be called only when
 * the context is initialized. After deinit, the context is reset to a safe
 * inert state (no dangling pointers).
 *
 * @param ctx AI context.
 */
void omnia_ai_deinit(omnia_ai_ctx_t* ctx)
{
    if (!ctx || !ctx->vtbl) return;

    if (ctx->initialized && ctx->vtbl->deinit) {
        ctx->vtbl->deinit(ctx);
    }

    ctx->initialized = false;
    ctx->last_inference_us_cached = 0;
    ctx->backend_state = NULL;
    ctx->port = NULL;
}

/**
 * @brief Run inference using the bound backend.
 *
 * This function optionally measures inference time using the platform millis()
 * hook and stores a coarse microsecond estimate as a fallback metric.
 *
 * @param ctx AI context.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_invoke(omnia_ai_ctx_t* ctx)
{
    if (!ctx || !ctx->vtbl) return OMNIA_AI_ERR_INVALID;
    if (!ctx->initialized) return OMNIA_AI_ERR_NOT_INITIALIZED;

    /* Wrapper-level coarse timing fallback (millis -> us). */
    const omnia_port_t* port = ctx->port;
    uint64_t t0_ms = 0, t1_ms = 0;
    bool have_ms = (port && port->v && port->v->millis);

    if (have_ms) t0_ms = port->v->millis();

    omnia_ai_status_t st = ctx->vtbl->invoke(ctx);

    if (have_ms) {
        t1_ms = port->v->millis();
        uint64_t dt_ms = (t1_ms >= t0_ms) ? (t1_ms - t0_ms) : 0;
        ctx->last_inference_us_cached = (uint32_t)(dt_ms * 1000u);
    }

    return st;
}

/**
 * @brief Get an input tensor descriptor by index.
 *
 * @param ctx   AI context.
 * @param index Input tensor index.
 * @param out   Output tensor descriptor.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_get_input(omnia_ai_ctx_t* ctx,
                                     uint32_t index,
                                     omnia_ai_tensor_desc_t* out)
{
    if (!ctx || !ctx->vtbl || !out) return OMNIA_AI_ERR_INVALID;
    if (!ctx->initialized) return OMNIA_AI_ERR_NOT_INITIALIZED;
    return ctx->vtbl->get_input(ctx, index, out);
}

/**
 * @brief Get an output tensor descriptor by index.
 *
 * @param ctx   AI context.
 * @param index Output tensor index.
 * @param out   Output tensor descriptor.
 * @return Status code.
 */
omnia_ai_status_t omnia_ai_get_output(omnia_ai_ctx_t* ctx,
                                      uint32_t index,
                                      omnia_ai_tensor_desc_t* out)
{
    if (!ctx || !ctx->vtbl || !out) return OMNIA_AI_ERR_INVALID;
    if (!ctx->initialized) return OMNIA_AI_ERR_NOT_INITIALIZED;
    return ctx->vtbl->get_output(ctx, index, out);
}

/**
 * @brief Get the last inference duration in microseconds.
 *
 * The function prefers the backend-provided timing when available. If the backend
 * does not provide timing or returns 0, the wrapper-level coarse estimate is used.
 *
 * @param ctx AI context.
 * @return Inference duration in microseconds, or 0 if unavailable.
 */
uint32_t omnia_ai_get_last_inference_us(omnia_ai_ctx_t* ctx)
{
    if (!ctx || !ctx->vtbl) return 0;
    if (!ctx->initialized) return 0;

    /* Prefer backend-provided timer if available. */
    if (ctx->vtbl->get_last_inference_us) {
        uint32_t v = ctx->vtbl->get_last_inference_us(ctx);
        if (v != 0) return v;
    }

    /* Fallback: wrapper-level coarse estimate. */
    return ctx->last_inference_us_cached;
}
