// core/ai/omnia_ai_internal.h
#ifndef OMNIA_AI_INTERNAL_H
#define OMNIA_AI_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "omnia_ai.h"

/**
 * @file omnia_ai_internal.h
 * @brief Internal helpers for the Omnia AI core–backend boundary.
 *
 * This header defines a minimal set of internal accessors used by
 * AI backends to interact with the core runtime without depending
 * on the concrete layout of ::omnia_ai_ctx_t.
 *
 * @note
 * This is an INTERNAL header.
 * It must NOT be included by applications or user-facing code.
 */

/**
 * @brief Get the backend-private state associated with a context.
 *
 * This function returns an opaque pointer owned by the backend.
 * The core does not interpret its contents.
 *
 * @param ctx Pointer to an initialized AI context.
 * @return Opaque backend state pointer, or NULL if ctx is NULL.
 */
void* omnia_ai__get_backend_state(omnia_ai_ctx_t* ctx);

/**
 * @brief Set the backend-private state associated with a context.
 *
 * The backend is responsible for allocating and managing the lifetime
 * of this state. The core stores it as an opaque pointer.
 *
 * @param ctx   Pointer to an AI context.
 * @param state Opaque backend state pointer (may be NULL).
 */
void  omnia_ai__set_backend_state(omnia_ai_ctx_t* ctx, void* state);

/**
 * @brief Get the Omnia port associated with an AI context.
 *
 * This provides access to timing, logging, and other platform services
 * through the Omnia port vtable, without exposing global state or
 * requiring backends to call omnia_port_get() directly.
 *
 * @param ctx Pointer to an AI context.
 * @return Pointer to the associated Omnia port, or NULL if not set.
 */
const omnia_port_t* omnia_ai__get_port(const omnia_ai_ctx_t* ctx);

/**
 * @brief Get the cached inference time (microseconds) stored by the core.
 *
 * This value is typically populated by the wrapper-level timing logic
 * and serves as a coarse fallback when the backend does not provide
 * a more precise measurement.
 *
 * @param ctx Pointer to an AI context.
 * @return Last cached inference time in microseconds, or 0 if unavailable.
 */
uint32_t omnia_ai__get_last_inference_us_cached(const omnia_ai_ctx_t* ctx);

/**
 * @brief Update the cached inference time (microseconds) stored by the core.
 *
 * Intended to be used by backends as a fallback synchronization mechanism
 * to keep profiling information consistent across layers.
 *
 * @param ctx Pointer to an AI context.
 * @param us  Inference time in microseconds.
 */
void     omnia_ai__set_last_inference_us_cached(omnia_ai_ctx_t* ctx, uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_AI_INTERNAL_H */
