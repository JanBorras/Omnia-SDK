// include/omnia_types.h
#ifndef OMNIA_TYPES_H
#define OMNIA_TYPES_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Status / errors (contract-wide) ---------- */
typedef enum {
  OMNIA_OK = 0,
  OMNIA_EINVAL,
  OMNIA_EIO,
  OMNIA_EBUSY,
  OMNIA_ETIMEDOUT,
  OMNIA_ENOMEM,
  OMNIA_ENOTSUP,
} omnia_status_t;

/* Helpers: fan el codi llegible i fan “d’arquitectura” al Cap 5 */
static inline int omnia_succeeded(omnia_status_t s) { return s == OMNIA_OK; }
static inline int omnia_failed(omnia_status_t s)    { return s != OMNIA_OK; }

/* Si prefereixes macros (mateix efecte, menys depuració amable):
#define OMNIA_SUCCEEDED(s) ((s) == OMNIA_OK)
#define OMNIA_FAILED(s)    ((s) != OMNIA_OK)
*/

/* ---------- Basic boolean (portable & explicit) ---------- */
typedef enum {
  OMNIA_FALSE = 0,
  OMNIA_TRUE  = 1
} omnia_bool_t;

/* ---------- Opaque handles (forward declarations) ---------- */
typedef struct omnia_port_s omnia_port_t; // opac

/* (Opcional, però útil per futur: una versió de l’SDK per diagnosi)
#define OMNIA_SDK_VERSION_MAJOR 0
#define OMNIA_SDK_VERSION_MINOR 1
#define OMNIA_SDK_VERSION_PATCH 0
*/

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_TYPES_H */
