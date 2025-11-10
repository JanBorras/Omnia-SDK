// include/omnia_types.h
#ifndef OMNIA_TYPES_H
#define OMNIA_TYPES_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  OMNIA_OK = 0,
  OMNIA_EINVAL,
  OMNIA_EIO,
  OMNIA_EBUSY,
  OMNIA_ETIMEDOUT,
  OMNIA_ENOMEM,
  OMNIA_ENOTSUP,
} omnia_status_t;

typedef struct omnia_port_s omnia_port_t; // opac

#ifdef __cplusplus
}
#endif
#endif
