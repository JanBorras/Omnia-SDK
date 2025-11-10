// include/omnia_rtos.h
#ifndef OMNIA_RTOS_H
#define OMNIA_RTOS_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef void* omnia_task_t;
typedef void* omnia_sem_t;
typedef void* omnia_queue_t;
typedef void* omnia_timer_t;

typedef void (*omnia_task_fn)(void* arg);
typedef void (*omnia_timer_cb)(void* arg);

typedef struct {
  // Sincronització bàsica
  omnia_mutex_t (*mutex_create)(void);
  void          (*mutex_lock)(omnia_mutex_t);
  void          (*mutex_unlock)(omnia_mutex_t);
  void          (*mutex_destroy)(omnia_mutex_t);

  // Tasques
  omnia_task_t  (*task_create)(omnia_task_fn fn, const char* name,
                               uint32_t stack, void* arg, int prio);
  void          (*task_delete)(omnia_task_t t);
  void          (*yield)(void);
  void          (*sleep_ms)(uint32_t ms);

  // Senyals / cues
  omnia_sem_t   (*sem_create)(uint32_t initial, uint32_t max);
  bool          (*sem_take)(omnia_sem_t, uint32_t timeout_ms);
  void          (*sem_give)(omnia_sem_t);
  void          (*sem_destroy)(omnia_sem_t);

  omnia_queue_t (*queue_create)(size_t item_size, size_t len);
  bool          (*queue_send)(omnia_queue_t, const void* item, uint32_t timeout_ms);
  bool          (*queue_recv)(omnia_queue_t, void* item, uint32_t timeout_ms);
  void          (*queue_destroy)(omnia_queue_t);

  // Timers d’alt nivell
  omnia_timer_t (*timer_create)(uint32_t period_ms, bool auto_reload,
                                omnia_timer_cb cb, void* arg);
  bool          (*timer_start)(omnia_timer_t);
  bool          (*timer_stop)(omnia_timer_t);
  void          (*timer_destroy)(omnia_timer_t);

  // Context
  bool          (*in_isr)(void); // útil per variants FromISR on calgui
} omnia_rtos_ops_t;

#endif
