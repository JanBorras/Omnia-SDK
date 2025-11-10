// ports/common/rtos_freertos.c
#include "omnia_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

static omnia_mutex_t mutex_create_impl(void){ return xSemaphoreCreateMutex(); }
static void mutex_lock_impl(omnia_mutex_t m){ xSemaphoreTake(m, portMAX_DELAY); }
static void mutex_unlock_impl(omnia_mutex_t m){ xSemaphoreGive(m); }
static void mutex_destroy_impl(omnia_mutex_t m){ vSemaphoreDelete(m); }

static omnia_task_t task_create_impl(omnia_task_fn fn, const char* name,
                                     uint32_t stack, void* arg, int prio){
  TaskHandle_t h = NULL;
  xTaskCreate((TaskFunction_t)fn, name, stack/sizeof(StackType_t), arg, prio, &h);
  return h;
}
static void task_delete_impl(omnia_task_t t){ vTaskDelete((TaskHandle_t)t); }
static void yield_impl(void){ taskYIELD(); }
static void sleep_ms_impl(uint32_t ms){ vTaskDelay(pdMS_TO_TICKS(ms)); }

static omnia_sem_t sem_create_impl(uint32_t initial, uint32_t max){
  (void)max; // FreeRTOS bin/counting: podries fer xSemaphoreCreateCounting
  SemaphoreHandle_t s = xSemaphoreCreateCounting(0xFFFF, initial);
  return s;
}
static bool sem_take_impl(omnia_sem_t s, uint32_t to){ return xSemaphoreTake(s, pdMS_TO_TICKS(to)) == pdTRUE; }
static void sem_give_impl(omnia_sem_t s){ xSemaphoreGive(s); }
static void sem_destroy_impl(omnia_sem_t s){ vSemaphoreDelete(s); }

static omnia_queue_t queue_create_impl(size_t isz, size_t len){ return xQueueCreate(len, isz); }
static bool queue_send_impl(omnia_queue_t q, const void* item, uint32_t to){
  return xQueueSend(q, item, pdMS_TO_TICKS(to)) == pdTRUE;
}
static bool queue_recv_impl(omnia_queue_t q, void* item, uint32_t to){
  return xQueueReceive(q, item, pdMS_TO_TICKS(to)) == pdTRUE;
}
static void queue_destroy_impl(omnia_queue_t q){ vQueueDelete(q); }

static void timer_cb_thunk(TimerHandle_t t){
  omnia_timer_cb cb = (omnia_timer_cb)pvTimerGetTimerID(t);
  if (cb) cb((void*)t); // o guarda arg en una altra estructura
}
static omnia_timer_t timer_create_impl(uint32_t period_ms, bool auto_reload,
                                       omnia_timer_cb cb, void* arg){
  (void)arg; // simplificació: usa pvTimerGetTimerID per guardar cb
  TimerHandle_t h = xTimerCreate("om", pdMS_TO_TICKS(period_ms),
                                 auto_reload, (void*)cb, timer_cb_thunk);
  return h;
}
static bool timer_start_impl(omnia_timer_t t){ return xTimerStart(t, 0) == pdPASS; }
static bool timer_stop_impl(omnia_timer_t t){ return xTimerStop(t, 0) == pdPASS; }
static void timer_destroy_impl(omnia_timer_t t){ xTimerDelete(t, 0); }

static bool in_isr_impl(void){ return xPortIsInsideInterrupt() != 0; }

static const omnia_rtos_ops_t RTOS_FREERTOS = {
  .mutex_create = mutex_create_impl,
  .mutex_lock   = mutex_lock_impl,
  .mutex_unlock = mutex_unlock_impl,
  .mutex_destroy= mutex_destroy_impl,
  .task_create  = task_create_impl,
  .task_delete  = task_delete_impl,
  .yield        = yield_impl,
  .sleep_ms     = sleep_ms_impl,
  .sem_create   = sem_create_impl,
  .sem_take     = sem_take_impl,
  .sem_give     = sem_give_impl,
  .sem_destroy  = sem_destroy_impl,
  .queue_create = queue_create_impl,
  .queue_send   = queue_send_impl,
  .queue_recv   = queue_recv_impl,
  .queue_destroy= queue_destroy_impl,
  .timer_create = timer_create_impl,
  .timer_start  = timer_start_impl,
  .timer_stop   = timer_stop_impl,
  .timer_destroy= timer_destroy_impl,
  .in_isr       = in_isr_impl
};

const omnia_rtos_ops_t* omnia_rtos_freertos_ops(void){ return &RTOS_FREERTOS; }
