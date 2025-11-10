// ports/stm32h7/port_stm32_freertos.c
extern const omnia_port_vtable_t STM32_IO;       // ja la tens
extern const omnia_rtos_ops_t*   omnia_rtos_freertos_ops(void);

void omnia_port_stm32_freertos_init(void){
  uint32_t caps = (1u<<0) /*RTOS*/ | (1u<<1) /*SPI_DMA? si disponible*/;
  omnia_port_register_ex(&STM32_IO, omnia_rtos_freertos_ops(), caps, NULL);
}
