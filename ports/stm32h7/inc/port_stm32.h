#ifndef OMNIA_PORT_STM32_H
#define OMNIA_PORT_STM32_H

#include <stdint.h>
#include "omnia_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicialitza el port STM32 i registra el vtable amb Omnia.
 *
 * S'ha de cridar un cop a l'arrencada abans d'utilitzar drivers que
 * depenguin de omnia_port (p. ex. el driver ST7735).
 */
void omnia_port_stm32_init(void);

/**
 * @brief Helper per empaquetar (GPIOx, PIN) en un omnia_gpio_t opac.
 *
 * Exemple d'ús (CM7 main.c):
 *   d.pin_cs  = omnia_mkpin(GPIOB, GPIO_PIN_9);
 *   d.pin_dc  = omnia_mkpin(GPIOB, GPIO_PIN_7);
 *   d.pin_rst = omnia_mkpin(GPIOB, GPIO_PIN_8);
 *   d.pin_bl  = omnia_mkpin(GPIOB, GPIO_PIN_6);
 *
 * Aquí `port` és un punter qualsevol (normalment GPIO_TypeDef*),
 * però es passa com a void* per evitar dependències fortes del HAL
 * en el header.
 */
omnia_gpio_t omnia_mkpin(void *port, uint16_t pin);

/* Start ADC1 DMA ring (used by your app start_sampling()) */
omnia_status_t omnia_port_stm32_adc1_dma_start(void);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_PORT_STM32_H */
