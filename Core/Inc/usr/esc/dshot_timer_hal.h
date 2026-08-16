/**
 * @file    dshot_timer_hal.h
 * @brief   Saida DShot600 pelos 4 ESCs (TIM1 PA8..PA11 + GPDMA1 CH3..CH6).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Implementado 100% em codigo (sem alteracao do CubeMX).
 * Glue HAL: compilado apenas no firmware (target raiz), como hardware_glue.
 */

#ifndef USR_ESC_DSHOT_TIMER_HAL_H
#define USR_ESC_DSHOT_TIMER_HAL_H

#include <stdint.h>

#include "control/mixer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Configura TIM1 (PWM DMA por canal), GPIO PA8..11 (AF1), DMA
 *         GPDMA1 CH3..CH6 (requests 58..61) e inicia os frames zero.
 * @retval 0 em sucesso, -1 em erro.
 */
int dshot_timer_inicializa(void);

/**
 * @brief  Prepara os frames dos 4 motores para a proxima janela de DMA.
 *         Chamado a cada iteracao do controle (contexto de ISR prio 5).
 * @param  saida comandos normalizados 0..1 de cada motor.
 * @param  armado 1 = envia throttle real (48..2047); 0 = envia frame 0.
 *         Na borda de subida envia DSHOT_FRAMES_ARM frames zero antes.
 */
void dshot_escreve(const saida_misturador_t *saida, uint8_t armado);

#ifdef __cplusplus
}
#endif

#endif /* USR_ESC_DSHOT_TIMER_HAL_H */
