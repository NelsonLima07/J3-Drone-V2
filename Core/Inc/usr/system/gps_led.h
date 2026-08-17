/**
 * @file    gps_led.h
 * @brief   LED dedicado de status do GPS (PB2).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Aceso SOLIDO quando o GPS esta pronto para o modo assistido
 * (fix valido + home gravado + HDOP dentro do teto). Piscando a
 * 2 Hz enquanto nao ha condicao de armar. Acesso direto ao GPIO.
 */

#ifndef USR_SYSTEM_GPS_LED_H
#define USR_SYSTEM_GPS_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Configura PB2 como saida push-pull (ativo alto).
 */
void gps_led_inicializa(void);

/**
 * @brief  Atualiza o LED: solido com gps_ok, pisca 2 Hz sem fix.
 * @param  gps_ok    1 quando o GPS esta pronto para armar.
 * @param  agora_ms  tempo atual (ms) para a fase do pisca-pisca.
 */
void gps_led_atualiza(uint8_t gps_ok, uint32_t agora_ms);

#ifdef __cplusplus
}
#endif

#endif /* USR_SYSTEM_GPS_LED_H */
