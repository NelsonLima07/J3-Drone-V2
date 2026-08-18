/**
 * @file    i3c_bmp581_hal.h
 * @brief   Transporte I3C1 (interrupt) para BMP581 com IBI.
 * @date    2026-08-17
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * PORTAS (ver docs/pinout_map.md - secao I3C1):
 *   - I3C1_SCL = PB6, I3C1_SDA = PB7 (AF3, push-pull, pull-up)
 *   - BMP581 static address 0x46, dynamic address 0x46 (SETDASA).
 *   - Modo I3C SDR 12.5 MHz (push-pull).
 *   - IBI (In-Band Interrupt) para notificacao de dados prontos.
 *
 * Implementa imuc_transport_t sobre I3C1 com HAL_IT (sem DMA).
 * A funcao ler usa HAL_I3C_Ctrl_Transmit_IT + HAL_I3C_Ctrl_Receive_IT.
 * IBI e tratado em HAL_I3C_CtrlEventCallback e disponibiliza leitura
 * da nova amostra via i3c_bmp581_hal_ibi_pronto().
 */

#ifndef USR_SENSORS_I3C_BMP581_HAL_H
#define USR_SENSORS_I3C_BMP581_HAL_H

#include <stdint.h>
#include "sensors/imuc42688_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP581_I3C_ADDR  0x46U /**< endereco I3C static (SDO=GND) */

/**
 * @brief  Vincula @p t ao barramento I3C1 para o BMP581.
 *         Configura I3C1 como controller, envia SETDASA e ENEC.
 * @return 0 em caso de sucesso, < 0 em caso de falha.
 */
int i3c_bmp581_hal_vincula(imuc_transport_t *t);

/**
 * @brief  Retorna 1 se ha IBI pendente (dado pronto).
 *         Limpa o flag apos chamada.
 */
uint8_t i3c_bmp581_hal_ibi_pronto(void);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_I3C_BMP581_HAL_H */
