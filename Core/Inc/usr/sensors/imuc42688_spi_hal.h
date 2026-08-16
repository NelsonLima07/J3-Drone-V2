/**
 * @file    imuc42688_spi_hal.h
 * @brief   Glue HAL (STM32 + SPI1 + GPDMA1) do driver do ICM-42688-P
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Este arquivo e compilado apenas no firmware (fora da lib usr).
 * Implementa o imuc_transport_t sobre o SPI1 com CS em PA4 e o fluxo
 * nao-bloqueante de data ready (INT1/PB0 -> burst de 14 bytes via DMA).
 */

#ifndef USR_SENSORS_IMUC42688_SPI_HAL_H
#define USR_SENSORS_IMUC42688_SPI_HAL_H

#include "sensors/imuc42688.h"

#ifdef __cplusplus
extern "C" {
#endif

/**< Callback chamado a cada medida nova (contexto de interrupcao). */
typedef void (*imuc_hal_ao_medir_t)(const imu_medida_t *medida);

/**
 * @brief  Vincula o driver ao transporte SPI e ao callback de medida.
 * @return 0 em caso de sucesso.
 */
int imuc_hal_vincular(imuc42688_t *imu, imuc_hal_ao_medir_t ao_medir);

/**
 * @brief  Inicia a leitura nao bloqueante do burst (14 bytes) via DMA.
 *         O CS e baixado; HAL_SPI_TxRxCpltCallback completa a transacao.
 * @return 0 em caso de sucesso.
 */
int imuc_hal_inicia_burst(void);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_IMUC42688_SPI_HAL_H */
