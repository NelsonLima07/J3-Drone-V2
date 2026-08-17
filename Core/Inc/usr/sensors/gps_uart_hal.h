/**
 * @file    gps_uart_hal.h
 * @brief   Transporte USART3 + DMA (GPDMA1 CH7) do GPS BN-220.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Glue somente firmware (usa HAL). Le a USART3 com DMA Normal
 * (GPDMA1 CH7, requisicao GPDMA1_REQUEST_USART3_RX = 25) em modo
 * ReceiveToIdle (o GPDMA do H5 nao tem circular classico) e expoe
 * um "drain" nao bloqueante para o parser NMEA (gps_nmea.c).
 */

#ifndef USR_SENSORS_GPS_UART_HAL_H
#define USR_SENSORS_GPS_UART_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Re-inicializa a USART3 em 9600 8N1 (padrao do BN-220; o
 *         CubeMX gera 115200) e arma a DMA GPDMA1 CH7 em modo Normal.
 * @return 0 em caso de sucesso, < 0 em caso de falha.
 */
int gps_uart_inicializa(void);

/**
 * @brief  Copia ate @p max bytes recebidos desde a ultima chamada.
 * @return quantidade de bytes copiados (0 se vazio).
 */
uint32_t gps_uart_ler(uint8_t *buf, uint32_t max);

/**
 * @brief  Evento de RX do HAL (TC ou IDLE na USART3): copia os bytes
 *         recebidos para o ring e rearma a DMA.
 * @param  size quantidade de bytes recebidos no bloco.
 */
void gps_uart_rx_event(uint16_t size);

/**
 * @brief  Erro de RX do HAL (overrun/noise/frame na USART3): rearma a DMA.
 */
void gps_uart_erro(void);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_GPS_UART_HAL_H */
