/**
 * @file    imuc42688_transport.h
 * @brief   Transporte abstrato (SPI/I2C) do driver do ICM-42688-P
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * A separacao de transporte permite testar o nucleo do driver no host
 * (mock) e usar SPI+DMA no firmware sem acoplar a lib usr ao HAL.
 */

#ifndef USR_SENSORS_IMUC42688_TRANSPORT_H
#define USR_SENSORS_IMUC42688_TRANSPORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Operacoes de barramento do ICM-42688-P.
 *
 * Todas as funcoes sao bloqueantes (sincronas) e devem retornar 0 em
 * caso de sucesso e != 0 em caso de falha de comunicacao.
 */
typedef struct imuc_transport {
  void *ctx;                          /**< contexto opaco do barramento   */

  /**< Le @p n bytes do registrador @p endereco (burst). */
  int (*ler)(void *ctx, uint8_t endereco, uint8_t *dados, uint16_t n);

  /**< Escreve @p n bytes a partir do registrador @p endereco. */
  int (*escrever)(void *ctx, uint8_t endereco, const uint8_t *dados, uint16_t n);

  /**< Atraso em microssegundos (opcional, pode ser NULL). */
  void (*atraso_us)(void *ctx, uint32_t us);
} imuc_transport_t;

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_IMUC42688_TRANSPORT_H */
