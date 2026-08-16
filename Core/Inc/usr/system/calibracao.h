/**
 * @file    calibracao.h
 * @brief   Calibracao do IMU (bias do gyro + referencia do accel) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Acumula amostras em contexto de interrupcao e calcula a media ao
 * finalizar. Nao depende do HAL.
 */

#ifndef USR_SYSTEM_CALIBRACAO_H
#define USR_SYSTEM_CALIBRACAO_H

#include <stdint.h>
#include "sensors/imuc42688.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double giro_soma[3];
  double accel_soma[3];
  float  giro_bias[3];  /**< bias medio do gyro (rad/s)           */
  float  accel_ref[3];  /**< gravidade media medida (m/s^2)       */
  uint32_t n;           /**< amostras acumuladas                  */
  uint32_t meta;        /**< amostras alvo (0 = sem limite)       */
  volatile uint8_t ativa;
  volatile uint8_t concluida;
} calibracao_t;

/**
 * @brief  Zera o estado (calibracao desativada).
 */
void calibracao_inicializa(calibracao_t *cal);

/**
 * @brief  Inicia uma coleta. Zera as somas e marca como ativa.
 * @param  meta_amostras alvo; ao atingir, a coleta finaliza sozinha.
 */
void calibracao_inicia(calibracao_t *cal, uint32_t meta_amostras);

/**
 * @brief  Acumula uma amostra (chamado a cada medida do IMU).
 * @return 1 quando a coleta terminou nesta chamada, 0 caso contrario,
 *         < 0 se inativa/argumentos invalidos.
 */
int calibracao_processa(calibracao_t *cal, const imu_medida_t *m);

/**
 * @brief  Calcula as medias (bias do gyro e referencia do accel).
 * @return 0 em caso de sucesso, < 0 caso contrario.
 */
int calibracao_finaliza(calibracao_t *cal);

/**
 * @brief  1 quando uma coleta esta em andamento.
 */
uint8_t calibracao_ativa(const calibracao_t *cal);

/**
 * @brief  1 quando a ultima coleta concluiu com sucesso.
 */
uint8_t calibracao_concluida(const calibracao_t *cal);

/**
 * @brief  Remove o bias do gyro de uma medida (mantem o accel).
 */
void calibracao_aplica_bias(const calibracao_t *cal,
                            const imu_medida_t *entrada,
                            imu_medida_t *saida);

#ifdef __cplusplus
}
#endif

#endif /* USR_SYSTEM_CALIBRACAO_H */
