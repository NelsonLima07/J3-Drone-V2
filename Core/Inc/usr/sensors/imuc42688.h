/**
 * @file    imuc42688.h
 * @brief   Driver em C puro do IMU ICM-42688-P (TDK/InvenSense)
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Nao depende do HAL: barramento acessado via imuc_transport_t.
 * Escalas fixas usadas no J3_DroneV2: gyro +-2000 dps (8 kHz),
 * accel +-16 g (8 kHz), DRDY mapeado em INT1.
 */

#ifndef USR_SENSORS_IMUC42688_H
#define USR_SENSORS_IMUC42688_H

#include <stdint.h>
#include "sensors/imuc42688_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Registradores do ICM-42688-P (SPI) */
#define ICM_WHO_AM_I       0x75U  /**< WHO_AM_I              */
#define ICM_DEVICE_CONFIG  0x11U  /**< DEVICE_CONFIG         */
#define ICM_PWR_MGMT0      0x4EU  /**< PWR_MGMT0             */
#define ICM_GYRO_CONFIG0   0x4FU  /**< GYRO_CONFIG0          */
#define ICM_ACCEL_CONFIG0  0x50U  /**< ACCEL_CONFIG0         */
#define ICM_INT_SOURCE0    0x65U  /**< INT_SOURCE0           */
#define ICM_BURST_INICIO   0x1DU  /**< 1o registro do burst  */
#define ICM_BURST_TAMANHO  14U    /**< temp(2)+accel(6)+gyro(6) */

/* WHO_AM_I esperado do ICM-42688-P */
#define ICM_WHO_AM_I_VALOR 0x47U

/* Mascaras de bit (SPI) */
#define ICM_SPI_BIT_LEITURA  0x80U
#define ICM_SPI_BIT_ESCRITA  0x7FU

/**
 * @brief  Medida decodificada do IMU em unidades do SI.
 */
typedef struct {
  float temp_c;          /**< temperatura (°C)                  */
  float aceleracao_m_s2[3]; /**< {X, Y, Z} (m/s^2)            */
  float giro_rad_s[3];      /**< {X, Y, Z} (rad/s)            */
} imu_medida_t;

/**
 * @brief  Estado do driver do ICM-42688-P.
 */
typedef struct {
  const imuc_transport_t *transporte; /**< barramento vinculado       */
  uint8_t registros[ICM_BURST_TAMANHO];/**< ultima leitura bruta      */
  uint32_t contador_leituras;          /**< amostras processadas       */
  uint8_t pronta;                      /**< 1 quando ha medida valida  */
  imu_medida_t medida;                 /**< ultima medida decodificada */
} imuc42688_t;

/**
 * @brief  Inicializa a estrutura do driver e vincula o transporte.
 * @return 0 em caso de sucesso, < 0 em caso de argumento invalido.
 */
int imuc42688_inicializa(imuc42688_t *imu, const imuc_transport_t *transporte);

/**
 * @brief  Le o WHO_AM_I e confirma a presenca do sensor.
 * @return 1 se WHO_AM_I == 0x47, 0 caso contrario (ou falha).
 */
int imuc42688_verifica_id(imuc42688_t *imu);

/**
 * @brief  Configura o sensor para operacao 6-eixos low noise:
 *         soft reset, gyro +-2000 dps / 8 kHz, accel +-16 g / 8 kHz,
 *         DRDY mapeado em INT1 (INT_SOURCE0).
 * @return 0 em caso de sucesso, < 0 em caso de falha de escrita.
 */
int imuc42688_configura(imuc42688_t *imu);

/**
 * @brief  Decodifica um burst bruto de 14 bytes (temp + accel + gyro)
 *         iniciado no registro 0x1D e atualiza a ultima medida.
 * @param  dados  buffer de ICM_BURST_TAMANHO bytes.
 */
void imuc42688_processa_burst(imuc42688_t *imu, const uint8_t *dados);

/**
 * @brief  Ponteiro para a ultima medida decodificada (ou NULL se vazio).
 */
const imu_medida_t *imuc42688_medida(const imuc42688_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_IMUC42688_H */
