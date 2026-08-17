/**
 * @file    lis3mdl.h
 * @brief   Driver em C puro do magnetometro LIS3MDL (ST).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Nao depende do HAL: barramento acessado via imuc_transport_t.
 * Escalas fixas usadas no J3_DroneV2: +-4 gauss, ODR 100 Hz,
 * modo continuo, BDU ativado. Leitura via I2C2 (endereco 0x1C) no
 * i2c2_hal (ver docs/pinout_map.md - secao I2C2).
 */

#ifndef USR_SENSORS_LIS3MDL_H
#define USR_SENSORS_LIS3MDL_H

#include <stdint.h>
#include "math/vector3.h"
#include "sensors/imuc42688_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIS3MDL_ENDERECO  0x1CU /**< endereco I2C 7 bits           */
#define LIS3MDL_WHO_AM_I  0x3DU /**< esperado no registro 0x0F    */

/* Registradores */
#define LIS3MDL_REG_WHO_AM_I    0x0FU
#define LIS3MDL_REG_CTRL1       0x20U
#define LIS3MDL_REG_CTRL2       0x21U
#define LIS3MDL_REG_CTRL3       0x22U
#define LIS3MDL_REG_CTRL4       0x23U
#define LIS3MDL_REG_CTRL5       0x24U
#define LIS3MDL_REG_STATUS      0x27U
#define LIS3MDL_REG_OUT_X_L     0x28U

/* Sensibilidade em +-4 gauss: 6842 LSB/gauss */
#define LIS3MDL_ESCALA_LSB_GAUSS 6842.0f

typedef vetor3_t mag_medida_t; /**< {x, y, z} em gauss             */

typedef struct {
  const imuc_transport_t *transporte; /**< barramento vinculado     */
  uint8_t  ok;                     /**< 1 quando id/configura ok     */
  uint32_t contador_leituras;      /**< amostras processadas         */
  mag_medida_t medida;             /**< ultima medida em gauss       */
} lis3mdl_t;

/**
 * @brief  Inicializa a estrutura e vincula o transporte.
 * @return 0 em caso de sucesso, < 0 em caso de argumento invalido.
 */
int lis3mdl_inicializa(lis3mdl_t *mag, const imuc_transport_t *transporte);

/**
 * @brief  Confirma a presenca (WHO_AM_I == 0x3D).
 * @return 1 se correto, 0 caso contrario (ou falha de barramento).
 */
int lis3mdl_verifica_id(lis3mdl_t *mag);

/**
 * @brief  Configura +-4 gauss, 100 Hz, modo continuo, BDU.
 * @return 0 em caso de sucesso, < 0 em caso de falha de escrita.
 */
int lis3mdl_configura(lis3mdl_t *mag);

/**
 * @brief  Le OUT_X_L..OUT_Z_H (6 bytes) e decodifica em gauss.
 * @return 1 quando ha uma amostra nova; 0 se dados nao prontos
 *         ou falha de leitura.
 */
uint8_t lis3mdl_processa(lis3mdl_t *mag);

/** Ponteiro para a ultima medida decodificada (ou NULL se vazio). */
const mag_medida_t *lis3mdl_medida(const lis3mdl_t *mag);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_LIS3MDL_H */
