/**
 * @file    bmp280.h
 * @brief   Driver em C puro do barometro BMP280 (Bosch).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Nao depende do HAL: barramento acessado via imuc_transport_t.
 * Modo normal, oversampling de temperatura 1x e pressao 4x, filtro
 * IIR 4. Altitude calculada pela formula barometrica com P0
 * configuravel (padrao 101325 Pa). Leitura via I2C2 (endereco 0x76)
 * no i2c2_hal (ver docs/pinout_map.md - secao I2C2).
 */

#ifndef USR_SENSORS_BMP280_H
#define USR_SENSORS_BMP280_H

#include <stdint.h>
#include "sensors/imuc42688_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP280_ENDERECO  0x76U /**< endereco I2C 7 bits (SDO baixo) */
#define BMP280_WHO_AM_I  0x58U /**< esperado no registro ID (0xD0) */

/* Registradores */
#define BMP280_REG_ID        0xD0U
#define BMP280_REG_RESET     0xE0U
#define BMP280_REG_STATUS    0xF3U
#define BMP280_REG_CTRL_MEAS 0xF4U
#define BMP280_REG_CONFIG    0xF5U
#define BMP280_REG_PRESS_MSB 0xF7U
#define BMP280_REG_TEMP_MSB  0xFAU
#define BMP280_REG_CALIB      0x88U /**< 24 bytes de calibracao     */

/* Calibracao: dig_T1 u16 em [0]; dig_T2..T3 s16 em [1..2];
 * dig_P1 u16 em [3]; dig_P2..P9 s16 em [4..11]. */
#define BMP280_DIG_TAMANHO 12

typedef struct {
  float temperatura_c;  /**< temperatura compensada (C)          */
  float pressao_pa;     /**< pressao compensada (Pa)             */
  float altitude_m;     /**< altitude em relacao a P0 (m)        */
} baro_medida_t;

typedef struct {
  const imuc_transport_t *transporte; /**< barramento vinculado   */
  uint8_t  ok;                        /**< 1 quando id/cal ok      */
  uint32_t contador_leituras;         /**< amostras processadas    */
  float    p0_pa;                     /**< pressao ao nivel do mar */
  int16_t  dig[BMP280_DIG_TAMANHO];   /**< calibracao compativel   */
  baro_medida_t medida;
} bmp280_t;

/**
 * @brief  Inicializa a estrutura e vincula o transporte.
 * @return 0 em caso de sucesso, < 0 em caso de argumento invalido.
 */
int bmp280_inicializa(bmp280_t *baro, const imuc_transport_t *transporte);

/**
 * @brief  Confirma a presenca (ID == 0x58).
 * @return 1 se correto, 0 caso contrario (ou falha de barramento).
 */
int bmp280_verifica_id(bmp280_t *baro);

/**
 * @brief  Soft reset, le calibracao e configura modo normal.
 * @return 0 em caso de sucesso, < 0 em caso de falha.
 */
int bmp280_configura(bmp280_t *baro);

/**
 * @brief  Le pressao (3 bytes) e temperatura (3 bytes), compensa
 *         e atualiza a medida (inclui altitude via P0).
 * @return 1 quando ha uma amostra nova; 0 em falha.
 */
uint8_t bmp280_processa(bmp280_t *baro);

/**
 * @brief  Funcao pura de compensacao (testavel no host).
 * @param  dig  calibracao lida do sensor (BMP280_DIG_TAMANHO).
 * @param  t_raw  temperatura crua (20 bits).
 * @param  p_raw  pressao crua (20 bits).
 * @param  temperatura_c  saida em graus Celsius.
 * @param  pressao_pa    saida em pascal.
 */
void bmp280_compensa(const int16_t *dig, int32_t t_raw, int32_t p_raw,
                     float *temperatura_c, float *pressao_pa);

/**
 * @brief  Formula barometrica: altitude (m) a partir da pressao.
 */
float bmp280_altitude(float pressao_pa, float p0_pa);

/** Ponteiro para a ultima medida decodificada (ou NULL se vazio). */
const baro_medida_t *bmp280_medida(const bmp280_t *baro);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_BMP280_H */
