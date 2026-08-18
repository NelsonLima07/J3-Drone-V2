/**
 * @file    bmp581.h
 * @brief   Driver em C puro do barometro BMP581 (Bosch) via I3C.
 * @date    2026-08-17
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Nao depende do HAL: barramento acessado via imuc_transport_t.
 * Compensacao feita on-chip pelo sensor (divisao por 65536/64).
 * Modo NORMAL, oversampling T=1x / P=8x, ODR 25 Hz, IBI habilitado.
 * Leitura via I3C1 (endereco 0x46) no i3c_bmp581_hal.
 */

#ifndef USR_SENSORS_BMP581_H
#define USR_SENSORS_BMP581_H

#include <stdint.h>
#include "sensors/imuc42688_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP581_ENDERECO  0x46U /**< endereco I3C static (SDO baixo) */
#define BMP581_WHO_AM_I  0x50U /**< esperado no registro CHIP_ID (0x01) */

/* Registradores */
#define BMP581_REG_CHIP_ID     0x01U
#define BMP581_REG_INT_SOURCE  0x15U
#define BMP581_REG_OSR_CONFIG  0x1BU
#define BMP581_REG_DSP_CONFIG  0x31U
#define BMP581_REG_ODR_CONFIG  0x36U
#define BMP581_REG_STATUS      0x40U
#define BMP581_REG_TEMP_XLSB   0x1DU
#define BMP581_REG_PRESS_XLSB  0x20U
#define BMP581_REG_CMD         0x7EU

/* Soft reset */
#define BMP581_SOFT_RESET_CMD  0xB6U

/* OSR_CONFIG: osr_t=1x (001<<3), osr_p=8x (011<<1), press_en=1 */
#define BMP581_OSR_CONFIG_VALOR  0x0FU
/* DSP_CONFIG: comp_pt_en=11 (ambas compensadas) */
#define BMP581_DSP_CONFIG_VALOR  0x03U
/* INT_SOURCE: drdy_en=1 */
#define BMP581_INT_SOURCE_VALOR  0x01U
/* ODR_CONFIG: odr=25Hz (0x0B<<2), pwr_mode=NORMAL (01) */
#define BMP581_ODR_CONFIG_VALOR  0x2DU

typedef struct {
  float temperatura_c;  /**< temperatura compensada (C)          */
  float pressao_pa;     /**< pressao compensada (Pa)             */
  float altitude_m;     /**< altitude em relacao a P0 (m)        */
} baro_medida_t;

typedef struct {
  const imuc_transport_t *transporte; /**< barramento vinculado   */
  uint8_t  ok;                        /**< 1 quando id ok         */
  uint32_t contador_leituras;         /**< amostras processadas   */
  float    p0_pa;                     /**< pressao ao nivel do mar */
  baro_medida_t medida;
} bmp581_t;

/**
 * @brief  Inicializa a estrutura e vincula o transporte.
 * @return 0 em caso de sucesso, < 0 em caso de argumento invalido.
 */
int bmp581_inicializa(bmp581_t *baro, const imuc_transport_t *transporte);

/**
 * @brief  Confirma a presenca (ID == 0x50).
 * @return 1 se correto, 0 caso contrario (ou falha de barramento).
 */
int bmp581_verifica_id(bmp581_t *baro);

/**
 * @brief  Soft reset e configura modo NORMAL com IBI habilitado.
 * @return 0 em caso de sucesso, < 0 em caso de falha.
 */
int bmp581_configura(bmp581_t *baro);

/**
 * @brief  Le 6 bytes (temp+press) ja compensados on-chip.
 *         Converte por divisao (÷65536 / ÷64) e atualiza a medida.
 * @return 1 quando ha uma amostra nova; 0 em falha.
 */
uint8_t bmp581_processa(bmp581_t *baro);

/**
 * @brief  Formula barometrica: altitude (m) a partir da pressao.
 */
float bmp581_altitude(float pressao_pa, float p0_pa);

/** Ponteiro para a ultima medida decodificada (ou NULL se vazio). */
const baro_medida_t *bmp581_medida(const bmp581_t *baro);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_BMP581_H */
