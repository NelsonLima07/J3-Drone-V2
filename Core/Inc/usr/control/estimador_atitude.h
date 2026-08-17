/**
 * @file    estimador_atitude.h
 * @brief   Estimador de atitude por fusao complementar (giroscopio + acelerometro)
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Estado: quaternion q de rotacao corpo -> mundo (NED).
 * - Integracao do giroscopio a cada amostra (8 kHz).
 * - Correcao de rol/arfagem pela direcao da gravidade medida no acelerometro,
 *   aplicada apenas quando a magnitude esta proxima de 1 g (evita corrigir
 *   durante aceleracao linear forte).
 * - Correcao de guinada pelo magnetometro (Mahony), aplicada via
 *   estimador_atitude_atualiza_mag() quando ha amostra nova do LIS3MDL
 *   (I2C2). A declinacao magnetica entra como configuracao.
 */

#ifndef USR_CONTROL_ESTIMADOR_ATITUDE_H
#define USR_CONTROL_ESTIMADOR_ATITUDE_H

#include <stdint.h>

#include "math/math_types.h"
#include "math/quaternion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float ganho;          /* Kp: taxa de correcao (rad/s) por erro unitario  */
  float g_min;          /* magnitude minima do accel em g para corrigir    */
  float g_max;          /* magnitude maxima do accel em g para corrigir    */
  float ganho_mag;      /* Kp da correcao de guinada pelo magnetometro     */
  float declinacao_rad; /* declinacao magnetica (rad), + = leste           */
} config_estimador_t;

typedef struct {
  quat_t q;                 /* rotacao corpo -> mundo (NED) */
  config_estimador_t config;
  uint32_t contador;        /* amostras processadas         */
} estimador_atitude_t;

/** Configuracao padrao: ganho 0.3, faixa 0.5g..2g. */
void estimador_config_padrao(config_estimador_t *cfg);

/**
 * @brief  Inicializa o estimador com a atitude inicial dada (rad).
 */
void estimador_atitude_inicializa(estimador_atitude_t *est,
                                  const config_estimador_t *cfg,
                                  float rol0, float arfagem0, float guinada0);

/**
 * @brief  Avanca a estimativa com uma amostra (giro em rad/s, accel em m/s^2).
 */
void estimador_atitude_atualiza(estimador_atitude_t *est,
                                const float giro[3], const float accel[3],
                                float dt);

/**
 * @brief  Correcao de guinada com o magnetometro (campo em gauss,
 *         frame do corpo) e um passo de integracao igual ao de atualiza().
 * @note   Use quando houver amostra nova do LIS3MDL (ex.: 100 Hz).
 */
void estimador_atitude_atualiza_mag(estimador_atitude_t *est,
                                    const float mag[3], float dt);

/** Euler {rol, arfagem, guinada} em rad. */
vetor3_t estimador_atitude_obtem_euler(const estimador_atitude_t *est);

/** Quaternion da estimativa atual (corpo -> mundo). */
quat_t estimador_atitude_obtem_quat(const estimador_atitude_t *est);

#ifdef __cplusplus
}
#endif

#endif /* USR_CONTROL_ESTIMADOR_ATITUDE_H */
