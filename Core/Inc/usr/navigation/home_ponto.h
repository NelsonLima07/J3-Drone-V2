/**
 * @file    home_ponto.h
 * @brief   Ponto de origem (home) para RTH em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Regra do piloto: enquanto DESARMADO, cada fix valido com erro
 * estimado MENOR atualiza o home (melhor candidato). No momento de
 * armar (borda ESPERA->VOO) o home e travado = ponto de decolagem.
 * Durante o voo ele fica congelado. Ao desarmar volta a procurar um
 * candidato melhor para o proximo voo.
 *
 * O "erro estimado" vem do gps_nmea (hdop * uere) e deve ser
 * informado via gps_medida_t.erro_estimado_m.
 */

#ifndef USR_NAVIGATION_HOME_PONTO_H
#define USR_NAVIGATION_HOME_PONTO_H

#include <stdint.h>
#include "serial/gps_nmea.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t  valido;      /**< 1 quando ha um home aceitavel           */
  uint8_t  travado;     /**< 1 durante o voo (congelado)            */
  float    lat_deg;     /**< latitude do home (graus)               */
  float    lon_deg;     /**< longitude do home (graus)              */
  float    alt_m;       /**< altitude do home (m)                   */
  float    erro_m;      /**< menor erro estimado encontrado (m)     */
} home_ponto_t;

typedef struct {
  float erro_max_m;     /**< teto do erro estimado para aceitar     */
} config_home_t;

/** Configuracao padrao: erro maximo 15 m. */
void config_home_padrao(config_home_t *cfg);

void home_ponto_inicializa(home_ponto_t *home, const config_home_t *cfg);

/**
 * @brief  Atualiza o melhor candidato (somente desarmado).
 * @note   Qualquer fix valido vira candidato inicial; so atualiza o
 *         home quando o novo erro estimado for MENOR que o gravado.
 *         O teto erro_max_m e usado pelo supervisor (gps_ok) para
 *         liberar o armar.
 */
void home_ponto_atualiza(home_ponto_t *home, const gps_medida_t *gps);

/**
 * @brief  Trava o home no armar (ponto de decolagem). Se a medida
 *         atual for melhor que a gravada, usa a atual.
 */
void home_ponto_trava(home_ponto_t *home, const gps_medida_t *gps);

/**
 * @brief  Destrava ao desarmar: invalida e reinicia a busca do
 *         melhor candidato (coordenadas antigas ficam para depuracao).
 */
void home_ponto_destrava(home_ponto_t *home);

/** 1 se o home esta valido e travado (utilizavel no voo). */
uint8_t home_ponto_valido(const home_ponto_t *home);

#ifdef __cplusplus
}
#endif

#endif /* USR_NAVIGATION_HOME_PONTO_H */
