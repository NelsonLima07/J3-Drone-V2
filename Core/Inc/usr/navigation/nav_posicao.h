/**
 * @file    nav_posicao.h
 * @brief   Geometria de navegacao (plano local tangente) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Usa a aproximacao equiretangular para curtas distancias (valida
 * ate ~100 km): converte lat/lon em metros de Norte/Leste em relacao
 * a uma origem e fornece rumo, erro NED e raio de chegada. Serve
 * para position hold, ir-para-waypoint e RTH.
 */

#ifndef USR_NAVIGATION_NAV_POSICAO_H
#define USR_NAVIGATION_NAV_POSICAO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_RAIO_TERRA_M 6371000.0f

typedef struct {
  uint8_t  alvo_valido;   /**< 1 quando ha um alvo definido         */
  float    lat_alvo_deg;  /**< latitude do alvo (graus)             */
  float    lon_alvo_deg;  /**< longitude do alvo (graus)            */
  float    alt_alvo_m;    /**< altitude do alvo (m)                 */
  float    raio_chegada_m;/**< dentro deste raio = chegou           */
} nav_posicao_t;

void nav_posicao_inicializa(nav_posicao_t *nav, float raio_chegada_m);

/**
 * @brief  Distancia horizontal entre dois pontos (m).
 */
float nav_posicao_distancia_m(float lat1_deg, float lon1_deg,
                              float lat2_deg, float lon2_deg);

/**
 * @brief  Rumo de um ponto ao outro (graus, 0=N, 90=E, 360).
 */
float nav_posicao_rumo_deg(float lat1_deg, float lon1_deg,
                           float lat2_deg, float lon2_deg);

/**
 * @brief  Erro de "ponto2 - ponto1" em metros no plano local
 *         (Norte positivo, Leste positivo).
 */
void nav_posicao_erro_ned(float lat1_deg, float lon1_deg,
                          float lat2_deg, float lon2_deg,
                          float *norte_m, float *leste_m);

void nav_posicao_definir_alvo(nav_posicao_t *nav, float lat_deg,
                              float lon_deg, float alt_m);

void nav_posicao_limpar_alvo(nav_posicao_t *nav);

/**
 * @brief  1 se a posicao atual esta dentro do raio de chegada.
 */
uint8_t nav_posicao_atingiu(const nav_posicao_t *nav, float lat_deg,
                            float lon_deg);

#ifdef __cplusplus
}
#endif

#endif /* USR_NAVIGATION_NAV_POSICAO_H */
