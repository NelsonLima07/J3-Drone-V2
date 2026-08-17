/**
 * @file    waypoints.h
 * @brief   Rota de waypoints digitada manualmente no firmware.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Fase 1: rota por indice sequencial (ainda nao ha interface serial).
 * Preencha os pontos abaixo e atualize ROTA_WAYPOINTS_N com a
 * quantidade de pontos. Cada ponto e um struct waypoint_t
 * { latitude (graus), longitude (graus), altitude (m) }.
 *
 * Exemplo de preenchimento:
 *   #define ROTA_WAYPOINTS_N 2u
 *   const waypoint_t ROTA_WAYPOINTS[ROTA_WAYPOINTS_N] = {
 *     { -23.550520f, -46.633309f, 120.0f },
 *     { -23.550800f, -46.633500f, 120.0f },
 *   };
 */

#ifndef USR_NAVIGATION_WAYPOINTS_H
#define USR_NAVIGATION_WAYPOINTS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float lat_deg;  /**< latitude (graus, -90..+90) */
  float lon_deg;  /**< longitude (graus, -180..+180) */
  float alt_m;    /**< altitude (m) */
} waypoint_t;

/* Quantidade de pontos da rota (edite junto com o array abaixo). */
#define ROTA_WAYPOINTS_N 0u

#if ROTA_WAYPOINTS_N > 0u
const waypoint_t ROTA_WAYPOINTS[ROTA_WAYPOINTS_N] = {
  /* { lat_deg, lon_deg, alt_m }, */
};
#endif

#ifdef __cplusplus
}
#endif

#endif /* USR_NAVIGATION_WAYPOINTS_H */
