/**
 * @file    nav_posicao.c
 * @brief   Geometria de navegacao (plano local tangente) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <math.h>
#include <string.h>
#include "navigation/nav_posicao.h"

static float graus_rad(float g)
{
  return g * 3.14159265358979323846f / 180.0f;
}

void nav_posicao_inicializa(nav_posicao_t *nav, float raio_chegada_m)
{
  memset(nav, 0, sizeof(*nav));
  nav->raio_chegada_m = raio_chegada_m;
}

float nav_posicao_distancia_m(float lat1_deg, float lon1_deg,
                              float lat2_deg, float lon2_deg)
{
  float lat1 = graus_rad(lat1_deg);
  float lat2 = graus_rad(lat2_deg);
  float dlat = lat2 - lat1;
  float dlon = graus_rad(lon2_deg - lon1_deg);
  float lat_med = (lat1 + lat2) * 0.5f;
  float norte = dlat * NAV_RAIO_TERRA_M;
  float leste = dlon * cosf(lat_med) * NAV_RAIO_TERRA_M;

  return sqrtf(norte * norte + leste * leste);
}

float nav_posicao_rumo_deg(float lat1_deg, float lon1_deg,
                           float lat2_deg, float lon2_deg)
{
  float lat1 = graus_rad(lat1_deg);
  float lat2 = graus_rad(lat2_deg);
  float dlon = graus_rad(lon2_deg - lon1_deg);
  float y = sinf(dlon) * cosf(lat2);
  float x = cosf(lat1) * sinf(lat2) -
            sinf(lat1) * cosf(lat2) * cosf(dlon);
  float rumo = atan2f(y, x) * 180.0f / 3.14159265358979323846f;

  if (rumo < 0.0f)
  {
    rumo += 360.0f;
  }
  return rumo;
}

void nav_posicao_erro_ned(float lat1_deg, float lon1_deg,
                          float lat2_deg, float lon2_deg,
                          float *norte_m, float *leste_m)
{
  float lat_med = graus_rad((lat1_deg + lat2_deg) * 0.5f);

  if (norte_m != 0)
  {
    *norte_m = (lat2_deg - lat1_deg) * graus_rad(1.0f) *
               NAV_RAIO_TERRA_M;
  }
  if (leste_m != 0)
  {
    *leste_m = (lon2_deg - lon1_deg) * graus_rad(1.0f) *
               cosf(lat_med) * NAV_RAIO_TERRA_M;
  }
}

void nav_posicao_definir_alvo(nav_posicao_t *nav, float lat_deg,
                              float lon_deg, float alt_m)
{
  nav->alvo_valido = 1;
  nav->lat_alvo_deg = lat_deg;
  nav->lon_alvo_deg = lon_deg;
  nav->alt_alvo_m = alt_m;
}

void nav_posicao_limpar_alvo(nav_posicao_t *nav)
{
  nav->alvo_valido = 0;
}

uint8_t nav_posicao_atingiu(const nav_posicao_t *nav, float lat_deg,
                            float lon_deg)
{
  if (nav == 0 || !nav->alvo_valido)
  {
    return 0;
  }
  return nav_posicao_distancia_m(nav->lat_alvo_deg, nav->lon_alvo_deg,
                                 lat_deg, lon_deg) <= nav->raio_chegada_m;
}
