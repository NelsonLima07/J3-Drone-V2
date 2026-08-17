/**
 * @file    home_ponto.c
 * @brief   Ponto de origem (home) para RTH em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <string.h>
#include "navigation/home_ponto.h"

void config_home_padrao(config_home_t *cfg)
{
  cfg->erro_max_m = 15.0f;
}

void home_ponto_inicializa(home_ponto_t *home, const config_home_t *cfg)
{
  (void)cfg;
  memset(home, 0, sizeof(*home));
}

void home_ponto_atualiza(home_ponto_t *home, const gps_medida_t *gps)
{
  if (home == 0 || gps == 0)
  {
    return;
  }
  /* Travado em voo: nada muda. */
  if (home->travado)
  {
    return;
  }
  if (!gps->fix_valido || gps->erro_estimado_m <= 0.0f)
  {
    return;
  }
  if (!home->valido || gps->erro_estimado_m < home->erro_m)
  {
    home->valido = 1;
    home->lat_deg = gps->lat_deg;
    home->lon_deg = gps->lon_deg;
    home->alt_m = gps->alt_m;
    home->erro_m = gps->erro_estimado_m;
  }
}

void home_ponto_trava(home_ponto_t *home, const gps_medida_t *gps)
{
  if (home == 0 || gps == 0)
  {
    return;
  }
  /* Ultima chance: se a medida atual for melhor, usa-a como decolagem. */
  if (gps->fix_valido && gps->erro_estimado_m > 0.0f)
  {
    if (!home->valido || gps->erro_estimado_m < home->erro_m)
    {
      home->valido = 1;
      home->lat_deg = gps->lat_deg;
      home->lon_deg = gps->lon_deg;
      home->alt_m = gps->alt_m;
      home->erro_m = gps->erro_estimado_m;
    }
  }
  home->travado = 1;
}

void home_ponto_destrava(home_ponto_t *home)
{
  home->travado = 0;
  home->valido = 0;
}

uint8_t home_ponto_valido(const home_ponto_t *home)
{
  return (home != 0) ? (uint8_t)(home->valido && home->travado) : 0u;
}
