/**
 * @file    test_home_ponto.c
 * @brief   Testes nativos do home point (melhor candidato + trava).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "navigation/home_ponto.h"

static int falhas = 0;

#define VERIFICA(cond, nome)                                        \
  do {                                                              \
    if (cond) {                                                     \
      printf("  ok: %s\n", nome);                                   \
    } else {                                                        \
      printf("  FALHOU: %s (linha %d)\n", nome, __LINE__);          \
      ++falhas;                                                     \
    }                                                               \
  } while (0)

static gps_medida_t fix(float lat, float lon, float erro)
{
  gps_medida_t m;
  memset(&m, 0, sizeof(m));
  m.fix_valido = 1;
  m.num_sats = 10;
  m.lat_deg = lat;
  m.lon_deg = lon;
  m.alt_m = 120.0f;
  m.hdop = erro / 5.0f;
  m.erro_estimado_m = erro;
  return m;
}

int testes_home_ponto(void)
{
  config_home_t cfg;
  home_ponto_t home;
  gps_medida_t m;

  config_home_padrao(&cfg);
  home_ponto_inicializa(&home, &cfg);

  /* Sem fix ainda: home invalido. */
  m = fix(-23.55f, -46.63f, 10.0f);
  m.fix_valido = 0;
  home_ponto_atualiza(&home, &m);
  VERIFICA(!home.valido, "sem fix -> home invalido");

  /* Primeiro fix vira candidato. */
  m = fix(-23.55f, -46.63f, 10.0f);
  home_ponto_atualiza(&home, &m);
  VERIFICA(home.valido == 1u, "primeiro fix vira candidato");
  VERIFICA(home.erro_m == 10.0f, "erro do primeiro candidato");
  VERIFICA(fabsf(home.lat_deg - (-23.55f)) < 1.0e-6f, "lat do candidato");

  /* Fix pior nao substitui. */
  m = fix(-23.56f, -46.64f, 12.0f);
  home_ponto_atualiza(&home, &m);
  VERIFICA(fabsf(home.lat_deg - (-23.55f)) < 1.0e-6f,
           "fix pior nao substitui o melhor");

  /* Fix melhor substitui. */
  m = fix(-23.551f, -46.631f, 4.0f);
  home_ponto_atualiza(&home, &m);
  VERIFICA(fabsf(home.lat_deg - (-23.551f)) < 1.0e-6f, "fix melhor substitui");
  VERIFICA(home.erro_m == 4.0f, "erro atualizado para o melhor");

  /* Trava (borda de armar): mesmo com fix pior, mantem o melhor. */
  m = fix(-23.55f, -46.63f, 20.0f);
  home_ponto_trava(&home, &m);
  VERIFICA(home_ponto_valido(&home) == 1u, "travado -> home valido p/ voo");
  VERIFICA(home.travado == 1u, "campo travado ativo");
  VERIFICA(fabsf(home.lat_deg - (-23.551f)) < 1.0e-6f,
           "trava manteve o melhor candidato");

  /* Em voo (travado), fix melhor nao atualiza. */
  m = fix(-23.55f, -46.63f, 1.0f);
  home_ponto_atualiza(&home, &m);
  VERIFICA(fabsf(home.lat_deg - (-23.551f)) < 1.0e-6f,
           "travado congela o home mesmo com fix melhor");

  /* Destrava (desarme): invalida e reinicia a busca. */
  home_ponto_destrava(&home);
  VERIFICA(home_ponto_valido(&home) == 0u, "destravado -> invalido");
  VERIFICA(home.travado == 0u, "destravado -> travado 0");
  m = fix(-23.56f, -46.64f, 8.0f);
  home_ponto_atualiza(&home, &m);
  VERIFICA(home.valido == 1u, "apos desarme volta a gravar candidato");

  printf("testes_home_ponto: %d falha(s)\n", falhas);
  return falhas;
}
