/**
 * @file    test_nav_posicao.c
 * @brief   Testes nativos da geometria de navegacao (nav_posicao).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>

#include "navigation/nav_posicao.h"

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

#define APROX(a, b, tol) (fabsf((a) - (b)) <= (tol))

int testes_nav_posicao(void)
{
  nav_posicao_t nav;
  float norte, leste, dist, rumo;

  /* ---------- Distancia: 1e-3 deg de lat ~= 111,2 m ---------- */
  dist = nav_posicao_distancia_m(0.0f, 0.0f, 0.001f, 0.0f);
  VERIFICA(APROX(dist, 111.19f, 1.0f), "1e-3 deg lat ~ 111,2 m");

  /* Mesmo ponto -> 0. */
  dist = nav_posicao_distancia_m(-23.55f, -46.63f, -23.55f, -46.63f);
  VERIFICA(APROX(dist, 0.0f, 0.01f), "mesmo ponto -> 0 m");

  /* ---------- Rumo: N, E, S, W ---------- */
  rumo = nav_posicao_rumo_deg(0.0f, 0.0f, 0.001f, 0.0f);
  VERIFICA(APROX(rumo, 0.0f, 0.5f), "ao norte -> rumo 0");
  rumo = nav_posicao_rumo_deg(0.0f, 0.0f, 0.0f, 0.001f);
  VERIFICA(APROX(rumo, 90.0f, 0.5f), "ao leste -> rumo 90");
  rumo = nav_posicao_rumo_deg(0.0f, 0.0f, -0.001f, 0.0f);
  VERIFICA(APROX(rumo, 180.0f, 0.5f), "ao sul -> rumo 180");
  rumo = nav_posicao_rumo_deg(0.0f, 0.0f, 0.0f, -0.001f);
  VERIFICA(APROX(rumo, 270.0f, 0.5f), "ao oeste -> rumo 270");

  /* ---------- Erro NED consistente com a distancia ---------- */
  nav_posicao_erro_ned(0.0f, 0.0f, 0.001f, 0.001f, &norte, &leste);
  VERIFICA(APROX(norte, 111.19f, 1.0f), "erro norte ~111,2 m");
  VERIFICA(APROX(leste, 111.19f, 1.0f), "erro leste ~111,2 m");

  /* ---------- Alvo e raio de chegada ---------- */
  nav_posicao_inicializa(&nav, 1.5f);
  nav_posicao_definir_alvo(&nav, -23.55f, -46.63f, 100.0f);
  VERIFICA(nav.alvo_valido == 1u, "alvo definido");
  VERIFICA(nav_posicao_atingiu(&nav, -23.55f, -46.63f) == 1u,
           "no alvo -> chegou");
  VERIFICA(nav_posicao_atingiu(&nav, -23.550009f, -46.63f) == 1u,
           "1 m do alvo -> dentro do raio");
  VERIFICA(nav_posicao_atingiu(&nav, -23.55f, -46.633f) == 0u,
           "~300 m do alvo -> fora do raio");
  nav_posicao_limpar_alvo(&nav);
  VERIFICA(nav_posicao_atingiu(&nav, -23.55f, -46.63f) == 0u,
           "sem alvo -> nunca chegou");

  printf("testes_nav_posicao: %d falha(s)\n", falhas);
  return falhas;
}
