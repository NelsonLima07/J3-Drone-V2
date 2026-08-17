/**
 * @file    test_gps_nmea.c
 * @brief   Testes nativos do parser NMEA (GGA/RMC) do GPS BN-220.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "serial/gps_nmea.h"

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

/* Sentenca de exemplo (checksums publicados na especificacao NMEA). */
static const char GGA_OK[] =
  "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static const char RMC_OK[] =
  "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

/* Monta "$corpo*HH\r\n" com o checksum calculado (para sentencas de teste). */
static void constroi(char *saida, size_t tam, const char *corpo)
{
  unsigned char x = 0;
  size_t n = strlen(corpo);
  size_t i;

  for (i = 0; i < n; ++i)
  {
    x ^= (unsigned char)corpo[i];
  }
  snprintf(saida, tam, "$%s*%02X\r\n", corpo, (unsigned)x);
}

int testes_gps_nmea(void)
{
  config_gps_t cfg;
  gps_parser_t p;
  gps_medida_t m;
  char sentenca[128];
  const uint8_t *b;
  uint32_t n, i;

  gps_config_padrao(&cfg);

  /* ---------- GGA byte a byte (sentenca real de exemplo) ---------- */
  {
    gps_nmea_inicializa(&p, &cfg);
    b = (const uint8_t *)GGA_OK;
    n = (uint32_t)strlen(GGA_OK);
    for (i = 0; i < n; ++i)
    {
      (void)gps_nmea_processa_byte(&p, b[i]);
    }
    m = gps_nmea_medida(&p);
    VERIFICA(m.versao >= 1u, "GGA atualizou a medida");
    VERIFICA(m.fix_valido == 1u, "GGA qualidade 1 e 8 sats -> fix valido");
    VERIFICA(m.num_sats == 8u, "GGA num_sats = 8");
    VERIFICA(APROX(m.hdop, 0.9f, 0.01f), "GGA hdop = 0.9");
    VERIFICA(APROX(m.erro_estimado_m, 4.5f, 0.05f), "erro = hdop * uere (5 m)");
    VERIFICA(APROX(m.alt_m, 545.4f, 0.1f), "GGA altitude = 545.4 m");
  }

  /* ---------- RMC: lat/lon, velocidade e rumo ---------- */
  {
    gps_nmea_inicializa(&p, &cfg);
    b = (const uint8_t *)RMC_OK;
    n = (uint32_t)strlen(RMC_OK);
    (void)gps_nmea_processa_buffer(&p, b, n);
    m = gps_nmea_medida(&p);
    VERIFICA(APROX(m.lat_deg, 48.1173f, 0.0002f), "RMC latitude 48.1173");
    VERIFICA(APROX(m.lon_deg, 11.5167f, 0.0002f), "RMC longitude 11.5167");
    VERIFICA(APROX(m.vel_m_s, 11.524f, 0.1f), "RMC 22.4 kn -> 11.52 m/s");
    VERIFICA(APROX(m.curso_deg, 84.4f, 0.1f), "RMC curso 84.4 deg");
  }

  /* ---------- Hemisferio Sul/Oeste (sinais) ---------- */
  {
    constroi(sentenca, sizeof(sentenca),
             "GPRMC,123519,A,4807.038,S,01131.000,W,000.0,000.0,230394,003.1,E");
    gps_nmea_inicializa(&p, &cfg);
    b = (const uint8_t *)sentenca;
    n = (uint32_t)strlen(sentenca);
    (void)gps_nmea_processa_buffer(&p, b, n);
    m = gps_nmea_medida(&p);
    VERIFICA(APROX(m.lat_deg, -48.1173f, 0.0002f), "S -> latitude negativa");
    VERIFICA(APROX(m.lon_deg, -11.5167f, 0.0002f), "W -> longitude negativa");
  }

  /* ---------- GGA sem fix (qualidade 0) ---------- */
  {
    constroi(sentenca, sizeof(sentenca),
             "GPGGA,123519,4807.038,N,01131.000,E,0,00,99.9,545.4,M,46.9,M,,");
    gps_nmea_inicializa(&p, &cfg);
    b = (const uint8_t *)sentenca;
    n = (uint32_t)strlen(sentenca);
    (void)gps_nmea_processa_buffer(&p, b, n);
    m = gps_nmea_medida(&p);
    VERIFICA(m.fix_valido == 0u, "qualidade 0 -> sem fix");
    VERIFICA(m.num_sats == 0u, "qualidade 0 -> zero sats");
  }

  /* ---------- Checksum errado e ignorado ---------- */
  {
    gps_parser_t p2;
    gps_medida_t m0;
    static const char GGA_ERRADO[] =
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n";
    gps_nmea_inicializa(&p, &cfg);
    gps_nmea_inicializa(&p2, &cfg);
    b = (const uint8_t *)GGA_ERRADO;
    n = (uint32_t)strlen(GGA_ERRADO);
    (void)gps_nmea_processa_buffer(&p, b, n);
    m = gps_nmea_medida(&p);
    m0 = gps_nmea_medida(&p2);
    VERIFICA(m.versao == m0.versao, "checksum errado nao atualiza medida");
  }

  /* ---------- Sentenca fatiada em pedacos pequenos ---------- */
  {
    gps_nmea_inicializa(&p, &cfg);
    b = (const uint8_t *)RMC_OK;
    n = (uint32_t)strlen(RMC_OK);
    for (i = 0; i < n; i += 3)
    {
      uint32_t resto = n - i;
      if (resto > 3)
      {
        resto = 3;
      }
      (void)gps_nmea_processa_buffer(&p, &b[i], resto);
    }
    m = gps_nmea_medida(&p);
    VERIFICA(APROX(m.vel_m_s, 11.524f, 0.1f),
             "RMC fatiado em chunks de 3 bytes decodifica igual");
  }

  /* ---------- UERE configurado muda o erro estimado ---------- */
  {
    config_gps_t cfg2 = cfg;
    cfg2.uere = 8.0f;
    gps_nmea_inicializa(&p, &cfg2);
    b = (const uint8_t *)GGA_OK;
    n = (uint32_t)strlen(GGA_OK);
    (void)gps_nmea_processa_buffer(&p, b, n);
    m = gps_nmea_medida(&p);
    VERIFICA(APROX(m.erro_estimado_m, 7.2f, 0.05f), "uere 8 m -> erro 7.2 m");
  }

  printf("testes_gps_nmea: %d falha(s)\n", falhas);
  return falhas;
}
