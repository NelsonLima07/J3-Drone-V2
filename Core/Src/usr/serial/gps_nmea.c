/**
 * @file    gps_nmea.c
 * @brief   Decodificador NMEA (GGA/RMC) em C puro para o GPS BN-220.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Maquina de estados simples por byte: procura '$', acumula ate '*',
 * le 2 hex de checksum e valida ao fim de linha. Suporta GGA e RMC
 * com qualquer talker (GP/GN). Coordenadas NMEA (ddmm.mmmm) sao
 * convertidas para graus decimais.
 */

#include <string.h>
#include <stdlib.h>
#include "serial/gps_nmea.h"

/* Estados internos do parser. */
enum {
  GPS_EST_PROCURA = 0u,
  GPS_EST_LINHA,
  GPS_EST_CHECKSUM
};

/* Indices de campo (0 = tipo da sentenca). */
#define GPS_CAMPO_TIPO      0
#define GPS_GGA_QUALIDADE   6
#define GPS_GGA_SATS        7
#define GPS_GGA_HDOP        8
#define GPS_GGA_ALT         9
#define GPS_RMC_STATUS      2
#define GPS_RMC_LAT         3
#define GPS_RMC_LAT_S       4
#define GPS_RMC_LON         5
#define GPS_RMC_LON_S       6
#define GPS_RMC_VEL         7
#define GPS_RMC_CURSO       8

#define GPS_KNOTS_M_S       0.5144444f

void gps_config_padrao(config_gps_t *cfg)
{
  cfg->uere = 5.0f;
}

void gps_nmea_inicializa(gps_parser_t *p, const config_gps_t *cfg)
{
  memset(p, 0, sizeof(*p));
  if (cfg != 0)
  {
    p->config = *cfg;
  }
  else
  {
    gps_config_padrao(&p->config);
  }
}

gps_medida_t gps_nmea_medida(const gps_parser_t *p)
{
  return p->medida;
}

/* Extrai o inicio do campo 'indice' da linha (separadores ',').
 * Retorna 0 se o campo nao existir. */
static const uint8_t *nmea_campo(const uint8_t *linha, int indice)
{
  const uint8_t *c = linha;
  int atual = 0;

  if (c == 0 || *c == 0)
  {
    return 0;
  }
  while (atual < indice)
  {
    while (*c != ',' && *c != 0)
    {
      c++;
    }
    if (*c == 0)
    {
      return 0;
    }
    c++;
    atual++;
  }
  return c;
}

static int hex_val(uint8_t c)
{
  if (c >= '0' && c <= '9')
  {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F')
  {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f')
  {
    return c - 'a' + 10;
  }
  return -1;
}

static uint8_t nmea_parse_float(const uint8_t *s, float *saida)
{
  char buf[16];
  uint8_t n = 0;

  if (s == 0)
  {
    return 0;
  }
  while (*s != ',' && *s != 0 && n < sizeof(buf) - 1)
  {
    buf[n++] = (char)*s;
    s++;
  }
  buf[n] = 0;
  if (n == 0)
  {
    return 0;
  }
  *saida = (float)strtod(buf, 0);
  return 1;
}

static uint8_t nmea_parse_int(const uint8_t *s, int *saida)
{
  char buf[8];
  uint8_t n = 0;

  if (s == 0)
  {
    return 0;
  }
  while (*s != ',' && *s != 0 && n < sizeof(buf) - 1)
  {
    buf[n++] = (char)*s;
    s++;
  }
  buf[n] = 0;
  if (n == 0)
  {
    return 0;
  }
  *saida = atoi(buf);
  return 1;
}

/* Conversao NMEA ddmm.mmmm -> graus decimais (com sinal N/S, E/W). */
static float nmea_latlon(float valor, uint8_t hemisferio_negativo)
{
  int graus = (int)(valor / 100.0f);
  float minutos = valor - (float)graus * 100.0f;
  float dec = (float)graus + minutos / 60.0f;

  if (hemisferio_negativo)
  {
    dec = -dec;
  }
  return dec;
}

static uint8_t nmea_eh_gga(const uint8_t *tipo)
{
  return tipo[0] == 'G' && tipo[1] == 'G' && tipo[2] == 'A';
}

static uint8_t nmea_eh_rmc(const uint8_t *tipo)
{
  return tipo[0] == 'R' && tipo[1] == 'M' && tipo[2] == 'C';
}

static void gps_parser_aplica(gps_parser_t *p, const uint8_t *linha)
{
  const uint8_t *tipo;
  const uint8_t *campo;
  float v;
  int iv;

  /* O parser consome o '$'; 'linha' comeca no talker ("GPGGA,...").
   * Talker GPS tem 2 letras (GP/GN) -> tipo em linha+2.
   * Exige corpo minimo "Gxxxx," para ler o tipo sem estouro. */
  if (linha == 0 || p->linha_ix < 6u)
  {
    return;
  }
  tipo = linha + 2;

  if (nmea_eh_gga(tipo))
  {
    if (nmea_parse_int(nmea_campo(linha, GPS_GGA_SATS), &iv))
    {
      p->medida.num_sats = (uint8_t)iv;
    }
    if (nmea_parse_float(nmea_campo(linha, GPS_GGA_QUALIDADE), &v))
    {
      p->medida.qualidade = v;
      p->medida.fix_valido = (uint8_t)(v >= 1.0f && p->medida.num_sats >= 3u);
    }
    if (nmea_parse_float(nmea_campo(linha, GPS_GGA_HDOP), &v))
    {
      p->medida.hdop = v;
      p->medida.erro_estimado_m = v * p->config.uere;
    }
    if (nmea_parse_float(nmea_campo(linha, GPS_GGA_ALT), &v))
    {
      p->medida.alt_m = v;
    }
  }
  else if (nmea_eh_rmc(tipo))
  {
    campo = nmea_campo(linha, GPS_RMC_STATUS);
    if (campo != 0 && campo[0] == 'A')
    {
      if (nmea_parse_float(nmea_campo(linha, GPS_RMC_LAT), &v))
      {
        campo = nmea_campo(linha, GPS_RMC_LAT_S);
        p->medida.lat_deg = nmea_latlon(v, campo != 0 && campo[0] == 'S');
      }
      if (nmea_parse_float(nmea_campo(linha, GPS_RMC_LON), &v))
      {
        campo = nmea_campo(linha, GPS_RMC_LON_S);
        p->medida.lon_deg = nmea_latlon(v, campo != 0 && campo[0] == 'W');
      }
      if (nmea_parse_float(nmea_campo(linha, GPS_RMC_VEL), &v))
      {
        p->medida.vel_m_s = v * GPS_KNOTS_M_S;
      }
      if (nmea_parse_float(nmea_campo(linha, GPS_RMC_CURSO), &v))
      {
        p->medida.curso_deg = v;
      }
    }
  }
  else
  {
    return;
  }
  p->medida.versao++;
}

uint8_t gps_nmea_processa_byte(gps_parser_t *p, uint8_t byte)
{
  uint8_t terminou = 0;

  switch (p->estado)
  {
    case GPS_EST_PROCURA:
      if (byte == '$')
      {
        p->estado = GPS_EST_LINHA;
        p->linha_ix = 0;
        p->checksum_calc = 0;
      }
      break;

    case GPS_EST_LINHA:
      if (byte == '$')
      {
        p->linha_ix = 0;
        p->checksum_calc = 0;
      }
      else if (byte == '*')
      {
        p->estado = GPS_EST_CHECKSUM;
        p->checksum_recebido = 0;
        p->checksum_ix = 0;
      }
      else if (byte == '\r' || byte == '\n')
      {
        p->estado = GPS_EST_PROCURA;
      }
      else
      {
        if (p->linha_ix < GPS_NMEA_LINHA_MAX - 1)
        {
          p->linha[p->linha_ix++] = byte;
        }
        p->checksum_calc ^= byte;
      }
      break;

    case GPS_EST_CHECKSUM:
      {
        int v = hex_val(byte);
        if (v >= 0)
        {
          p->checksum_recebido = (uint8_t)(p->checksum_recebido * 16 + v);
          p->checksum_ix++;
          if (p->checksum_ix >= 2)
          {
            if (p->checksum_recebido == p->checksum_calc)
            {
              p->linha[p->linha_ix] = 0;
              gps_parser_aplica(p, p->linha);
              terminou = 1;
            }
            p->estado = GPS_EST_PROCURA;
          }
        }
        else if (byte == '\r' || byte == '\n')
        {
          p->estado = GPS_EST_PROCURA;
        }
        else
        {
          p->estado = GPS_EST_PROCURA;
        }
      }
      break;

    default:
      p->estado = GPS_EST_PROCURA;
      break;
  }
  return terminou;
}

uint32_t gps_nmea_processa_buffer(gps_parser_t *p, const uint8_t *dados,
                                 uint32_t n)
{
  uint32_t i;
  uint32_t contagem = 0;

  for (i = 0; i < n; ++i)
  {
    contagem += gps_nmea_processa_byte(p, dados[i]);
  }
  return contagem;
}
