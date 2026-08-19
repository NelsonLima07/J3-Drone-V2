/**
 * @file    j3serial.c
 * @brief   Implementacao do protocolo j3Serial v0.1 em C puro.
 * @date    2026-08-18
 * @author  Nelson Lima
 * @ai      opencode (deepseek-v4-flash-free)
 *
 * Ver docs/j3Serial_protocolo.md para a especificacao completa.
 */

#include "serial/j3serial.h"

#include <string.h>

/* --- CRC8 (poli 0x31, init 0x00, MSB-first, sem xorout) -------------------- */

/* Passo incremental do CRC8 (usado pelo parser e pelo calculo em buffer). */
static uint8_t crc8_passo(uint8_t crc, uint8_t byte)
{
  int b;

  crc ^= byte;
  for (b = 0; b < 8; ++b)
  {
    if (crc & 0x80u)
    {
      crc = (uint8_t)((crc << 1) ^ 0x31u);
    }
    else
    {
      crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

uint8_t j3serial_crc8(const uint8_t *dados, uint32_t n)
{
  uint8_t crc = 0x00;
  uint32_t i;

  for (i = 0; i < n; ++i)
  {
    crc = crc8_passo(crc, dados[i]);
  }
  return crc;
}

/* --- Frame ----------------------------------------------------------------- */

uint32_t j3serial_monta_frame(uint8_t tipo, const uint8_t *payload,
                              uint8_t len, uint8_t *saida)
{
  uint8_t crc;

  if (saida == 0 || len > J3SERIAL_PAYLOAD_MAX)
  {
    return 0;
  }
  if (len > 0 && payload == 0)
  {
    return 0;
  }

  saida[0] = J3SERIAL_MAGIC;
  saida[1] = len;
  saida[2] = tipo;
  if (len > 0)
  {
    memcpy(&saida[3], payload, len);
  }
  crc = j3serial_crc8(&saida[2], 1u + (uint32_t)len);
  saida[3u + len] = crc;

  return 4u + (uint32_t)len;
}

/* --- Parser ---------------------------------------------------------------- */

void j3serial_parser_reseta(j3serial_parser_t *p)
{
  if (p != 0)
  {
    p->estado = 0;
    p->len = 0;
    p->tipo = 0;
    p->crc_calc = 0;
    p->payload_ix = 0;
  }
}

uint8_t j3serial_parser_byte(j3serial_parser_t *p, uint8_t byte)
{
  if (p == 0)
  {
    return 0;
  }

  switch (p->estado)
  {
    case 0: /* MAGIC */
      if (byte == J3SERIAL_MAGIC)
      {
        p->estado = 1;
      }
      break;

    case 1: /* LEN */
      p->len = byte;
      if (p->len > J3SERIAL_PAYLOAD_MAX)
      {
        p->estado = 0;
        break;
      }
      p->payload_ix = 0;
      p->crc_calc = 0;
      p->estado = 2;
      break;

    case 2: /* TIPO */
      p->tipo = byte;
      p->crc_calc = crc8_passo(0, byte);
      p->estado = (p->len == 0) ? 4 : 3;
      break;

    case 3: /* PAYLOAD */
      p->payload[p->payload_ix++] = byte;
      p->crc_calc = crc8_passo(p->crc_calc, byte);
      if (p->payload_ix >= p->len)
      {
        p->estado = 4;
      }
      break;

    case 4: /* CRC */
      p->estado = 0;
      if (byte == p->crc_calc)
      {
        return 1;
      }
      break;

    default:
      p->estado = 0;
      break;
  }
  return 0;
}

/* --- Helpers de payload ---------------------------------------------------- */

uint8_t *j3serial_put_u8(uint8_t *b, uint8_t v)
{
  b[0] = v;
  return b + 1;
}

uint8_t *j3serial_put_u16(uint8_t *b, uint16_t v)
{
  b[0] = (uint8_t)(v & 0xFFu);
  b[1] = (uint8_t)(v >> 8);
  return b + 2;
}

uint8_t *j3serial_put_f32(uint8_t *b, float v)
{
  memcpy(b, &v, 4);
  return b + 4;
}

uint16_t j3serial_get_u16(const uint8_t *b)
{
  return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

float j3serial_get_f32(const uint8_t *b)
{
  float v;
  memcpy(&v, b, 4);
  return v;
}

/* --- Rota ------------------------------------------------------------------ */

void j3serial_rota_limpa(j3serial_rota_t *rota)
{
  if (rota != 0)
  {
    rota->n = 0;
  }
}

uint8_t j3serial_rota_carrega(j3serial_rota_t *rota,
                              const uint8_t *payload, uint8_t len,
                              uint8_t *n_pontos)
{
  uint8_t n;
  uint8_t i;

  if (rota == 0 || payload == 0)
  {
    return J3S_ACK_ARGUMENTO_INVALIDO;
  }
  if (len < 1)
  {
    return J3S_ACK_ARGUMENTO_INVALIDO;
  }

  n = payload[0];
  if (n == 0 || (uint16_t)len != 1u + (uint16_t)n * J3SERIAL_WP_TAMANHO)
  {
    return J3S_ACK_ARGUMENTO_INVALIDO;
  }
  if (n > J3SERIAL_ROTA_MAX)
  {
    return J3S_ACK_ROTA_CHEIA;
  }

  /* Valida tudo antes de comitar (upload atomico). */
  for (i = 0; i < n; ++i)
  {
    const uint8_t *wp = &payload[1u + (uint16_t)i * J3SERIAL_WP_TAMANHO];
    float lat = j3serial_get_f32(wp);
    float lon = j3serial_get_f32(wp + 4);
    if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f)
    {
      return J3S_ACK_ARGUMENTO_INVALIDO;
    }
  }

  rota->n = n;
  for (i = 0; i < n; ++i)
  {
    const uint8_t *wp = &payload[1u + (uint16_t)i * J3SERIAL_WP_TAMANHO];
    rota->pontos[i].lat_deg = j3serial_get_f32(wp);
    rota->pontos[i].lon_deg = j3serial_get_f32(wp + 4);
    rota->pontos[i].alt_m = j3serial_get_f32(wp + 8);
  }
  if (n_pontos != 0)
  {
    *n_pontos = n;
  }
  return J3S_ACK_OK;
}

/* --- Missao ---------------------------------------------------------------- */

void j3serial_missao_inicia(j3serial_missao_t *m, uint8_t total)
{
  if (m != 0)
  {
    m->ativa = 1;
    m->indice = 0;
    m->total = total;
  }
}

void j3serial_missao_pausa(j3serial_missao_t *m)
{
  if (m != 0)
  {
    m->ativa = 0;
  }
}

uint8_t j3serial_missao_avanca(j3serial_missao_t *m)
{
  if (m == 0 || !m->ativa)
  {
    return 0;
  }
  if (m->indice + 1u >= m->total)
  {
    m->ativa = 0;
    return 0;
  }
  m->indice++;
  return 1;
}

/* --- Teste de motor -------------------------------------------------------- */

void j3serial_teste_motor_inicializa(j3serial_teste_motor_t *t)
{
  if (t != 0)
  {
    memset(t, 0, sizeof(*t));
  }
}

uint8_t j3serial_teste_motor_inicia(j3serial_teste_motor_t *t,
                                    uint8_t motor, uint8_t pico_pct,
                                    uint16_t subida_ms, uint16_t pico_ms,
                                    uint16_t descida_ms, uint32_t agora_ms)
{
  if (t == 0)
  {
    return J3S_ACK_ARGUMENTO_INVALIDO;
  }
  if (t->ativo)
  {
    return J3S_ACK_MOTOR_OCUPADO;
  }
  if (motor < 1 || motor > 4 || pico_pct < 1 || pico_pct > 100 ||
      subida_ms < 100 || descida_ms < 100)
  {
    return J3S_ACK_ARGUMENTO_INVALIDO;
  }

  t->ativo = 1;
  t->motor = motor;
  t->fase = 0;
  t->pico_pct = pico_pct;
  t->subida_ms = subida_ms;
  t->pico_ms = pico_ms;
  t->descida_ms = descida_ms;
  t->fase_inicio_ms = agora_ms;
  t->percentual_mil = 0;
  t->fim_pendente = 0;
  return J3S_ACK_OK;
}

void j3serial_teste_motor_aborta(j3serial_teste_motor_t *t)
{
  if (t != 0 && t->ativo)
  {
    t->percentual_mil = 0;
    t->ativo = 0;
    t->fim_pendente = 2;
  }
}

void j3serial_teste_motor_atualiza(j3serial_teste_motor_t *t, uint32_t agora_ms)
{
  uint32_t decorrido;
  uint32_t fase_duracao;
  uint32_t pct_alvo;

  if (t == 0 || !t->ativo)
  {
    return;
  }

  decorrido = agora_ms - t->fase_inicio_ms;

  switch (t->fase)
  {
    case 0: /* SUBIDA: 0 -> pico em subida_ms */
      fase_duracao = t->subida_ms;
      pct_alvo = (uint32_t)t->pico_pct * 100u;
      if (decorrido >= fase_duracao || fase_duracao == 0)
      {
        t->percentual_mil = (uint16_t)pct_alvo;
        t->fase = 1;
        t->fase_inicio_ms = agora_ms;
      }
      else
      {
        t->percentual_mil = (uint16_t)(pct_alvo * decorrido / fase_duracao);
      }
      break;

    case 1: /* PICO: mantem por pico_ms */
      t->percentual_mil = (uint16_t)((uint32_t)t->pico_pct * 100u);
      if (decorrido >= t->pico_ms)
      {
        t->fase = 2;
        t->fase_inicio_ms = agora_ms;
      }
      break;

    case 2: /* DESCIDA: pico -> 0 em descida_ms */
      fase_duracao = t->descida_ms;
      pct_alvo = (uint32_t)t->pico_pct * 100u;
      if (decorrido >= fase_duracao || fase_duracao == 0)
      {
        t->percentual_mil = 0;
        t->ativo = 0;
        t->fim_pendente = 1;
      }
      else
      {
        t->percentual_mil = (uint16_t)(pct_alvo *
                                       (fase_duracao - decorrido) / fase_duracao);
      }
      break;

    default:
      t->fase = 0;
      break;
  }
}

uint8_t j3serial_teste_motor_ativa(const j3serial_teste_motor_t *t)
{
  return (t != 0 && t->ativo) ? 1u : 0u;
}

uint16_t j3serial_teste_motor_percentual_mil(const j3serial_teste_motor_t *t)
{
  return (t != 0) ? t->percentual_mil : 0u;
}

uint8_t j3serial_teste_motor_motor(const j3serial_teste_motor_t *t)
{
  return (t != 0) ? t->motor : 0u;
}