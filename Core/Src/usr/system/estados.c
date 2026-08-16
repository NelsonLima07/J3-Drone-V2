/**
 * @file    estados.c
 * @brief   Maquina de estados do voo em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "system/estados.h"

void config_estados_padrao(config_estados_t *cfg)
{
  if (cfg == 0)
  {
    return;
  }
  cfg->throttle_idx    = 2;   /* CH3 (AETR) */
  cfg->yaw_idx         = 3;   /* CH4        */
  cfg->calibracao_idx  = 4;   /* CH5        */
  cfg->throttle_min    = 1100;
  cfg->yaw_esquerda    = 1900;
  cfg->yaw_direita     = 1100;
  cfg->calibracao_on   = 1700;
  cfg->tempo_gesto_ms  = 2000;
  cfg->tempo_falha_ms  = 200;
  cfg->tempo_switch_ms = 100;
}

void estados_inicializa(estados_t *es, const config_estados_t *cfg)
{
  (void)cfg;
  if (es == 0)
  {
    return;
  }
  es->modo = ESTADO_ESPERA;
  es->ultimo_sinal_ms = 0;
  es->gesto_inicio_ms = 0;
  es->switch_inicio_ms = 0;
  es->gesto_hold = 0;
  es->switch_hold = 0;
  es->cal_release = 0;
  es->controle_habilitado = 0;
}

estado_modo_t estados_atualiza(estados_t *es, const config_estados_t *cfg,
                               const uint16_t *canais, uint8_t tem_sinal,
                               uint8_t calibracao_concluida, uint32_t agora_ms)
{
  uint8_t cal_acionado;

  if (es == 0 || cfg == 0)
  {
    return (es != 0) ? es->modo : ESTADO_ESPERA;
  }

  if (tem_sinal)
  {
    es->ultimo_sinal_ms = agora_ms;
  }

  /* Failsafe: sem sinal ha tempo demais -> ESPERA. */
  if ((uint32_t)(agora_ms - es->ultimo_sinal_ms) >= cfg->tempo_falha_ms)
  {
    es->modo = ESTADO_ESPERA;
    es->controle_habilitado = 0;
    es->gesto_hold = 0;
    es->switch_hold = 0;
    es->cal_release = 1;
    return es->modo;
  }

  if (canais == 0)
  {
    return es->modo;
  }

  /* Estado do switch de calibracao (CH5). */
  cal_acionado = canais[cfg->calibracao_idx] >= cfg->calibracao_on;
  if (cal_acionado)
  {
    if (!es->switch_hold)
    {
      es->switch_inicio_ms = agora_ms;
    }
    es->switch_hold = 1;
  }
  else
  {
    es->switch_hold = 0;
    es->cal_release = 0;
  }

  /* Saindo da calibracao: CH5 solto ou coleta concluida. */
  if (es->modo == ESTADO_CALIBRACAO)
  {
    if (!es->switch_hold || calibracao_concluida)
    {
      es->modo = ESTADO_ESPERA;
      es->switch_hold = 0;
      es->cal_release = 1;   /* exige soltar o CH5 para recalibrar */
    }
    return es->modo;
  }

  /* Entrando na calibracao via CH5 (com debounce). */
  if (!es->cal_release && es->switch_hold &&
      (uint32_t)(agora_ms - es->switch_inicio_ms) >= cfg->tempo_switch_ms)
  {
    es->modo = ESTADO_CALIBRACAO;
    es->controle_habilitado = 0;
    es->gesto_hold = 0;
    return es->modo;
  }

  /* Gesto de armar/desarmar (throttle no minimo + yaw extremo). */
  if (es->modo == ESTADO_ESPERA)
  {
    if (canais[cfg->throttle_idx] <= cfg->throttle_min &&
        canais[cfg->yaw_idx] >= cfg->yaw_esquerda)
    {
      if (!es->gesto_hold)
      {
        es->gesto_inicio_ms = agora_ms;
      }
      es->gesto_hold = 1;
      if ((uint32_t)(agora_ms - es->gesto_inicio_ms) >= cfg->tempo_gesto_ms)
      {
        es->modo = ESTADO_VOO;
        es->controle_habilitado = 1;
        es->gesto_hold = 0;
      }
    }
    else
    {
      es->gesto_hold = 0;
    }
  }
  else /* ESTADO_VOO */
  {
    if (canais[cfg->throttle_idx] <= cfg->throttle_min &&
        canais[cfg->yaw_idx] <= cfg->yaw_direita)
    {
      if (!es->gesto_hold)
      {
        es->gesto_inicio_ms = agora_ms;
      }
      es->gesto_hold = 1;
      if ((uint32_t)(agora_ms - es->gesto_inicio_ms) >= cfg->tempo_gesto_ms)
      {
        es->modo = ESTADO_ESPERA;
        es->controle_habilitado = 0;
        es->gesto_hold = 0;
      }
    }
    else
    {
      es->gesto_hold = 0;
    }
  }

  return es->modo;
}

uint8_t estados_controle_habilitado(const estados_t *es)
{
  return (es != 0) ? es->controle_habilitado : 0;
}
