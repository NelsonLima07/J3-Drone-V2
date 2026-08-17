/**
 * @file    test_estados.c
 * @brief   Testes nativos da maquina de estados do voo
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include "system/estados.h"
#include "serial/ibus.h"

static void preenche(uint16_t *canais, uint16_t throttle, uint16_t yaw, uint16_t ch5)
{
  int i;

  for (i = 0; i < IBUS_NUM_CANAIS; ++i)
  {
    canais[i] = 1500;
  }
  canais[2] = throttle;
  canais[3] = yaw;
  canais[4] = ch5;
}

int testes_estados(void)
{
  int falhas = 0;
  config_estados_t cfg;
  estados_t es;
  uint16_t canais[IBUS_NUM_CANAIS];
  estado_modo_t modo;
  uint32_t t = 0;

  config_estados_padrao(&cfg);
  estados_inicializa(&es, &cfg);

  /* ESPERA -> VOO: gesto (throttle minimo + yaw esquerda) sustentado 2 s. */
  preenche(canais, 1000, 1950, 1000);
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_ESPERA)
  {
    printf("FALHA: armou antes do tempo de gesto\n");
    falhas++;
  }
  t += cfg.tempo_gesto_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_VOO)
  {
    printf("FALHA: nao armou apos %u ms\n", (unsigned)cfg.tempo_gesto_ms);
    falhas++;
  }
  if (!estados_controle_habilitado(&es))
  {
    printf("FALHA: controle nao habilitado ao armar\n");
    falhas++;
  }

  /* Gate de armar: com armar_permitido=0 o gesto nao arma. */
  t += 1000;
  preenche(canais, 1000, 1050, 1000);  /* desarma primeiro */
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  t += cfg.tempo_gesto_ms + 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (estados_controle_habilitado(&es))
  {
    printf("FALHA: controle habilitado apos desarmar\n");
    falhas++;
  }
  preenche(canais, 1000, 1950, 1000);
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 0, t);  /* gate fechado */
  t += cfg.tempo_gesto_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 0, t);
  if (modo == ESTADO_VOO)
  {
    printf("FALHA: armou com armar_permitido=0\n");
    falhas++;
  }
  /* Libera: exige gesto novo (o hold foi cancelado). */
  t += 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo == ESTADO_VOO)
  {
    printf("FALHA: armou sem novo gesto apos liberar o gate\n");
    falhas++;
  }
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 0, 0, 1, t);  /* solta o gesto */
  preenche(canais, 1000, 1950, 1000);
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  t += cfg.tempo_gesto_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_VOO)
  {
    printf("FALHA: nao armou com gesto novo e gate liberado\n");
    falhas++;
  }

  /* Failsafe: perda de sinal -> ESPERA e controle desabilitado. */
  t += 1000;
  modo = estados_atualiza(&es, &cfg, canais, 0, 0, 1, t);
  if (modo != ESTADO_ESPERA)
  {
    printf("FALHA: failsafe nao voltou para ESPERA\n");
    falhas++;
  }
  if (estados_controle_habilitado(&es))
  {
    printf("FALHA: controle habilitado apos failsafe\n");
    falhas++;
  }

  /* Re-arma (gesto ainda sustentado) e desarma (yaw direita). */
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  t += cfg.tempo_gesto_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_VOO)
  {
    printf("FALHA: nao re-armou\n");
    falhas++;
  }
  preenche(canais, 1000, 1050, 1000);
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  t += cfg.tempo_gesto_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_ESPERA)
  {
    printf("FALHA: nao desarmou com yaw direita\n");
    falhas++;
  }
  if (estados_controle_habilitado(&es))
  {
    printf("FALHA: controle ainda habilitado apos desarmar\n");
    falhas++;
  }

  /* Entra em calibracao segurando CH5. */
  preenche(canais, 1500, 1500, 1800);
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  t += cfg.tempo_switch_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_CALIBRACAO)
  {
    printf("FALHA: nao entrou em calibracao via CH5\n");
    falhas++;
  }

  /* CH5 sustentado e calibracao concluida -> sai para ESPERA. */
  t += cfg.tempo_switch_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 1, 1, t);
  if (modo != ESTADO_ESPERA)
  {
    printf("FALHA: nao saiu da calibracao ao concluir\n");
    falhas++;
  }

  /* CH5 ainda pressionado: nao recalibra sem soltar. */
  t += cfg.tempo_switch_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo == ESTADO_CALIBRACAO)
  {
    printf("FALHA: recalibrou sem soltar o CH5\n");
    falhas++;
  }

  /* Solta e pressiona de novo -> calibra de novo. */
  preenche(canais, 1500, 1500, 1000);
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  preenche(canais, 1500, 1500, 1800);
  t += 1;
  (void)estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  t += cfg.tempo_switch_ms + 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_CALIBRACAO)
  {
    printf("FALHA: nao recalibrou apos soltar o CH5\n");
    falhas++;
  }

  /* Solta o CH5 durante a calibracao -> ESPERA. */
  preenche(canais, 1500, 1500, 1000);
  t += 1;
  modo = estados_atualiza(&es, &cfg, canais, 1, 0, 1, t);
  if (modo != ESTADO_ESPERA)
  {
    printf("FALHA: nao saiu da calibracao ao soltar CH5\n");
    falhas++;
  }

  printf("testes_estados: %d falha(s)\n", falhas);
  return falhas;
}
