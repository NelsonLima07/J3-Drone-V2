/**
 * @file    estados.h
 * @brief   Maquina de estados do voo (ESPERA / VOO / CALIBRACAO) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Nao depende do HAL: recebe os canais, o sinal de sinal presente e o
 * tempo em ms. As bordas (entrou em VOO, entrou em CALIBRACAO) sao
 * detectadas pelo supervisor comparando o modo retornado.
 */

#ifndef USR_SYSTEM_ESTADOS_H
#define USR_SYSTEM_ESTADOS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ESTADO_ESPERA = 0,   /**< aguardando gesto de armar ou CH5 */
  ESTADO_VOO,          /**< controle habilitado               */
  ESTADO_CALIBRACAO    /**< coletando amostras de calibracao  */
} estado_modo_t;

typedef struct {
  uint16_t throttle_idx;    /**< indice do canal de throttle (AETR: CH3 -> 2) */
  uint16_t yaw_idx;         /**< indice do canal de yaw (CH4 -> 3)            */
  uint16_t calibracao_idx;  /**< indice do switch de calibracao (CH5 -> 4)    */
  uint16_t throttle_min;    /**< throttle abaixo deste = minimo               */
  uint16_t yaw_esquerda;    /**< yaw >= deste = esquerda (arma)               */
  uint16_t yaw_direita;     /**< yaw <= deste = direita (desarma)             */
  uint16_t calibracao_on;   /**< CH5 acima deste = acionado                   */
  uint32_t tempo_gesto_ms;  /**< debounce do gesto de armar/desarmar          */
  uint32_t tempo_falha_ms;  /**< sem sinal por este tempo -> ESPERA           */
  uint32_t tempo_switch_ms; /**< debounce do switch de calibracao             */
} config_estados_t;

typedef struct {
  estado_modo_t modo;
  uint32_t ultimo_sinal_ms;
  uint32_t gesto_inicio_ms;
  uint32_t switch_inicio_ms;
  uint8_t  gesto_hold;      /* gesto valido em curso                        */
  uint8_t  switch_hold;     /* CH5 pressionado em curso                     */
  uint8_t  cal_release;     /* exige soltar o CH5 antes de recalibrar       */
  uint8_t  controle_habilitado;
} estados_t;

/**
 * @brief  Configuracao padrao (iBus AETR, gesto 2 s, CH5 = canal 4).
 */
void config_estados_padrao(config_estados_t *cfg);

/**
 * @brief  Zera o estado e comeca em ESTADO_ESPERA.
 */
void estados_inicializa(estados_t *es, const config_estados_t *cfg);

/**
 * @brief  Avanca a maquina de estados.
 * @param  es                   estado da FSM (persistido entre chamadas)
 * @param  cfg                  configuracao
 * @param  canais               canais do iBus (IBUS_NUM_CANAIS) ou NULL
 * @param  tem_sinal            1 quando ha quadro iBus recente
 * @param  calibracao_concluida 1 quando a coleta de calibracao terminou
 * @param  armar_permitido      1 libera o armar (gate de GPS/modo);
 *                              com 0 o gesto e cancelado
 * @param  agora_ms             tempo atual (ms)
 * @return novo modo.
 */
estado_modo_t estados_atualiza(estados_t *es, const config_estados_t *cfg,
                               const uint16_t *canais, uint8_t tem_sinal,
                               uint8_t calibracao_concluida,
                               uint8_t armar_permitido, uint32_t agora_ms);

/**
 * @brief  1 quando o controle esta habilitado (modo VOO).
 */
uint8_t estados_controle_habilitado(const estados_t *es);

#ifdef __cplusplus
}
#endif

#endif /* USR_SYSTEM_ESTADOS_H */
