/**
 * @file    j3serial.h
 * @brief   Protocolo j3Serial v0.1 (waypoints, telemetria, teste de motor)
 *          em C puro para a USART1. Ver docs/j3Serial_protocolo.md.
 * @date    2026-08-18
 * @author  Nelson Lima
 * @ai      opencode (deepseek-v4-flash-free)
 *
 * Nao depende do HAL: parser de frame, CRC8, rota em RAM, missao e a
 * FSM do teste de motor sao testaveis no host (Core/Src/usr/tests).
 * O transporte (USART1) fica no j3serial_uart_hal.
 *
 * Frame: 0xAA | LEN | TIPO | PAYLOAD[LEN] | CRC8
 * CRC8 sobre TIPO+PAYLOAD (poli 0x31, init 0x00, MSB-first).
 */

#ifndef USR_SERIAL_J3SERIAL_H
#define USR_SERIAL_J3SERIAL_H

#include <stdint.h>
#include "navigation/waypoints.h"

#ifdef __cplusplus
extern "C" {
#endif

#define J3SERIAL_MAGIC          0xAAu
#define J3SERIAL_ROTA_MAX       16u  /* limitado pelo frame (16*12+1 <= 200) */
#define J3SERIAL_PAYLOAD_MAX    200u
#define J3SERIAL_FRAME_MAX      (3u + J3SERIAL_PAYLOAD_MAX + 1u) /* 204 */
#define J3SERIAL_WP_TAMANHO     12u

/* --- Tipos de mensagem (docs/j3Serial_protocolo.md secoes 4 e 5) ---------- */

enum {
  J3S_CMD_UPLOAD_ROTA          = 0x01,
  J3S_CMD_LIMPAR_ROTA          = 0x02,
  J3S_CMD_INICIAR_MISSAO       = 0x03,
  J3S_CMD_PAUSAR_MISSAO        = 0x04,
  J3S_CMD_CONSULTAR_ROTA       = 0x05,
  J3S_CMD_CONFIG_TELEMETRIA    = 0x06,
  J3S_CMD_PEDIR_TELEMETRIA     = 0x08,
  J3S_CMD_TESTE_MOTOR          = 0x09,
  J3S_CMD_PARAR_TESTE          = 0x0A,
  J3S_CMD_PEDIR_IMU            = 0x0B,
  J3S_CMD_PEDIR_MAG            = 0x0C,
  J3S_CMD_PEDIR_BARO           = 0x0D,
  J3S_CMD_PEDIR_GPS            = 0x0E,
  J3S_CMD_PEDIR_ATITUDE        = 0x0F,
  J3S_CMD_PEDIR_ESTADO         = 0x10,

  J3S_MSG_ACK                  = 0x81,
  J3S_MSG_ROTA_RESPOSTA        = 0x85,
  J3S_MSG_TELEMETRIA           = 0x86,
  J3S_MSG_EVENTO               = 0x87,
  J3S_MSG_RESPOSTA_IMU         = 0x8B,
  J3S_MSG_RESPOSTA_MAG         = 0x8C,
  J3S_MSG_RESPOSTA_BARO        = 0x8D,
  J3S_MSG_RESPOSTA_GPS         = 0x8E,
  J3S_MSG_RESPOSTA_ATITUDE     = 0x8F,
  J3S_MSG_RESPOSTA_ESTADO      = 0x90
};

/* --- Status do ACK (secao 5.19 do protocolo) ------------------------------- */

enum {
  J3S_ACK_OK                  = 0,
  J3S_ACK_ERRO                = 1,
  J3S_ACK_NAO_PERMITIDO       = 2,
  J3S_ACK_ROTA_CHEIA          = 3,
  J3S_ACK_MOTOR_OCUPADO       = 4,
  J3S_ACK_SENSOR_INATIVO      = 5,
  J3S_ACK_ARGUMENTO_INVALIDO  = 6
};

/* --- Eventos (secao 5.10 do protocolo) ------------------------------------- */

enum {
  J3S_EV_ROTA_CARREGADA  = 0,
  J3S_EV_ROTA_LIMPA      = 1,
  J3S_EV_MISSAO_INICIADA = 2,
  J3S_EV_CHEGOU_WAYPOINT = 3,
  J3S_EV_ROTA_CONCLUIDA  = 4,
  J3S_EV_MISSAO_PAUSADA  = 5,
  J3S_EV_MISSAO_ABORTADA = 6, /* motivo: 1 perdeu fix GPS, 2 desarmou */
  J3S_EV_TESTE_MOTOR_FIM = 7  /* status: 0 concluido, 1 abortado        */
};

/* --- Parser de frame ------------------------------------------------------- */

typedef struct {
  uint8_t  estado;        /* 0 magic, 1 len, 2 tipo, 3 payload, 4 crc */
  uint8_t  len;
  uint8_t  tipo;
  uint8_t  crc_calc;
  uint8_t  payload_ix;
  uint8_t  payload[J3SERIAL_PAYLOAD_MAX];
} j3serial_parser_t;

/* --- Rota (RAM, substitui a rota hardcoded de waypoints.h) ----------------- */

typedef struct {
  uint8_t   n;
  waypoint_t pontos[J3SERIAL_ROTA_MAX];
} j3serial_rota_t;

/* --- Missao ---------------------------------------------------------------- */

typedef struct {
  uint8_t ativa;   /* 1 = missao em andamento */
  uint8_t indice;  /* waypoint alvo atual     */
  uint8_t total;   /* waypoints da rota      */
} j3serial_missao_t;

/* --- FSM do teste de motor (subida -> pico -> descida) --------------------- */

typedef struct {
  uint8_t  ativo;            /* 1 = teste em andamento            */
  uint8_t  motor;            /* 1..4 (somente individual)         */
  uint8_t  fase;             /* 0 subida, 1 pico, 2 descida       */
  uint8_t  pico_pct;         /* pico da rampa (1..100)            */
  uint16_t subida_ms;
  uint16_t pico_ms;
  uint16_t descida_ms;
  uint32_t fase_inicio_ms;
  uint16_t percentual_mil;   /* 0..10000 (0,00%..100,00%), atomico */
  uint8_t  fim_pendente;     /* evento a gerar: 0 nenhum, 1 fim,
                                2 abortado                        */
} j3serial_teste_motor_t;

/* --- CRC8 e frame ---------------------------------------------------------- */

/**
 * @brief  CRC8 (poli 0x31, init 0x00, MSB-first, sem xorout).
 */
uint8_t j3serial_crc8(const uint8_t *dados, uint32_t n);

/**
 * @brief  Monta um frame completo em saida (magic+len+tipo+payload+crc).
 * @return tamanho total do frame, ou 0 se len > J3SERIAL_PAYLOAD_MAX.
 */
uint32_t j3serial_monta_frame(uint8_t tipo, const uint8_t *payload,
                              uint8_t len, uint8_t *saida);

/**
 * @brief  Alimenta o parser com um byte do stream.
 * @return 1 quando um frame completo e valido terminou (tipo/payload
 *         disponiveis em p->tipo/p->payload); 0 caso contrario.
 */
uint8_t j3serial_parser_byte(j3serial_parser_t *p, uint8_t byte);

/** Zera o parser (estado MAGIC). */
void j3serial_parser_reseta(j3serial_parser_t *p);

/* --- Helpers de payload (little-endian) ------------------------------------ */

uint8_t *j3serial_put_u8(uint8_t *b, uint8_t v);
uint8_t *j3serial_put_u16(uint8_t *b, uint16_t v);
uint8_t *j3serial_put_f32(uint8_t *b, float v);
uint16_t j3serial_get_u16(const uint8_t *b);
float    j3serial_get_f32(const uint8_t *b);

/* --- Rota ------------------------------------------------------------------ */

void j3serial_rota_limpa(j3serial_rota_t *rota);

/**
 * @brief  Carrega a rota a partir do payload do UPLOAD_ROTA.
 * @param  n_pontos saida com a quantidade carregada (quando OK).
 * @return status do ACK: J3S_ACK_OK, J3S_ACK_ROTA_CHEIA ou
 *         J3S_ACK_ARGUMENTO_INVALIDO (payload malformado).
 */
uint8_t j3serial_rota_carrega(j3serial_rota_t *rota,
                              const uint8_t *payload, uint8_t len,
                              uint8_t *n_pontos);

/* --- Missao ---------------------------------------------------------------- */

void j3serial_missao_inicia(j3serial_missao_t *m, uint8_t total);
void j3serial_missao_pausa(j3serial_missao_t *m);

/**
 * @brief  Avanca para o proximo waypoint.
 * @return 1 se avancou; 0 se ja estava no ultimo (rota concluida).
 */
uint8_t j3serial_missao_avanca(j3serial_missao_t *m);

/* --- Teste de motor -------------------------------------------------------- */

void j3serial_teste_motor_inicializa(j3serial_teste_motor_t *t);

/**
 * @brief  Inicia a rampa do motor (subida -> pico -> descida).
 * @return J3S_ACK_OK, J3S_ACK_MOTOR_OCUPADO (outro teste ativo) ou
 *         J3S_ACK_ARGUMENTO_INVALIDO (motor fora de 1..4, pico 0,
 *         subida/descida < 100 ms).
 */
uint8_t j3serial_teste_motor_inicia(j3serial_teste_motor_t *t,
                                    uint8_t motor, uint8_t pico_pct,
                                    uint16_t subida_ms, uint16_t pico_ms,
                                    uint16_t descida_ms, uint32_t agora_ms);

/** Aborta imediatamente (motor -> 0 e fim_pendente = 2 se havia teste). */
void j3serial_teste_motor_aborta(j3serial_teste_motor_t *t);

/**
 * @brief  Avanca a FSM com o tempo atual (chamar no loop principal).
 *         Quando termina: percentual = 0, ativo = 0, fim_pendente = 1.
 */
void j3serial_teste_motor_atualiza(j3serial_teste_motor_t *t, uint32_t agora_ms);

/** 1 quando ha teste de motor em andamento. */
uint8_t j3serial_teste_motor_ativa(const j3serial_teste_motor_t *t);

/** Percentual atual (0..10000 = 0,00%..100,00%) do motor em teste. */
uint16_t j3serial_teste_motor_percentual_mil(const j3serial_teste_motor_t *t);

/** Motor em teste (1..4) ou 0 se nenhum. */
uint8_t j3serial_teste_motor_motor(const j3serial_teste_motor_t *t);

#ifdef __cplusplus
}
#endif

#endif /* USR_SERIAL_J3SERIAL_H */