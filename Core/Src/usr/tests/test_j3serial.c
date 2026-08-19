/**
 * @file    test_j3serial.c
 * @brief   Testes nativos do protocolo j3Serial (CRC8, frame, parser,
 *          rota, missao e FSM do teste de motor).
 * @date    2026-08-18
 * @author  Nelson Lima
 * @ai      opencode (deepseek-v4-flash-free)
 *
 * Vetores de CRC8 e frames conforme docs/j3Serial_protocolo.md (secao 2.1
 * e secoes 8.x).
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "serial/j3serial.h"

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

static void alimenta(j3serial_parser_t *p, const uint8_t *frame, uint32_t n)
{
  uint32_t i;
  for (i = 0; i < n; ++i)
  {
    (void)j3serial_parser_byte(p, frame[i]);
  }
}

int testes_j3serial(void)
{
  /* ---------- CRC8: vetores da especificacao ---------- */
  {
    static const uint8_t v1[] = {0x81, 0x01, 0x00};
    static const uint8_t v2[] = {0x02};
    static const uint8_t v3[] = {0x03};
    static const uint8_t v5[] = {0x05};
    static const uint8_t v8[] = {0x08};
    static const uint8_t vu[] = {0x01, 0x01, 0x77, 0x67, 0xBC, 0xC1,
                                 0x82, 0x88, 0x3A, 0xC2, 0x00, 0x00, 0xF0, 0x42};
    static const uint8_t vt[] = {0x09, 0x02, 0x50, 0xD0, 0x07, 0xB8, 0x0B, 0xD0, 0x07};

    VERIFICA(j3serial_crc8(v1, sizeof(v1)) == 0x67, "CRC8 ACK [81 01 00] == 0x67");
    VERIFICA(j3serial_crc8(v2, sizeof(v2)) == 0x62, "CRC8 LIMPAR_ROTA == 0x62");
    VERIFICA(j3serial_crc8(v3, sizeof(v3)) == 0x53, "CRC8 INICIAR_MISSAO == 0x53");
    VERIFICA(j3serial_crc8(v5, sizeof(v5)) == 0xF5, "CRC8 CONSULTAR_ROTA == 0xF5");
    VERIFICA(j3serial_crc8(v8, sizeof(v8)) == 0xB9, "CRC8 PEDIR_TELEMETRIA == 0xB9");
    VERIFICA(j3serial_crc8(vu, sizeof(vu)) == 0xC5, "CRC8 UPLOAD 1 waypoint == 0xC5");
    VERIFICA(j3serial_crc8(vt, sizeof(vt)) == 0x81, "CRC8 TESTE_MOTOR == 0x81");
    VERIFICA(j3serial_crc8(0, 0) == 0x00, "CRC8 vazio == 0x00");
  }

  /* ---------- monta_frame ---------- */
  {
    uint8_t frame[16];
    uint8_t payload[2];
    uint32_t n;
    static const uint8_t esperado[] = {0xAA, 0x02, 0x81, 0x01, 0x00, 0x67};

    payload[0] = 0x01;
    payload[1] = 0x00;
    n = j3serial_monta_frame(J3S_MSG_ACK, payload, 2, frame);
    VERIFICA(n == sizeof(esperado), "monta_frame ACK: tamanho");
    VERIFICA(memcmp(frame, esperado, sizeof(esperado)) == 0,
             "monta_frame ACK: bytes (AA 02 81 01 00 67)");
    VERIFICA(j3serial_monta_frame(J3S_MSG_ACK, payload, 2, 0) == 0,
             "monta_frame: saida nula rejeitada");
  }

  /* ---------- parser: frame completo e fatiado ---------- */
  {
    static const uint8_t frame[] = {0xAA, 0x0D, 0x01, 0x01,
                                    0x77, 0x67, 0xBC, 0xC1, 0x82, 0x88, 0x3A, 0xC2,
                                    0x00, 0x00, 0xF0, 0x42, 0xC5};
    j3serial_parser_t p;
    uint32_t i;
    uint8_t completo = 0;

    j3serial_parser_reseta(&p);
    alimenta(&p, frame, sizeof(frame));
    VERIFICA(j3serial_parser_byte(&p, 0x00) == 0, "parser: sem frame apos termino");
    j3serial_parser_reseta(&p);
    for (i = 0; i < sizeof(frame); ++i)
    {
      completo = j3serial_parser_byte(&p, frame[i]);
    }
    VERIFICA(completo == 1, "parser: UPLOAD_ROTA completo byte a byte");
    VERIFICA(p.tipo == J3S_CMD_UPLOAD_ROTA, "parser: tipo == 0x01");
    VERIFICA(p.len == 13 && p.payload[0] == 1, "parser: payload (n=1)");
    VERIFICA(APROX(j3serial_get_f32(&p.payload[1]), -23.550520f, 1e-4f),
             "parser: lat decodificada");
    VERIFICA(APROX(j3serial_get_f32(&p.payload[5]), -46.633309f, 1e-4f),
             "parser: lon decodificada");
    VERIFICA(APROX(j3serial_get_f32(&p.payload[9]), 120.0f, 1e-3f),
             "parser: alt decodificada");
  }

  /* ---------- parser: resync apos lixo e CRC errado ---------- */
  {
    static const uint8_t lixo[] = {0x00, 0x55, 0xAA};
    static const uint8_t frame_ok[] = {0xAA, 0x00, 0x02, 0x62};
    static const uint8_t frame_bad[] = {0xAA, 0x00, 0x02, 0x00};
    j3serial_parser_t p;
    uint32_t i;
    uint8_t completo = 0;

    j3serial_parser_reseta(&p);
    alimenta(&p, lixo, sizeof(lixo));
    alimenta(&p, frame_ok, sizeof(frame_ok));
    VERIFICA(j3serial_parser_byte(&p, 0x00) == 0,
             "parser: lixo antes do magic nao gera frame falso");

    j3serial_parser_reseta(&p);
    alimenta(&p, frame_bad, sizeof(frame_bad));
    VERIFICA(j3serial_parser_byte(&p, 0x00) == 0, "parser: CRC errado rejeitado");
    alimenta(&p, frame_ok, sizeof(frame_ok));
    VERIFICA(j3serial_parser_byte(&p, 0x00) == 0,
             "parser: resync apos CRC errado");
  }

  /* ---------- parser: LEN acima do maximo descartado ---------- */
  {
    static const uint8_t frame[] = {0xAA, 0xFF, 0x01};
    j3serial_parser_t p;
    uint32_t i;

    j3serial_parser_reseta(&p);
    alimenta(&p, frame, sizeof(frame));
    VERIFICA(j3serial_parser_byte(&p, 0xAA) == 0, "parser: LEN 255 descartado");
  }

  /* ---------- rota ---------- */
  {
    j3serial_rota_t rota;
    uint8_t payload[1u + 2u * J3SERIAL_WP_TAMANHO];
    uint8_t n = 0;
    uint8_t *b = payload;
    uint8_t st;
    uint8_t payload_bad[1 + J3SERIAL_WP_TAMANHO];

    j3serial_rota_limpa(&rota);
    VERIFICA(rota.n == 0, "rota: limpa inicia com n=0");

    b = j3serial_put_u8(b, 2);
    b = j3serial_put_f32(b, -23.550520f);
    b = j3serial_put_f32(b, -46.633309f);
    b = j3serial_put_f32(b, 120.0f);
    b = j3serial_put_f32(b, -23.551000f);
    b = j3serial_put_f32(b, -46.634000f);
    b = j3serial_put_f32(b, 100.0f);
    st = j3serial_rota_carrega(&rota, payload, (uint8_t)sizeof(payload), &n);
    VERIFICA(st == J3S_ACK_OK && n == 2 && rota.n == 2,
             "rota: carrega 2 waypoints");
    VERIFICA(APROX(rota.pontos[1].alt_m, 100.0f, 1e-3f),
             "rota: alt do segundo ponto");

    payload_bad[0] = 0;
    st = j3serial_rota_carrega(&rota, payload_bad, 1, &n);
    VERIFICA(st == J3S_ACK_ARGUMENTO_INVALIDO, "rota: n=0 rejeitado");

    payload_bad[0] = 17;
    st = j3serial_rota_carrega(&rota, payload_bad, 1 + 17u * J3SERIAL_WP_TAMANHO, &n);
    VERIFICA(st == J3S_ACK_ROTA_CHEIA, "rota: n>16 rejeitado (ROTA_CHEIA)");

    payload_bad[0] = 1;
    memset(&payload_bad[1], 0, J3SERIAL_WP_TAMANHO);
    (void)j3serial_put_f32(&payload_bad[1], 91.0f); /* lat fora de -90..90 */
    st = j3serial_rota_carrega(&rota, payload_bad, 1 + 12u, &n);
    VERIFICA(st == J3S_ACK_ARGUMENTO_INVALIDO, "rota: lat invalida rejeitada");

    j3serial_rota_limpa(&rota);
    VERIFICA(rota.n == 0, "rota: limpa apaga");
  }

  /* ---------- helpers de payload ---------- */
  {
    uint8_t buf[8];
    uint8_t *b = buf;
    b = j3serial_put_u16(b, 0x1234);
    b = j3serial_put_f32(b, 3.141592f);
    VERIFICA(j3serial_get_u16(&buf[0]) == 0x1234, "helpers: u16 round-trip");
    VERIFICA(APROX(j3serial_get_f32(&buf[2]), 3.141592f, 1e-6f),
             "helpers: f32 round-trip");
  }

  /* ---------- missao ---------- */
  {
    j3serial_missao_t m;
    j3serial_missao_inicia(&m, 3);
    VERIFICA(m.ativa && m.indice == 0 && m.total == 3, "missao: inicia no 0");
    VERIFICA(j3serial_missao_avanca(&m) == 1 && m.indice == 1,
             "missao: avanca para 1");
    VERIFICA(j3serial_missao_avanca(&m) == 1 && m.indice == 2,
             "missao: avanca para 2");
    VERIFICA(j3serial_missao_avanca(&m) == 0 && !m.ativa,
             "missao: ultimo waypoint encerra");
    VERIFICA(j3serial_missao_avanca(&m) == 0, "missao: avanca apos fim == 0");
  }

  /* ---------- FSM do teste de motor ---------- */
  {
    j3serial_teste_motor_t t;

    j3serial_teste_motor_inicializa(&t);
    VERIFICA(!j3serial_teste_motor_ativa(&t), "teste: inicia parado");

    VERIFICA(j3serial_teste_motor_inicia(&t, 2, 80, 2000, 3000, 2000, 1000) ==
                 J3S_ACK_OK,
             "teste: inicia motor 2 pico 80%%");
    VERIFICA(j3serial_teste_motor_ativa(&t), "teste: ativo");
    VERIFICA(j3serial_teste_motor_inicia(&t, 1, 50, 1000, 3000, 1000, 2000) ==
                 J3S_ACK_MOTOR_OCUPADO,
             "teste: segundo inicio -> MOTOR_OCUPADO");

    j3serial_teste_motor_atualiza(&t, 1000); /* inicio: fase subida */
    VERIFICA(t.percentual_mil == 0, "teste: subida t=0 -> 0%%");
    j3serial_teste_motor_atualiza(&t, 2000); /* 1000 ms de subida */
    VERIFICA(t.percentual_mil == 4000, "teste: subida 50%% em 1 s");
    j3serial_teste_motor_atualiza(&t, 3000); /* fim da subida (2000 ms) */
    VERIFICA(t.percentual_mil == 8000 && t.fase == 1, "teste: pico alcancado");
    j3serial_teste_motor_atualiza(&t, 4000);
    VERIFICA(t.percentual_mil == 8000, "teste: mantem pico");
    j3serial_teste_motor_atualiza(&t, 6000); /* 3000 ms de pico */
    VERIFICA(t.fase == 2, "teste: entrou na descida");
    j3serial_teste_motor_atualiza(&t, 7000); /* metade da descida */
    VERIFICA(t.percentual_mil == 4000, "teste: descida 50%%");
    j3serial_teste_motor_atualiza(&t, 8000); /* fim da descida */
    VERIFICA(t.percentual_mil == 0 && !t.ativo && t.fim_pendente == 1,
             "teste: concluido (0%% + evento)");

    j3serial_teste_motor_inicializa(&t);
    VERIFICA(j3serial_teste_motor_inicia(&t, 1, 100, 1000, 1000, 1000, 0) ==
                 J3S_ACK_OK,
             "teste: reinicia");
    j3serial_teste_motor_atualiza(&t, 500);
    j3serial_teste_motor_aborta(&t);
    VERIFICA(t.percentual_mil == 0 && !t.ativo && t.fim_pendente == 2,
             "teste: abortado (0%% + evento abortado)");
    j3serial_teste_motor_aborta(&t);
    VERIFICA(t.fim_pendente == 2, "teste: aborta sem teste nao muda evento");

    /* Argumentos invalidos (FSM parado: MOTOR_OCUPADO tem precedencia). */
    VERIFICA(j3serial_teste_motor_inicia(&t, 0, 50, 1000, 3000, 1000, 2000) ==
                 J3S_ACK_ARGUMENTO_INVALIDO,
             "teste: motor 0 invalido");
    VERIFICA(j3serial_teste_motor_inicia(&t, 5, 50, 1000, 3000, 1000, 2000) ==
                 J3S_ACK_ARGUMENTO_INVALIDO,
             "teste: motor 5 invalido");
    VERIFICA(j3serial_teste_motor_inicia(&t, 1, 0, 1000, 3000, 1000, 2000) ==
                 J3S_ACK_ARGUMENTO_INVALIDO,
             "teste: pico 0 invalido");
    VERIFICA(j3serial_teste_motor_inicia(&t, 1, 50, 50, 3000, 1000, 2000) ==
                 J3S_ACK_ARGUMENTO_INVALIDO,
             "teste: subida < 100 ms invalida");
  }

  return falhas;
}