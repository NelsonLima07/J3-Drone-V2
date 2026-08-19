/**
 * @file    main_j3dronev2.c
 * @brief   Ponto de entrada do sistema de controle de voo J3_DroneV2.
 *          Aqui o sistema e inicializado e supervisionado.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Compilado no target raiz (junto com os glues HAL): usa main.h (LED_MODO)
 * e HAL_GetTick.
 */

#include "main.h"
#include "main_j3dronev2.h"
#include "control/controlador.h"
#include "control/pid_cascade.h"
#include "control/mixer.h"
#include "control/estimador_atitude.h"
#include "sensors/imuc42688.h"
#include "sensors/imuc42688_spi_hal.h"
#include "sensors/lis3mdl.h"
#include "sensors/bmp581.h"
#include "sensors/gps_uart_hal.h"
#include "sensors/i2c2_hal.h"
#include "sensors/i3c_bmp581_hal.h"
#include "system/hardware_glue.h"
#include "system/estados.h"
#include "system/calibracao.h"
#include "system/gps_led.h"
#include "serial/ibus_uart_hal.h"
#include "serial/radio_comandos.h"
#include "serial/gps_nmea.h"
#include "serial/j3serial.h"
#include "serial/j3serial_uart_hal.h"
#include "navigation/home_ponto.h"
#include "navigation/nav_posicao.h"
#include "navigation/nav_controle.h"
#include "math/filters.h"
#include "esc/dshot_timer_hal.h"

/* Periodo do loop de controle = ODR do gyro (8 kHz) */
#define DT_LOOP_TAXA 0.000125f /* 1/8000 s */

/* Calibracao: 2 s de amostras a 8 kHz */
#define CALIBRACAO_AMOSTRAS 16000U

/* LED de status (PB5, ativo alto) */
#define LED_ESPERA_PERIODO_MS  200U  /* 5 Hz  */
#define LED_CALIBRACAO_PERIODO_MS 1000U /* 1 Hz */

/* GPS / modo assistido (CH6 = indice 5, segundo AETR do receptor) */
#define GPS_MODO_CH          5u
#define GPS_SWITCH_ON        1700U
#define GPS_SATS_MIN         8u
#define GPS_HDOP_MAX         2.5f
#define GPS_FIX_JANELA_MS    1000U
#define GPS_I2C_PERIODO_MS   10U   /* 100 Hz para mag + baro         */
#define GPS_BARO_FC_HZ       1.0f  /* corte do LPF da altitude baro  */
#define GPS_VELV_FC_HZ       5.0f  /* corte do LPF da vel vertical   */

static controlador_t sistema_controlador;
static controlador_cascata_t sistema_cascata;
static imuc42688_t sistema_imu;
static setpoint_t setpoint;
static saida_controle_t saida;
static saida_misturador_t saida_misturada;

static estimador_atitude_t sistema_estimador;
static config_estimador_t config_estimador;
static config_radio_t config_radio;

static estados_t sistema_estados;
static config_estados_t config_estados;
static calibracao_t sistema_calibracao;

/* GPS: parser NMEA + home point + navegacao */
static gps_parser_t sistema_gps;
static config_gps_t config_gps;
static gps_medida_t gps_medida;
static uint32_t gps_ultima_versao;
static uint32_t gps_ultima_mudanca_ms;
static uint8_t  gps_fix_ok;

static home_ponto_t sistema_home;
static config_home_t config_home;

static nav_posicao_t sistema_nav;
static nav_controle_t sistema_nav_ctrl;
static nav_comando_t nav_cmd;
static uint8_t hold_engajado;
static float hold_lat, hold_lon, hold_alt;

/* Mag (LIS3MDL) na I2C2. baro (BMP581) na I3C1. mag_sombra e consumido na ISR. */
static lis3mdl_t sistema_mag;
static bmp581_t sistema_baro;
static imuc_transport_t transporte_mag;
static imuc_transport_t transporte_baro;
static volatile float mag_sombra[3];
static volatile uint8_t mag_pronto;
static uint8_t mag_ok;
static uint8_t baro_ok;
static uint32_t proxima_leitura_i2c_ms;
static uint32_t ultima_baro_ms;
static filtro_lpf1_t baro_alt_filtro;
static filtro_lpf1_t vel_v_filtro;
static float baro_alt;
static float baro_alt_ant;
static float vel_v;

/* Escrito pelo loop principal, lido pela ISR do gyro (ao_medir).
   Byte unico: leitura/escrita atomicas no ARM. */
static volatile uint8_t controle_habilitado;

/* Setpoint escrito pelo loop principal e copiado pela ISR (padrao dshot). */
static setpoint_t setpoint_sombra;
static volatile uint8_t setpoint_pronto;

static uint8_t imu_ok;

/* j3Serial (USART1): waypoints, telemetria, sensores e teste de motor. */
static j3serial_parser_t sistema_j3s_parser;
static j3serial_rota_t sistema_rota;
static j3serial_missao_t sistema_missao;
static j3serial_teste_motor_t sistema_teste_motor;
static uint32_t telemetria_periodo_ms;
static uint32_t proxima_telemetria_ms;
static estado_modo_t modo_atual;
static float dist_alvo_m;

/* Sombra do teste de motor (escrita no loop, lida na ISR do gyro). */
static volatile uint8_t teste_sombra_ativo;
static volatile uint8_t teste_sombra_motor; /* 0 = nenhum */
static volatile uint16_t teste_sombra_percentual_mil;

/* Saida atual dos motores em % (escrita na ISR, lida no PEDIR_ESTADO). */
static volatile uint8_t motores_saida_pct[MIXER_NUM_MOTORES];

/**
 * @brief  Callback chamado pelo glue a cada amostra do ICM-42688-P
 *         (contexto da interrupcao da DMA SPI1).
 */
static void ao_medir(const imu_medida_t *medida)
{
  imu_medida_t m;
  atitude_t atitude = {0};
  velocidade_angular_t taxa;

  if (medida == 0)
  {
    return;
  }
  m = *medida;

  /* Calibracao: acumula bias do gyro e referencia do accel (quando ativa). */
  (void)calibracao_processa(&sistema_calibracao, &m);
  calibracao_aplica_bias(&sistema_calibracao, &m, &m);

  /* Estimador de atitude roda sempre (converge antes de armar). */
  estimador_atitude_atualiza(&sistema_estimador, m.giro_rad_s,
                             m.aceleracao_m_s2, DT_LOOP_TAXA);

  /* Correcao de guinada quando ha amostra nova do LIS3MDL (~100 Hz). */
  if (mag_pronto)
  {
    float mag[3];
    mag[0] = mag_sombra[0];
    mag[1] = mag_sombra[1];
    mag[2] = mag_sombra[2];
    estimador_atitude_atualiza_mag(&sistema_estimador, mag, DT_LOOP_TAXA);
    mag_pronto = 0;
  }

  /* Teste de motor (somente desarmado): o motor testado recebe a rampa
     e os demais ficam em 0 (bypass do mixer). */
  if (teste_sombra_ativo && !controle_habilitado)
  {
    saida_misturada.motores[0] = 0.0f;
    saida_misturada.motores[1] = 0.0f;
    saida_misturada.motores[2] = 0.0f;
    saida_misturada.motores[3] = 0.0f;
    if (teste_sombra_motor >= 1u && teste_sombra_motor <= MIXER_NUM_MOTORES)
    {
      saida_misturada.motores[teste_sombra_motor - 1u] =
          (float)teste_sombra_percentual_mil * 0.0001f;
    }
    dshot_escreve(&saida_misturada, 1u);
    motores_saida_pct[0] = (uint8_t)(saida_misturada.motores[0] * 100.0f);
    motores_saida_pct[1] = (uint8_t)(saida_misturada.motores[1] * 100.0f);
    motores_saida_pct[2] = (uint8_t)(saida_misturada.motores[2] * 100.0f);
    motores_saida_pct[3] = (uint8_t)(saida_misturada.motores[3] * 100.0f);
    return;
  }

  if (!controle_habilitado)
  {
    /* Desarmado: mantem os frames zero nos 4 ESCs (o DMA re-arma sozinho). */
    dshot_escreve(&saida_misturada, 0u);
    motores_saida_pct[0] = 0;
    motores_saida_pct[1] = 0;
    motores_saida_pct[2] = 0;
    motores_saida_pct[3] = 0;
    return;
  }

  /* Copia o setpoint escrito pelo loop principal (atomico: mesma prioridade). */
  if (setpoint_pronto)
  {
    setpoint = setpoint_sombra;
    setpoint_pronto = 0;
  }

  taxa.angvel.x = m.giro_rad_s[0];
  taxa.angvel.y = m.giro_rad_s[1];
  taxa.angvel.z = m.giro_rad_s[2];
  atitude.euler = estimador_atitude_obtem_euler(&sistema_estimador);

  controlador_atualiza(&sistema_controlador, &atitude, &taxa,
                       &setpoint, DT_LOOP_TAXA, &saida);

  misturador_aplica(saida.thrust, &saida.torque, &saida_misturada);
  misturador_normaliza(&saida_misturada);
  dshot_escreve(&saida_misturada, 1u);
  motores_saida_pct[0] = (uint8_t)(saida_misturada.motores[0] * 100.0f);
  motores_saida_pct[1] = (uint8_t)(saida_misturada.motores[1] * 100.0f);
  motores_saida_pct[2] = (uint8_t)(saida_misturada.motores[2] * 100.0f);
  motores_saida_pct[3] = (uint8_t)(saida_misturada.motores[3] * 100.0f);
}

/* --- j3Serial (USART1): respostas e despacho do protocolo ------------------ */

static void j3s_envia(uint8_t tipo, const uint8_t *payload, uint8_t len)
{
  uint8_t frame[J3SERIAL_FRAME_MAX];
  uint32_t n = j3serial_monta_frame(tipo, payload, len, frame);
  if (n > 0)
  {
    (void)j3serial_uart_envia(frame, n);
  }
}

static void j3s_ack(uint8_t tipo_origem, uint8_t status)
{
  uint8_t p[2];
  p[0] = tipo_origem;
  p[1] = status;
  j3s_envia(J3S_MSG_ACK, p, 2);
}

static void j3s_evento(uint8_t evento, const uint8_t *dados, uint8_t n)
{
  uint8_t p[3];
  p[0] = evento;
  if (n > 0)
  {
    p[1] = dados[0];
  }
  if (n > 1)
  {
    p[2] = dados[1];
  }
  j3s_envia(J3S_MSG_EVENTO, p, 1u + n);
}

static void j3s_telemetria_monta(uint8_t *payload)
{
  uint8_t *b = payload;
  vetor3_t euler;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  euler = estimador_atitude_obtem_euler(&sistema_estimador);
  __set_PRIMASK(primask);

  b = j3serial_put_u8(b, (uint8_t)modo_atual);
  b = j3serial_put_u8(b, controle_habilitado);
  b = j3serial_put_f32(b, euler.x);
  b = j3serial_put_f32(b, euler.y);
  b = j3serial_put_f32(b, euler.z);
  b = j3serial_put_f32(b, gps_medida.lat_deg);
  b = j3serial_put_f32(b, gps_medida.lon_deg);
  b = j3serial_put_f32(b, gps_medida.alt_m);
  b = j3serial_put_f32(b, gps_medida.vel_m_s);
  b = j3serial_put_f32(b, gps_medida.curso_deg);
  b = j3serial_put_u8(b, gps_medida.num_sats);
  b = j3serial_put_f32(b, gps_medida.hdop);
  b = j3serial_put_f32(b, sistema_home.lat_deg);
  b = j3serial_put_f32(b, sistema_home.lon_deg);
  b = j3serial_put_f32(b, sistema_home.alt_m);
  b = j3serial_put_u8(b, home_ponto_valido(&sistema_home));
  b = j3serial_put_u16(b, sistema_missao.ativa ? sistema_missao.indice : 0xFFFFu);
  b = j3serial_put_u16(b, sistema_rota.n);
  b = j3serial_put_f32(b, dist_alvo_m);
  b = j3serial_put_f32(b, baro_alt);
}

static void j3s_envia_telemetria(void)
{
  uint8_t payload[61];
  j3s_telemetria_monta(payload);
  j3s_envia(J3S_MSG_TELEMETRIA, payload, (uint8_t)sizeof(payload));
}

static void j3s_resposta_rota(void)
{
  uint8_t payload[1u + J3SERIAL_ROTA_MAX * J3SERIAL_WP_TAMANHO];
  uint8_t *b = payload;
  uint8_t i;

  b = j3serial_put_u8(b, sistema_rota.n);
  for (i = 0; i < sistema_rota.n; ++i)
  {
    b = j3serial_put_f32(b, sistema_rota.pontos[i].lat_deg);
    b = j3serial_put_f32(b, sistema_rota.pontos[i].lon_deg);
    b = j3serial_put_f32(b, sistema_rota.pontos[i].alt_m);
  }
  j3s_envia(J3S_MSG_ROTA_RESPOSTA, payload,
            1u + (uint8_t)(sistema_rota.n * J3SERIAL_WP_TAMANHO));
}

static void j3s_resposta_imu(void)
{
  const imu_medida_t *m;

  if (!imu_ok)
  {
    j3s_ack(J3S_CMD_PEDIR_IMU, J3S_ACK_SENSOR_INATIVO);
    return;
  }
  m = imuc42688_medida(&sistema_imu);
  if (m == 0)
  {
    j3s_ack(J3S_CMD_PEDIR_IMU, J3S_ACK_SENSOR_INATIVO);
    return;
  }
  {
    uint8_t payload[28];
    uint8_t *b = payload;
    b = j3serial_put_f32(b, m->temp_c);
    b = j3serial_put_f32(b, m->aceleracao_m_s2[0]);
    b = j3serial_put_f32(b, m->aceleracao_m_s2[1]);
    b = j3serial_put_f32(b, m->aceleracao_m_s2[2]);
    b = j3serial_put_f32(b, m->giro_rad_s[0]);
    b = j3serial_put_f32(b, m->giro_rad_s[1]);
    b = j3serial_put_f32(b, m->giro_rad_s[2]);
    j3s_envia(J3S_MSG_RESPOSTA_IMU, payload, (uint8_t)sizeof(payload));
  }
}

static void j3s_resposta_mag(void)
{
  const mag_medida_t *m;

  if (!mag_ok)
  {
    j3s_ack(J3S_CMD_PEDIR_MAG, J3S_ACK_SENSOR_INATIVO);
    return;
  }
  m = lis3mdl_medida(&sistema_mag);
  if (m == 0)
  {
    j3s_ack(J3S_CMD_PEDIR_MAG, J3S_ACK_SENSOR_INATIVO);
    return;
  }
  {
    uint8_t payload[12];
    uint8_t *b = payload;
    b = j3serial_put_f32(b, m->x);
    b = j3serial_put_f32(b, m->y);
    b = j3serial_put_f32(b, m->z);
    j3s_envia(J3S_MSG_RESPOSTA_MAG, payload, (uint8_t)sizeof(payload));
  }
}

static void j3s_resposta_baro(void)
{
  const baro_medida_t *m;

  if (!baro_ok)
  {
    j3s_ack(J3S_CMD_PEDIR_BARO, J3S_ACK_SENSOR_INATIVO);
    return;
  }
  m = bmp581_medida(&sistema_baro);
  if (m == 0)
  {
    j3s_ack(J3S_CMD_PEDIR_BARO, J3S_ACK_SENSOR_INATIVO);
    return;
  }
  {
    uint8_t payload[12];
    uint8_t *b = payload;
    b = j3serial_put_f32(b, m->temperatura_c);
    b = j3serial_put_f32(b, m->pressao_pa);
    b = j3serial_put_f32(b, m->altitude_m);
    j3s_envia(J3S_MSG_RESPOSTA_BARO, payload, (uint8_t)sizeof(payload));
  }
}

static void j3s_resposta_gps(void)
{
  uint8_t payload[33];
  uint8_t *b = payload;
  b = j3serial_put_u8(b, gps_medida.fix_valido);
  b = j3serial_put_u8(b, gps_medida.num_sats);
  b = j3serial_put_f32(b, gps_medida.lat_deg);
  b = j3serial_put_f32(b, gps_medida.lon_deg);
  b = j3serial_put_f32(b, gps_medida.alt_m);
  b = j3serial_put_f32(b, gps_medida.vel_m_s);
  b = j3serial_put_f32(b, gps_medida.curso_deg);
  b = j3serial_put_f32(b, gps_medida.hdop);
  b = j3serial_put_f32(b, gps_medida.erro_estimado_m);
  j3s_envia(J3S_MSG_RESPOSTA_GPS, payload, (uint8_t)sizeof(payload));
}

static void j3s_resposta_atitude(void)
{
  vetor3_t euler;
  uint32_t primask;
  uint8_t payload[12];
  uint8_t *b = payload;

  primask = __get_PRIMASK();
  __disable_irq();
  euler = estimador_atitude_obtem_euler(&sistema_estimador);
  __set_PRIMASK(primask);

  b = j3serial_put_f32(b, euler.x);
  b = j3serial_put_f32(b, euler.y);
  b = j3serial_put_f32(b, euler.z);
  j3s_envia(J3S_MSG_RESPOSTA_ATITUDE, payload, (uint8_t)sizeof(payload));
}

static void j3s_resposta_estado(void)
{
  uint8_t payload[7u + J3SERIAL_ROTA_MAX * J3SERIAL_WP_TAMANHO];
  uint8_t *b = payload;
  uint8_t i;

  b = j3serial_put_u8(b, (uint8_t)modo_atual);
  b = j3serial_put_u8(b, controle_habilitado);
  b = j3serial_put_u8(b, motores_saida_pct[0]);
  b = j3serial_put_u8(b, motores_saida_pct[1]);
  b = j3serial_put_u8(b, motores_saida_pct[2]);
  b = j3serial_put_u8(b, motores_saida_pct[3]);
  b = j3serial_put_u8(b, sistema_rota.n);
  for (i = 0; i < sistema_rota.n; ++i)
  {
    b = j3serial_put_f32(b, sistema_rota.pontos[i].lat_deg);
    b = j3serial_put_f32(b, sistema_rota.pontos[i].lon_deg);
    b = j3serial_put_f32(b, sistema_rota.pontos[i].alt_m);
  }
  j3s_envia(J3S_MSG_RESPOSTA_ESTADO, payload,
            7u + (uint8_t)(sistema_rota.n * J3SERIAL_WP_TAMANHO));
}

static void j3s_processa_comando(const j3serial_parser_t *p, uint32_t agora_ms)
{
  switch (p->tipo)
  {
    case J3S_CMD_UPLOAD_ROTA:
    {
      uint8_t n = 0;
      uint8_t st = j3serial_rota_carrega(&sistema_rota, p->payload, p->len, &n);
      j3s_ack(J3S_CMD_UPLOAD_ROTA, st);
      if (st == J3S_ACK_OK)
      {
        j3serial_missao_pausa(&sistema_missao);
        j3s_evento(J3S_EV_ROTA_CARREGADA, &n, 1);
      }
      break;
    }

    case J3S_CMD_LIMPAR_ROTA:
      j3serial_rota_limpa(&sistema_rota);
      j3serial_missao_pausa(&sistema_missao);
      j3s_ack(J3S_CMD_LIMPAR_ROTA, J3S_ACK_OK);
      j3s_evento(J3S_EV_ROTA_LIMPA, 0, 0);
      break;

    case J3S_CMD_INICIAR_MISSAO:
    {
      uint8_t st;
      if (j3serial_teste_motor_ativa(&sistema_teste_motor))
      {
        st = J3S_ACK_MOTOR_OCUPADO;
      }
      else if (!controle_habilitado || !gps_fix_ok ||
               !home_ponto_valido(&sistema_home))
      {
        st = J3S_ACK_NAO_PERMITIDO;
      }
      else if (sistema_rota.n == 0)
      {
        st = J3S_ACK_ERRO;
      }
      else
      {
        st = J3S_ACK_OK;
      }
      j3s_ack(J3S_CMD_INICIAR_MISSAO, st);
      if (st == J3S_ACK_OK)
      {
        j3serial_missao_inicia(&sistema_missao, sistema_rota.n);
        nav_posicao_definir_alvo(&sistema_nav,
                                 sistema_rota.pontos[0].lat_deg,
                                 sistema_rota.pontos[0].lon_deg,
                                 sistema_rota.pontos[0].alt_m);
        j3s_evento(J3S_EV_MISSAO_INICIADA, 0, 0);
      }
      break;
    }

    case J3S_CMD_PAUSAR_MISSAO:
      if (sistema_missao.ativa)
      {
        j3serial_missao_pausa(&sistema_missao);
        j3s_evento(J3S_EV_MISSAO_PAUSADA, 0, 0);
      }
      j3s_ack(J3S_CMD_PAUSAR_MISSAO, J3S_ACK_OK);
      break;

    case J3S_CMD_CONSULTAR_ROTA:
      j3s_resposta_rota();
      break;

    case J3S_CMD_CONFIG_TELEMETRIA:
    {
      uint16_t periodo = (p->len >= 2) ? j3serial_get_u16(p->payload) : 0;
      telemetria_periodo_ms = (periodo == 0) ? 0 :
                              ((periodo < 50) ? 50 : periodo);
      proxima_telemetria_ms = agora_ms + telemetria_periodo_ms;
      j3s_ack(J3S_CMD_CONFIG_TELEMETRIA, J3S_ACK_OK);
      break;
    }

    case J3S_CMD_PEDIR_TELEMETRIA:
      j3s_envia_telemetria();
      break;

    case J3S_CMD_TESTE_MOTOR:
    {
      uint8_t st;
      if (p->len != 8)
      {
        st = J3S_ACK_ARGUMENTO_INVALIDO;
      }
      else if (controle_habilitado)
      {
        st = J3S_ACK_NAO_PERMITIDO;
      }
      else
      {
        st = j3serial_teste_motor_inicia(&sistema_teste_motor,
                                         p->payload[0], p->payload[1],
                                         j3serial_get_u16(&p->payload[2]),
                                         j3serial_get_u16(&p->payload[4]),
                                         j3serial_get_u16(&p->payload[6]),
                                         agora_ms);
      }
      j3s_ack(J3S_CMD_TESTE_MOTOR, st);
      break;
    }

    case J3S_CMD_PARAR_TESTE:
      j3serial_teste_motor_aborta(&sistema_teste_motor);
      j3s_ack(J3S_CMD_PARAR_TESTE, J3S_ACK_OK);
      break;

    case J3S_CMD_PEDIR_IMU:
      j3s_resposta_imu();
      break;

    case J3S_CMD_PEDIR_MAG:
      j3s_resposta_mag();
      break;

    case J3S_CMD_PEDIR_BARO:
      j3s_resposta_baro();
      break;

    case J3S_CMD_PEDIR_GPS:
      j3s_resposta_gps();
      break;

    case J3S_CMD_PEDIR_ATITUDE:
      j3s_resposta_atitude();
      break;

    case J3S_CMD_PEDIR_ESTADO:
      j3s_resposta_estado();
      break;

    default:
      break;
  }
}

static void j3s_teste_sombra_atualiza(void)
{
  teste_sombra_ativo = j3serial_teste_motor_ativa(&sistema_teste_motor);
  teste_sombra_motor = j3serial_teste_motor_motor(&sistema_teste_motor);
  teste_sombra_percentual_mil =
      j3serial_teste_motor_percentual_mil(&sistema_teste_motor);
}

static void j3s_checa_fim_teste(void)
{
  if (sistema_teste_motor.fim_pendente)
  {
    uint8_t dados[2];
    dados[0] = sistema_teste_motor.motor;
    dados[1] = (sistema_teste_motor.fim_pendente == 2u) ? 1u : 0u;
    sistema_teste_motor.fim_pendente = 0;
    j3s_evento(J3S_EV_TESTE_MOTOR_FIM, dados, 2);
  }
}

static void sistema_inicializa(void)
{
  config_cascata_t config;

  cascata_config_padrao(&config);
  controlador_inicializa(&sistema_controlador, &ops_cascata, &sistema_cascata, &config);

  /* Estimador de atitude (fusao giro+accel) e mapa dos canais do rádio. */
  estimador_config_padrao(&config_estimador);
  estimador_atitude_inicializa(&sistema_estimador, &config_estimador, 0.0f, 0.0f, 0.0f);
  radio_comandos_config_padrao(&config_radio);
  setpoint_pronto = 0;

  /* Inicializa o ICM-42688-P: transporte SPI+DMA, WHO_AM_I e configuração */
  imu_ok = 0;
  if (imuc_hal_vincular(&sistema_imu, ao_medir) == 0 &&
      imuc42688_verifica_id(&sistema_imu) &&
      imuc42688_configura(&sistema_imu) == 0)
  {
    imu_ok = 1;
  }

  /* GPS (BN-220): parser NMEA + home point + navegacao. */
  gps_config_padrao(&config_gps);
  gps_nmea_inicializa(&sistema_gps, &config_gps);
  gps_ultima_versao = 0;
  gps_ultima_mudanca_ms = 0;
  gps_fix_ok = 0;
  config_home_padrao(&config_home);
  home_ponto_inicializa(&sistema_home, &config_home);
  nav_controle_inicializa(&sistema_nav_ctrl, 0);
  nav_posicao_inicializa(&sistema_nav, 1.5f);
  hold_engajado = 0;

  /* Mag (LIS3MDL, 0x1C) na I2C2 (polling) e baro (BMP581, 0x46) na I3C1. */
  (void)i2c2_hal_vincula(&transporte_mag, LIS3MDL_ENDERECO);
  (void)i3c_bmp581_hal_vincula(&transporte_baro);
  mag_ok = 0;
  baro_ok = 0;
  if (lis3mdl_inicializa(&sistema_mag, &transporte_mag) == 0 &&
      lis3mdl_verifica_id(&sistema_mag) &&
      lis3mdl_configura(&sistema_mag) == 0)
  {
    mag_ok = 1;
  }
  if (bmp581_inicializa(&sistema_baro, &transporte_baro) == 0 &&
      bmp581_verifica_id(&sistema_baro) &&
      bmp581_configura(&sistema_baro) == 0)
  {
    baro_ok = 1;
  }
  mag_pronto = 0;
  mag_sombra[0] = 0.0f;
  mag_sombra[1] = 0.0f;
  mag_sombra[2] = 0.0f;
  filtro_lpf1_inicializa(&baro_alt_filtro, GPS_BARO_FC_HZ, 0.01f);
  filtro_lpf1_inicializa(&vel_v_filtro, GPS_VELV_FC_HZ, 0.01f);
  baro_alt = 0.0f;
  baro_alt_ant = 0.0f;
  vel_v = 0.0f;
  proxima_leitura_i2c_ms = 0;
  ultima_baro_ms = 0;

  /* j3Serial: parser, rota, missao e teste de motor (USART1). */
  j3serial_parser_reseta(&sistema_j3s_parser);
  j3serial_rota_limpa(&sistema_rota);
  j3serial_missao_pausa(&sistema_missao);
  j3serial_teste_motor_inicializa(&sistema_teste_motor);
  telemetria_periodo_ms = 0;
  proxima_telemetria_ms = 0;
  modo_atual = ESTADO_ESPERA;
  dist_alvo_m = 0.0f;
  j3s_teste_sombra_atualiza();
}

static void led_escreve(uint8_t aceso)
{
  HAL_GPIO_WritePin(LED_MODO_GPIO_Port, LED_MODO_Pin,
                    aceso ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void led_atualiza(estado_modo_t modo, uint8_t tem_sinal, uint32_t agora_ms)
{
  uint32_t fase;

  /* Sem sinal do receptor: LED apagado. */
  if (!tem_sinal)
  {
    led_escreve(0);
    return;
  }

  switch (modo)
  {
    case ESTADO_VOO:
      led_escreve(1);
      break;
    case ESTADO_CALIBRACAO:
      fase = agora_ms % LED_CALIBRACAO_PERIODO_MS;
      led_escreve(fase < (LED_CALIBRACAO_PERIODO_MS / 2U) ? 1 : 0);
      break;
    case ESTADO_ESPERA:
    default:
      fase = agora_ms % LED_ESPERA_PERIODO_MS;
      led_escreve(fase < (LED_ESPERA_PERIODO_MS / 2U) ? 1 : 0);
      break;
  }
}

void main_j3dronev2(void)
{
  estado_modo_t modo;
  estado_modo_t modo_ant;
  uint16_t canais[IBUS_NUM_CANAIS];
  uint8_t tem_sinal;
  uint32_t agora;

  /* Reparo em codigo da regressao do CubeMX (PLL2, DMA, NVIC, IRQ). */
  hardware_glue_inicializa();

  sistema_inicializa();

  config_estados_padrao(&config_estados);
  estados_inicializa(&sistema_estados, &config_estados);
  calibracao_inicializa(&sistema_calibracao);
  controle_habilitado = 0;

  (void)ibus_uart_inicializa();
  (void)dshot_timer_inicializa();
  (void)gps_uart_inicializa();
  (void)j3serial_uart_inicializa();
  gps_led_inicializa();

  modo = estados_atualiza(&sistema_estados, &config_estados, 0, 0, 0, 1, HAL_GetTick());
  modo_ant = modo;

  while (1)
  {
    uint8_t ch6_on;
    uint8_t home_pronto;
    uint8_t gps_ok;
    uint8_t armar_permitido;

    agora = HAL_GetTick();

    tem_sinal = (uint8_t)ibus_uart_tem_sinal(agora);
    (void)ibus_uart_canais(canais);

    /* GPS: esvazia o ring (USART3 + DMA GPDMA1 CH7) no parser NMEA. */
    {
      uint8_t gps_bytes[64];
      uint32_t n = gps_uart_ler(gps_bytes, sizeof(gps_bytes));
      if (n > 0)
      {
        (void)gps_nmea_processa_buffer(&sistema_gps, gps_bytes, n);
      }
    }
    if (sistema_gps.medida.versao != gps_ultima_versao)
    {
      gps_ultima_versao = sistema_gps.medida.versao;
      gps_medida = gps_nmea_medida(&sistema_gps);
      gps_ultima_mudanca_ms = agora;
      if (!controle_habilitado)
      {
        home_ponto_atualiza(&sistema_home, &gps_medida);
      }
    }
    gps_fix_ok = (uint8_t)(gps_medida.fix_valido &&
                  gps_medida.num_sats >= GPS_SATS_MIN &&
                  gps_medida.hdop > 0.0f && gps_medida.hdop < GPS_HDOP_MAX &&
                  (uint32_t)(agora - gps_ultima_mudanca_ms) < GPS_FIX_JANELA_MS);

    /* j3Serial: esvazia o ring (USART1) e processa os comandos do host. */
    {
      uint8_t j3s_bytes[64];
      uint32_t n = j3serial_uart_ler(j3s_bytes, sizeof(j3s_bytes));
      uint32_t i;
      for (i = 0; i < n; ++i)
      {
        if (j3serial_parser_byte(&sistema_j3s_parser, j3s_bytes[i]))
        {
          j3s_processa_comando(&sistema_j3s_parser, agora);
        }
      }
    }

    /* Mag e baro na I2C2 em cadencia de 100 Hz (polling). */
    if ((int32_t)(agora - proxima_leitura_i2c_ms) >= 0)
    {
      proxima_leitura_i2c_ms = agora + GPS_I2C_PERIODO_MS;
      if (mag_ok && lis3mdl_processa(&sistema_mag))
      {
        const mag_medida_t *m = lis3mdl_medida(&sistema_mag);
        if (m != 0)
        {
          mag_sombra[0] = m->x;
          mag_sombra[1] = m->y;
          mag_sombra[2] = m->z;
          mag_pronto = 1;
        }
      }
      if (baro_ok && bmp581_processa(&sistema_baro))
      {
        float dt_s = (float)(agora - ultima_baro_ms) * 0.001f;
        baro_alt = filtro_lpf1_aplica(&baro_alt_filtro,
                                      sistema_baro.medida.altitude_m);
        if (dt_s < 0.001f || dt_s > 0.05f)
        {
          dt_s = 0.01f;
        }
        vel_v = filtro_lpf1_aplica(&vel_v_filtro,
                                   (baro_alt - baro_alt_ant) / dt_s);
        baro_alt_ant = baro_alt;
        ultima_baro_ms = agora;
      }
    }

    /* Modo assistido (CH6): habilita o GPS no armar e o position hold. */
    ch6_on = (canais[GPS_MODO_CH] >= GPS_SWITCH_ON) ? 1u : 0u;
    home_pronto = (uint8_t)(sistema_home.valido &&
                   sistema_home.erro_m <= config_home.erro_max_m);
    gps_ok = (uint8_t)(gps_fix_ok && home_pronto);
    armar_permitido = ch6_on ? gps_ok : 1u;

    /* Sticks -> setpoint (shadow lido pela ISR do controle). */
    radio_comandos_processa(&config_radio, canais, controle_habilitado,
                            &setpoint_sombra);

    /* Missao j3Serial: navegacao para o waypoint atual da rota. */
    if (sistema_missao.ativa)
    {
      if (controle_habilitado && gps_fix_ok)
      {
        const waypoint_t *wp = &sistema_rota.pontos[sistema_missao.indice];
        float norte_m, leste_m, dist_m, guinada;
        float vel_n, vel_e, rumo;
        vetor3_t euler;
        uint32_t primask;

        nav_posicao_erro_ned(wp->lat_deg, wp->lon_deg,
                             gps_medida.lat_deg, gps_medida.lon_deg,
                             &norte_m, &leste_m);
        dist_m = nav_posicao_distancia_m(wp->lat_deg, wp->lon_deg,
                                         gps_medida.lat_deg,
                                         gps_medida.lon_deg);
        rumo = gps_medida.curso_deg * 3.14159265358979323846f / 180.0f;
        vel_n = cosf(rumo) * gps_medida.vel_m_s;
        vel_e = sinf(rumo) * gps_medida.vel_m_s;

        primask = __get_PRIMASK();
        __disable_irq();
        euler = estimador_atitude_obtem_euler(&sistema_estimador);
        __set_PRIMASK(primask);
        guinada = euler.z;

        nav_controle_atualiza(&sistema_nav_ctrl, &nav_cmd, guinada,
                              -norte_m, -leste_m, vel_n, vel_e,
                              wp->alt_m - baro_alt, vel_v,
                              dist_m, gps_medida.vel_m_s);

        setpoint_sombra.atitude.euler.x = nav_cmd.rol_des_rad;
        setpoint_sombra.atitude.euler.y = nav_cmd.arfagem_des_rad;
        setpoint_sombra.taxa.angvel.z = nav_cmd.taxa_guinada_rad_s;
        setpoint_sombra.throttle = nav_cmd.throttle;
        dist_alvo_m = dist_m;

        if (nav_posicao_atingiu(&sistema_nav, gps_medida.lat_deg,
                                gps_medida.lon_deg))
        {
          uint8_t idx = sistema_missao.indice;
          if (j3serial_missao_avanca(&sistema_missao))
          {
            /* Chegou em um waypoint intermediario: avanca para o proximo. */
            wp = &sistema_rota.pontos[sistema_missao.indice];
            nav_posicao_definir_alvo(&sistema_nav, wp->lat_deg, wp->lon_deg,
                                     wp->alt_m);
            j3s_evento(J3S_EV_CHEGOU_WAYPOINT, &idx, 1);
          }
          else
          {
            /* Ultimo waypoint: conclui a rota e segura a posicao final. */
            j3s_evento(J3S_EV_CHEGOU_WAYPOINT, &idx, 1);
            j3s_evento(J3S_EV_ROTA_CONCLUIDA, 0, 0);
            hold_lat = gps_medida.lat_deg;
            hold_lon = gps_medida.lon_deg;
            hold_alt = baro_alt;
            nav_posicao_definir_alvo(&sistema_nav, hold_lat, hold_lon, hold_alt);
            hold_engajado = 1;
          }
        }
      }
      else
      {
        /* Perdeu o fix ou desarmou durante a missao: aborta. */
        uint8_t motivo = controle_habilitado ? 1u : 2u;
        j3serial_missao_pausa(&sistema_missao);
        hold_engajado = 0;
        dist_alvo_m = 0.0f;
        j3s_evento(J3S_EV_MISSAO_ABORTADA, &motivo, 1);
      }
    }
    else if (controle_habilitado && ch6_on && gps_fix_ok && mag_ok && baro_ok)
    {
      /* Position hold: engaja armado + CH6 + GPS/mag/baro ok. */
      if (!hold_engajado)
      {
        hold_lat = gps_medida.lat_deg;
        hold_lon = gps_medida.lon_deg;
        hold_alt = baro_alt;
        nav_posicao_inicializa(&sistema_nav, 1.5f);
        nav_posicao_definir_alvo(&sistema_nav, hold_lat, hold_lon, hold_alt);
        hold_engajado = 1;
      }
      {
        float norte_m, leste_m, dist_m, guinada;
        float vel_n, vel_e, rumo;
        vetor3_t euler;
        uint32_t primask;

        nav_posicao_erro_ned(hold_lat, hold_lon,
                             gps_medida.lat_deg, gps_medida.lon_deg,
                             &norte_m, &leste_m);
        dist_m = nav_posicao_distancia_m(hold_lat, hold_lon,
                                         gps_medida.lat_deg,
                                         gps_medida.lon_deg);
        rumo = gps_medida.curso_deg * 3.14159265358979323846f / 180.0f;
        vel_n = cosf(rumo) * gps_medida.vel_m_s;
        vel_e = sinf(rumo) * gps_medida.vel_m_s;

        primask = __get_PRIMASK();
        __disable_irq();
        euler = estimador_atitude_obtem_euler(&sistema_estimador);
        __set_PRIMASK(primask);
        guinada = euler.z;

        nav_controle_atualiza(&sistema_nav_ctrl, &nav_cmd, guinada,
                              -norte_m, -leste_m, vel_n, vel_e,
                              hold_alt - baro_alt, vel_v,
                              dist_m, gps_medida.vel_m_s);

        setpoint_sombra.atitude.euler.x = nav_cmd.rol_des_rad;
        setpoint_sombra.atitude.euler.y = nav_cmd.arfagem_des_rad;
        setpoint_sombra.taxa.angvel.z = nav_cmd.taxa_guinada_rad_s;
        setpoint_sombra.throttle = nav_cmd.throttle;
        dist_alvo_m = dist_m;
      }
    }
    else
    {
      hold_engajado = 0;
      dist_alvo_m = 0.0f;
    }
    setpoint_pronto = 1;

    modo = estados_atualiza(&sistema_estados, &config_estados, canais,
                            tem_sinal, calibracao_concluida(&sistema_calibracao),
                            armar_permitido, agora);

    if (modo != modo_ant)
    {
      if (modo == ESTADO_VOO)
      {
        /* Borda de subida do VOO: aborta teste de motor (se houver). */
        j3serial_teste_motor_aborta(&sistema_teste_motor);
        /* Trava o home (ponto de decolagem). */
        home_ponto_trava(&sistema_home, &gps_medida);
        controlador_reseta(&sistema_controlador);
        controle_habilitado = 1;
      }
      if (modo == ESTADO_CALIBRACAO)
      {
        calibracao_inicia(&sistema_calibracao, CALIBRACAO_AMOSTRAS);
      }
      if (modo_ant == ESTADO_VOO)
      {
        controle_habilitado = 0;
        hold_engajado = 0;
        /* Destrava o home: volta a procurar melhor candidato. */
        home_ponto_destrava(&sistema_home);
      }
      modo_ant = modo;
    }
    modo_atual = modo;

    /* Telemetria periodica (somente se o host configurou). */
    if (telemetria_periodo_ms > 0 &&
        (int32_t)(agora - proxima_telemetria_ms) >= 0)
    {
      proxima_telemetria_ms = agora + telemetria_periodo_ms;
      j3s_envia_telemetria();
    }

    /* FSM do teste de motor + eventos de fim + sombra para a ISR. */
    j3serial_teste_motor_atualiza(&sistema_teste_motor, agora);
    j3s_checa_fim_teste();
    j3s_teste_sombra_atualiza();

    led_atualiza(modo, tem_sinal, agora);
    gps_led_atualiza(gps_ok, agora);
  }
}
