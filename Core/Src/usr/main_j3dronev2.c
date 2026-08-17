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
#include "sensors/bmp280.h"
#include "sensors/gps_uart_hal.h"
#include "sensors/i2c2_hal.h"
#include "system/hardware_glue.h"
#include "system/estados.h"
#include "system/calibracao.h"
#include "system/gps_led.h"
#include "serial/ibus_uart_hal.h"
#include "serial/radio_comandos.h"
#include "serial/gps_nmea.h"
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

/* Mag (LIS3MDL) e baro (BMP280) na I2C2. mag_sombra e consumido na ISR. */
static lis3mdl_t sistema_mag;
static bmp280_t sistema_baro;
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

  if (!controle_habilitado)
  {
    /* Desarmado: mantem os frames zero nos 4 ESCs (o DMA re-arma sozinho). */
    dshot_escreve(&saida_misturada, 0u);
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

  /* Mag (LIS3MDL, 0x1C) e baro (BMP280, 0x76) na I2C2 (polling). */
  (void)i2c2_hal_vincula(&transporte_mag, LIS3MDL_ENDERECO);
  (void)i2c2_hal_vincula(&transporte_baro, BMP280_ENDERECO);
  mag_ok = 0;
  baro_ok = 0;
  if (lis3mdl_inicializa(&sistema_mag, &transporte_mag) == 0 &&
      lis3mdl_verifica_id(&sistema_mag) &&
      lis3mdl_configura(&sistema_mag) == 0)
  {
    mag_ok = 1;
  }
  if (bmp280_inicializa(&sistema_baro, &transporte_baro) == 0 &&
      bmp280_verifica_id(&sistema_baro) &&
      bmp280_configura(&sistema_baro) == 0)
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
      if (baro_ok && bmp280_processa(&sistema_baro))
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

    /* Position hold: engaja armado + CH6 + GPS/mag/baro ok. */
    if (controle_habilitado && ch6_on && gps_fix_ok && mag_ok && baro_ok)
    {
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
      }
    }
    else
    {
      hold_engajado = 0;
    }
    setpoint_pronto = 1;

    modo = estados_atualiza(&sistema_estados, &config_estados, canais,
                            tem_sinal, calibracao_concluida(&sistema_calibracao),
                            armar_permitido, agora);

    if (modo != modo_ant)
    {
      if (modo == ESTADO_VOO)
      {
        /* Borda de subida do VOO: trava o home (ponto de decolagem). */
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

    led_atualiza(modo, tem_sinal, agora);
    gps_led_atualiza(gps_ok, agora);
  }
}
