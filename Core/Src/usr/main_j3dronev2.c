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
#include "system/hardware_glue.h"
#include "system/estados.h"
#include "system/calibracao.h"
#include "serial/ibus_uart_hal.h"
#include "serial/radio_comandos.h"
#include "esc/dshot_timer_hal.h"

/* Periodo do loop de controle = ODR do gyro (8 kHz) */
#define DT_LOOP_TAXA 0.000125f /* 1/8000 s */

/* Calibracao: 2 s de amostras a 8 kHz */
#define CALIBRACAO_AMOSTRAS 16000U

/* LED de status (PB5, ativo alto) */
#define LED_ESPERA_PERIODO_MS  200U  /* 5 Hz  */
#define LED_CALIBRACAO_PERIODO_MS 1000U /* 1 Hz */

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

  modo = estados_atualiza(&sistema_estados, &config_estados, 0, 0, 0, HAL_GetTick());
  modo_ant = modo;

  while (1)
  {
    agora = HAL_GetTick();

    tem_sinal = (uint8_t)ibus_uart_tem_sinal(agora);
    (void)ibus_uart_canais(canais);

    /* Sticks -> setpoint (shadow lido pela ISR do controle). */
    radio_comandos_processa(&config_radio, canais, controle_habilitado,
                            &setpoint_sombra);
    setpoint_pronto = 1;

    modo = estados_atualiza(&sistema_estados, &config_estados, canais,
                            tem_sinal, calibracao_concluida(&sistema_calibracao),
                            agora);

    if (modo != modo_ant)
    {
      if (modo == ESTADO_VOO)
      {
        /* Borda de subida do VOO: zera o controlador e habilita o controle. */
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
      }
      modo_ant = modo;
    }

    led_atualiza(modo, tem_sinal, agora);
  }
}
