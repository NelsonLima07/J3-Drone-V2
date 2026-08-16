/**
 * @file    test_control.c
 * @brief   Testes nativos da biblioteca de controle
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "math/math_types.h"
#include "control/control_types.h"
#include "control/pid.h"
#include "control/pid_cascade.h"
#include "control/mixer.h"

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

int testes_control(void)
{
  /* ---------- PID: resposta ao degrau ---------- */
  {
    ganhos_pid_t ganhos = {.kp = 1.0f, .ki = 0.1f, .kd = 0.0f, .d_fc = 0.0f,
                           .limite_integral = 0.5f, .limite_saida = 1.0f,
                           .peso_setpoint = 1.0f};
    controlador_pid_t pid;
    float saida = 0.0f;
    float valor_abs = 0.0f;
    int k;
    int estourou = 0;

    pid_inicializa(&pid, &ganhos, 0.001f);
    for (k = 0; k < 2000; ++k)
    {
      saida = pid_atualiza(&pid, 1.0f, 0.0f);
      if (saida > 1.0f + 1.0e-6f || saida < -1.0e-6f)
        estourou = 1;
    }
    valor_abs = fabsf(saida - 1.0f);
    VERIFICA(valor_abs < 0.05f, "PID assenta perto do setpoint");
    VERIFICA(!estourou, "PID respeita limite_saida");
  }

  /* ---------- PID: anti-windup ---------- */
  {
    ganhos_pid_t ganhos = {.kp = 0.1f, .ki = 2.0f, .kd = 0.0f, .d_fc = 0.0f,
                           .limite_integral = 0.5f, .limite_saida = 1.0f,
                           .peso_setpoint = 1.0f};
    controlador_pid_t pid;
    float saida = 0.0f;
    int k;

    pid_inicializa(&pid, &ganhos, 0.001f);
    for (k = 0; k < 100000; ++k)
      saida = pid_atualiza(&pid, 10.0f, 0.0f);

    VERIFICA(APROX(saida, ganhos.limite_saida, 1.0e-6f), "PID saturado na saida");
    VERIFICA(pid_obtem_integral(&pid) <= ganhos.limite_integral + 1.0e-6f,
             "integrador nao estoura o limite (anti-windup)");
  }

  /* ---------- PID: reset ---------- */
  {
    ganhos_pid_t ganhos = {.kp = 1.0f, .ki = 1.0f, .kd = 0.0f, .d_fc = 0.0f,
                           .limite_integral = 1.0f, .limite_saida = 1.0f,
                           .peso_setpoint = 1.0f};
    controlador_pid_t pid;
    pid_inicializa(&pid, &ganhos, 0.001f);
    pid_atualiza(&pid, 1.0f, 0.0f);
    pid_reseta(&pid);
    VERIFICA(pid_obtem_integral(&pid) == 0.0f, "pid_reseta zera o integrador");
  }

  /* ---------- Cascata: hover em ACRO com comandos nulos ---------- */
  {
    config_cascata_t config;
    controlador_cascata_t cascata;
    atitude_t atitude = {{0.0f, 0.0f, 0.0f}};
    velocidade_angular_t taxa = {{0.0f, 0.0f, 0.0f}};
    setpoint_t setpoint = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f}}, 0.5f};
    saida_controle_t saida;
    int k;

    cascata_config_padrao(&config);
    config.modo = CONTROLADOR_MODO_ACRO;
    controlador_cascata_inicializa(&cascata, &config);

    for (k = 0; k < 100; ++k)
      controlador_cascata_atualiza(&cascata, &atitude, &taxa, &setpoint, config.dt_taxa, &saida);

    VERIFICA(fabsf(saida.torque.x) < 1.0e-3f &&
             fabsf(saida.torque.y) < 1.0e-3f &&
             fabsf(saida.torque.z) < 1.0e-3f, "ACRO sem comando -> torques nulos");
    VERIFICA(APROX(saida.thrust, 0.5f, 1.0e-6f), "ACRO repassa throttle");
  }

  /* ---------- Cascata: modo ANGLE corrige o rol ---------- */
  {
    config_cascata_t config;
    controlador_cascata_t cascata;
    atitude_t atitude = {{0.0f, 0.0f, 0.0f}};
    velocidade_angular_t taxa = {{0.0f, 0.0f, 0.0f}};
    setpoint_t setpoint = {{{0.2f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f}}, 0.5f};
    saida_controle_t saida;
    int k;

    cascata_config_padrao(&config);
    config.modo = CONTROLADOR_MODO_ANGLE;
    controlador_cascata_inicializa(&cascata, &config);

    for (k = 0; k < 200; ++k)
      controlador_cascata_atualiza(&cascata, &atitude, &taxa, &setpoint, config.dt_taxa, &saida);

    VERIFICA(saida.torque.x > 0.0f, "ANGLE gera torque positivo para subir o rol");
    VERIFICA(APROX(saida.thrust, 0.5f, 1.0e-6f), "ANGLE repassa throttle");
  }

  /* ---------- Mixer ---------- */
  {
    saida_misturador_t s;
    vetor3_t torque_zero = {0.0f, 0.0f, 0.0f};
    vetor3_t torque_rol = {0.2f, 0.0f, 0.0f};
    vetor3_t torque_arf = {0.0f, 0.3f, 0.0f};
    int i;

    misturador_aplica(0.5f, &torque_zero, &s);
    VERIFICA(APROX(s.motores[0], 0.5f, 1.0e-6f) && APROX(s.motores[1], 0.5f, 1.0e-6f) &&
             APROX(s.motores[2], 0.5f, 1.0e-6f) && APROX(s.motores[3], 0.5f, 1.0e-6f),
             "mixer sem torque -> todos iguais ao thrust");

    misturador_aplica(0.5f, &torque_rol, &s);
    VERIFICA(s.motores[0] > s.motores[2], "rol positivo sobe lado direito (m1)");
    VERIFICA(s.motores[1] > s.motores[3], "rol positivo sobe lado direito (m2)");
    for (i = 0; i < 4; ++i)
      VERIFICA(s.motores[i] >= 0.0f && s.motores[i] <= 1.0f, "mixer normalizado em [0,1]");

    misturador_aplica(0.2f, &torque_arf, &s);
    VERIFICA(s.motores[0] > s.motores[1], "arfagem positiva sobe frente (m1)");
    VERIFICA(s.motores[2] > s.motores[3], "arfagem positiva sobe frente (m3)");
  }

  /* ---------- Interface (vtable) ---------- */
  {
    config_cascata_t config;
    controlador_cascata_t cascata;
    controlador_t ctrl;
    atitude_t atitude = {{0.0f, 0.0f, 0.0f}};
    velocidade_angular_t taxa = {{0.0f, 0.0f, 0.0f}};
    setpoint_t setpoint = {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 0.0f}}, 0.4f};
    saida_controle_t saida;

    cascata_config_padrao(&config);
    controlador_inicializa(&ctrl, &ops_cascata, &cascata, &config);
    controlador_atualiza(&ctrl, &atitude, &taxa, &setpoint, config.dt_taxa, &saida);
    controlador_reseta(&ctrl);

    VERIFICA(strcmp(controlador_nome(&ctrl), "cascata") == 0, "controlador_nome = cascata");
    VERIFICA(APROX(saida.thrust, 0.4f, 1.0e-6f), "interface atualiza via vtable");
  }

  return falhas;
}
