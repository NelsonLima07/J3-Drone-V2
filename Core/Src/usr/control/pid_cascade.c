/**
 * @file    pid_cascade.c
 * @brief   Controlador em cascata: ângulo -> taxa -> torque
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "control/pid_cascade.h"
#include "math/math_utils.h"

/* Acesso seguro por eixo (0=rol, 1=arfagem, 2=guinada) */
static float componente(vetor3_t v, int eixo)
{
  if (eixo == 0)
    return v.x;
  if (eixo == 1)
    return v.y;
  return v.z;
}

void cascata_config_padrao(config_cascata_t *config)
{
  int i;
  const ganhos_pid_t ganhos_angulo_iniciais = {
      .kp = 6.0f,          /* (rad/s) por rad de erro de atitude */
      .ki = 0.0f,
      .kd = 0.0f,
      .d_fc = 0.0f,
      .limite_integral = 0.5f,
      .limite_saida = 1.0f,
      .peso_setpoint = 1.0f,
  };
  const ganhos_pid_t ganhos_taxa_iniciais = {
      .kp = 0.035f,        /* torque por (rad/s) de erro de taxa */
      .ki = 0.40f,
      .kd = 0.0012f,
      .d_fc = 30.0f,
      .limite_integral = 0.3f,
      .limite_saida = 1.0f,
      .peso_setpoint = 1.0f,
  };

  config->modo = CONTROLADOR_MODO_ANGLE;
  config->dt_angulo = 1.0f / 1000.0f;
  config->dt_taxa = 1.0f / 8000.0f;
  config->limite_comando_taxa = 4.0f; /* rad/s */

  for (i = 0; i < 3; ++i)
  {
    config->ganhos_angulo[i] = ganhos_angulo_iniciais;
    config->ganhos_taxa[i] = ganhos_taxa_iniciais;
  }
}

void controlador_cascata_inicializa(controlador_cascata_t *c, const config_cascata_t *config)
{
  int i;

  c->ops = &ops_cascata;
  c->config = *config;

  c->divisor = (config->dt_taxa > 0.0f)
                   ? (uint32_t)(config->dt_angulo / config->dt_taxa + 0.5f)
                   : 1u;
  if (c->divisor < 1u)
    c->divisor = 1u;
  c->passo = 0u;

  for (i = 0; i < 3; ++i)
  {
    pid_inicializa(&c->pid_angulo[i], &config->ganhos_angulo[i], config->dt_angulo);
    pid_inicializa(&c->pid_taxa[i], &config->ganhos_taxa[i], config->dt_taxa);
    c->comando_taxa[i] = 0.0f;
  }
}

void controlador_cascata_atualiza(controlador_cascata_t *c,
                                  const atitude_t *atitude,
                                  const velocidade_angular_t *taxa,
                                  const setpoint_t *setpoint,
                                  float dt,
                                  saida_controle_t *saida)
{
  int eixo;

  (void)dt;

  if (c->config.modo == CONTROLADOR_MODO_ANGLE)
  {
    c->passo++;
    if (c->passo >= c->divisor)
    {
      c->passo = 0u;
      for (eixo = 0; eixo < 2; ++eixo)
      {
        float alvo = componente(setpoint->atitude.euler, eixo);
        float atual = componente(atitude->euler, eixo);
        float comando = pid_atualiza(&c->pid_angulo[eixo], alvo, atual);

        c->comando_taxa[eixo] = limita(comando,
                                       -c->config.limite_comando_taxa,
                                       c->config.limite_comando_taxa);
      }
    }
    /* Guinada: comando por taxa (nao se auto-nivela; sem magnetometro). */
    c->comando_taxa[2] = limita(componente(setpoint->taxa.angvel, 2),
                                -c->config.limite_comando_taxa,
                                c->config.limite_comando_taxa);
  }
  else
  {
    for (eixo = 0; eixo < 3; ++eixo)
    {
      c->comando_taxa[eixo] = limita(componente(setpoint->taxa.angvel, eixo),
                                     -c->config.limite_comando_taxa,
                                     c->config.limite_comando_taxa);
    }
  }

  for (eixo = 0; eixo < 3; ++eixo)
  {
    float atual = componente(taxa->angvel, eixo);
    float comando = pid_atualiza(&c->pid_taxa[eixo], c->comando_taxa[eixo], atual);

    if (eixo == 0)
      saida->torque.x = comando;
    else if (eixo == 1)
      saida->torque.y = comando;
    else
      saida->torque.z = comando;
  }

  saida->thrust = setpoint->throttle;
}

void controlador_cascata_reseta(controlador_cascata_t *c)
{
  int i;

  for (i = 0; i < 3; ++i)
  {
    pid_reseta(&c->pid_angulo[i]);
    pid_reseta(&c->pid_taxa[i]);
    c->comando_taxa[i] = 0.0f;
  }
  c->passo = 0u;
}

/* Implementação concreta da interface */
static void cascata_ops_inicializa(void *self, const void *config)
{
  controlador_cascata_inicializa((controlador_cascata_t *)self, (const config_cascata_t *)config);
}

static void cascata_ops_atualiza(void *self,
                                 const atitude_t *atitude,
                                 const velocidade_angular_t *taxa,
                                 const setpoint_t *setpoint,
                                 float dt,
                                 saida_controle_t *saida)
{
  controlador_cascata_atualiza((controlador_cascata_t *)self, atitude, taxa, setpoint, dt, saida);
}

static void cascata_ops_reseta(void *self)
{
  controlador_cascata_reseta((controlador_cascata_t *)self);
}

const ops_controlador_t ops_cascata = {
    .nome = "cascata",
    .inicializa = cascata_ops_inicializa,
    .atualiza = cascata_ops_atualiza,
    .reseta = cascata_ops_reseta,
};
