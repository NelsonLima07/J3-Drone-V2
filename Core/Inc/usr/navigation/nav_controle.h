/**
 * @file    nav_controle.h
 * @brief   Controlador de navegacao (posicao/altura/rumo) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Converte erro de posicao (Norte/Leste em metros), erro de altura e
 * erro de rumo em um comando de atitude (rol/arfagem), taxa de
 * guinada e throttle, pronto para ser escrito no setpoint em modo
 * ANGLE. Puramente proporcional + amortecimento de velocidade.
 */

#ifndef USR_NAVIGATION_NAV_CONTROLE_H
#define USR_NAVIGATION_NAV_CONTROLE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp_posicao;     /**< (rad de inclinacao) / metro de erro  */
  float kd_velocidade;  /**< (rad) / (m/s) - amortecimento        */
  float kp_guinada;     /**< (rad/s) / rad de erro de rumo        */
  float max_angulo;     /**< limite de inclinacao (rad)           */
  float kp_altura;      /**< throttle / metro de erro de altura   */
  float kd_altura;      /**< throttle / (m/s) - amortecimento     */
  float thrust_hover;   /**< throttle de flutuacao (0..1)         */
  float raio_chegada;   /**< m: dentro disto pode segurar o rumo    */
  float vel_chegada;    /**< m/s: abaixo disto pode segurar o rumo  */
  float alinhar_rad;    /**< rad: congelar a guinada so quando o     */
                        /*   erro de rumo estiver abaixo disto      */
} config_nav_controle_t;

typedef struct {
  float rol_des_rad;        /**< atitude desejada (eixo X)         */
  float arfagem_des_rad;    /**< atitude desejada (eixo Y)         */
  float taxa_guinada_rad_s; /**< taxa de guinada desejada (eixo Z) */
  float throttle;           /**< 0..1                              */
} nav_comando_t;

typedef struct {
  config_nav_controle_t config;
} nav_controle_t;

/** Configuracao padrao do position hold (ajuste em campo). */
void config_nav_controle_padrao(config_nav_controle_t *cfg);

void nav_controle_inicializa(nav_controle_t *nav,
                             const config_nav_controle_t *cfg);

/**
 * @brief  Computa o comando de navegacao.
 * @param  guinada_rad     guinada atual (rad, -PI..PI)
 * @param  erro_norte_m    alvo - atual (Norte positivo, m)
 * @param  erro_leste_m    alvo - atual (Leste positivo, m)
 * @param  vel_norte_m_s   velocidade NED (Norte positivo)
 * @param  vel_leste_m_s   velocidade NED (Leste positivo)
 * @param  alt_erro_m      alvo - atual (positivo = subir)
 * @param  vel_v_m_s       velocidade vertical (positivo = subir)
 * @param  dist_m          distancia horizontal ate o alvo
 * @param  vel_solo_m_s    velocidade horizontal
 */
void nav_controle_atualiza(nav_controle_t *nav, nav_comando_t *cmd,
                           float guinada_rad,
                           float erro_norte_m, float erro_leste_m,
                           float vel_norte_m_s, float vel_leste_m_s,
                           float alt_erro_m, float vel_v_m_s,
                           float dist_m, float vel_solo_m_s);

/** Envolve angulo para [-PI, PI). */
float nav_controle_envolve_pi(float angulo);

#ifdef __cplusplus
}
#endif

#endif /* USR_NAVIGATION_NAV_CONTROLE_H */
