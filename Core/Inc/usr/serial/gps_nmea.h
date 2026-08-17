/**
 * @file    gps_nmea.h
 * @brief   Decodificador NMEA (GGA/RMC) em C puro para o GPS BN-220.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Nao depende do HAL: recebe bytes do transporte (USART3 + DMA no
 * gps_uart_hal) e entrega uma medida GPS. Parser em streaming: aceita
 * a sentenca fatiada em qualquer tamanho de buffer.
 *
 * E fonte do "erro estimado" horizontal usado pelo home point:
 *   erro_estimado_m = hdop * uere  (uere configuravel, padrao 5 m).
 */

#ifndef USR_SERIAL_GPS_NMEA_H
#define USR_SERIAL_GPS_NMEA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_NMEA_LINHA_MAX 96u  /**< tamanho maximo de uma sentenca NMEA */

/**
 * @brief  Medida GPS decodificada.
 */
typedef struct {
  uint8_t  fix_valido;      /**< qualidade >= 1 (qualquer fix 3D)   */
  uint8_t  num_sats;        /**< satelites usados no fix            */
  float    qualidade;       /**< GGA quality: 0=inv, 1=GPS, 2=DGPS  */
  float    lat_deg;         /**< latitude (graus, -90..+90)          */
  float    lon_deg;         /**< longitude (graus, -180..+180)       */
  float    alt_m;           /**< altitude MSL (m, do GGA)            */
  float    vel_m_s;         /**< velocidade solo (m/s, do RMC)       */
  float    curso_deg;       /**< rumo sobre o solo (0..360, do RMC)  */
  float    hdop;            /**< HDOP do GGA (0..99)                 */
  float    erro_estimado_m; /**< = hdop * uere (erro horizontal est.)*/
  uint32_t versao;          /**< incrementa a cada atualizacao       */
} gps_medida_t;

typedef struct {
  float uere;               /**< erro unitario do receptor (m)      */
} config_gps_t;

typedef struct {
  config_gps_t config;
  gps_medida_t medida;
  uint8_t linha[GPS_NMEA_LINHA_MAX];
  uint16_t linha_ix;
  uint8_t  estado;          /* maquina do parser                    */
  uint8_t  checksum_calc;
  uint8_t  checksum_recebido;
  uint8_t  checksum_ix;
} gps_parser_t;

/** Configuracao padrao: UERE 5 m. */
void gps_config_padrao(config_gps_t *cfg);

/**
 * @brief  Inicializa o parser (zera medida e estado).
 */
void gps_nmea_inicializa(gps_parser_t *p, const config_gps_t *cfg);

/**
 * @brief  Alimenta o parser com um byte do stream NMEA.
 * @return 1 se uma sentenca valida terminou (medida pode ter mudado);
 *         0 caso contrario.
 */
uint8_t gps_nmea_processa_byte(gps_parser_t *p, uint8_t byte);

/**
 * @brief  Conveniencia: alimenta um buffer inteiro.
 * @return numero de sentencas validas terminadas.
 */
uint32_t gps_nmea_processa_buffer(gps_parser_t *p, const uint8_t *dados,
                                 uint32_t n);

/** Medida atual (copia). */
gps_medida_t gps_nmea_medida(const gps_parser_t *p);

#ifdef __cplusplus
}
#endif

#endif /* USR_SERIAL_GPS_NMEA_H */
