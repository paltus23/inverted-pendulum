/**
 * @file as5048a.h
 * @author paltusoff@gmail.com
 * @brief  A library for AMS AS5048A rotary position sensor/magnetic encoder.
 *        Just a remake from AS5047P library.
 * @copyright MIT License, Copyright (c) 2026
 * @remark AS5048A SPI Interface:
 *         - Mode=1(CPOL=0, CPHA=1).
 *             - CPOL=0 --> Clock is low when idle.
 *             - CPHA=1 --> Data is sampled on the second edge(falling edge).
 *         - CSn(chip select) active low.
 *         - Data size=16-bit.
 *         - Bit order is MSB first.
 *         - Max clock rates up to 10 MHz.
 *         - Only supports slave operation mode.
 *
 */

#ifndef AS5048A_H
#define AS5048A_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

  typedef enum
  {
    AS5018A_OK,
    AS5018A_ERROR,
  } as5048a_err_t;

  typedef as5048a_err_t (*as5048a_spi_send_t)(uint8_t *data, uint32_t len);
  typedef as5048a_err_t (*as5048a_spi_read_t)(uint8_t *tx_data, uint8_t *rx_data, uint32_t len);
  typedef as5048a_err_t (*as5048a_spi_select_t)(void);
  typedef as5048a_err_t (*as5048a_spi_deselect_t)(void);

  /**
   * @brief t_CSn: High time of CSn between two transmissions, Min: 350 ns.
   */
  typedef void (*as5048a_delay_t)(void);

  typedef struct
  {
    as5048a_spi_send_t spi_send;
    as5048a_spi_read_t spi_read;
    as5048a_spi_select_t spi_select;
    as5048a_spi_deselect_t spi_deselect;
    as5048a_delay_t delay;
  } as5048a_handle_t;

  as5048a_err_t as5048a_make_handle(as5048a_spi_send_t spi_send_func,
                                    as5048a_spi_read_t spi_read_func,
                                    as5048a_spi_select_t spi_select_func,
                                    as5048a_spi_deselect_t spi_deselect_func,
                                    as5048a_delay_t delay_func,
                                    as5048a_handle_t *as5048a_handle);

  void as5048a_set_zero(const as5048a_handle_t *as5048a_handle, uint16_t position);

  as5048a_err_t as5048a_get_position(const as5048a_handle_t *as5048a_handle, uint16_t *position);

  as5048a_err_t as5048a_get_angle(const as5048a_handle_t *as5048a_handle, float *angle_degree);

  uint16_t as5048a_get_error_status(const as5048a_handle_t *as5048a_handle);

  as5048a_err_t as5048a_get_diag(const as5048a_handle_t *as5048a_handle, uint16_t *diag);

#ifdef __cplusplus
}
#endif

#endif /* AS5048A_H */