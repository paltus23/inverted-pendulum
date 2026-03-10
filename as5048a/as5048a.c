/**
 * @file as5048a.c
 * @author paltusoff@gmail.com
 * @brief  A library for AMS AS5048A rotary position sensor/magnetic encoder.
 *        Just a remake from AS5047P library.
 * @copyright MIT License, Copyright (c) 2026
 *
 */

#include "as5048a.h"

#define BIT_MODITY(src, i, val) ((src) ^= (-(val) ^ (src)) & (1UL << (i)))
#define BIT_READ(src, i) (((src) >> (i) & 1U))
#define BIT_TOGGLE(src, i) ((src) ^= 1UL << (i))

/* Volatile register address. */
#define AS5048A_NOP ((uint16_t)0x0000)
#define AS5048A_ERRFL ((uint16_t)0x0001)
#define AS5048A_PROG ((uint16_t)0x0003)
#define AS5048A_DIAAGC ((uint16_t)0x3FFD)
#define AS5048A_MAG ((uint16_t)0x3FFE)
#define AS5048A_ANGLE ((uint16_t)0x3FFF)

/* Non-Volatile register address. */
#define AS5048A_ZPOSM ((uint16_t)0x0016)
#define AS5048A_ZPOSL ((uint16_t)0x0017)

#define OP_WRITE ((uint8_t)0)
#define OP_READ ((uint8_t)1)

static void send_command(const as5048a_handle_t *as5048a_handle, uint16_t address, uint8_t op_read_write);
static void send_data(const as5048a_handle_t *as5048a_handle, uint16_t address, uint16_t data);
static uint16_t read_data(const as5048a_handle_t *as5048a_handle, uint16_t address);

void spi_transmit(const as5048a_handle_t *as5048a_handle, uint16_t data);
uint16_t spi_receive(const as5048a_handle_t *as5048a_handle, uint16_t frame);

void as5048a_nop(const as5048a_handle_t *as5048a_handle);
void delay(volatile uint16_t t);
uint8_t is_even_parity(uint16_t data);

/**
 * @brief Make a AS5048A handle.
 *
 * @param[in] spi_send_func
 * @param[in] spi_read_func
 * @param[in] spi_select_func
 * @param[in] spi_deselect_func
 * @param[in] delay_func The function that delay 350ns for t_CSn.
 * @param[out] as5048a_handle AS5048A handle.
 * @return Status code.
 *         0: Success.
 */
as5048a_err_t as5048a_make_handle(as5048a_spi_send_t spi_send_func,
                                  as5048a_spi_read_t spi_read_func,
                                  as5048a_spi_deselect_t spi_select_func,
                                  as5048a_spi_deselect_t spi_deselect_func,
                                  as5048a_delay_t delay_func,
                                  as5048a_handle_t *as5048a_handle)
{
  as5048a_handle->spi_send = spi_send_func;
  as5048a_handle->spi_read = spi_read_func;
  as5048a_handle->spi_select = spi_select_func;
  as5048a_handle->spi_deselect = spi_deselect_func;
  as5048a_handle->delay = delay_func;
  return AS5018A_OK; /* Success. */
}

/**
 * @brief Reading error flags.
 *
 * @param as5048a_handle AS5048A handle.
 * @return Error flags. 0 for no error occurred.
 */
uint16_t as5048a_get_error_status(const as5048a_handle_t *as5048a_handle)
{
  return read_data(as5048a_handle, AS5048A_ERRFL);
}

/**
 * @brief Read current position.
 *
 * @param as5048a_handle AS5048A handle.
 * @param position Current position raw value.
 * @return Status code.
 *         AS5018A_OK: Success.
 *         AS5018A_ERROR: Error occurred.
 */
as5048a_err_t as5048a_get_position(const as5048a_handle_t *as5048a_handle,
                                   uint16_t *position)
{
  uint16_t data = read_data(as5048a_handle, AS5048A_ANGLE);
  *position = data & 0x3FFF;
  if (BIT_READ(data, 14) == 0)
  {
    return AS5018A_OK; /* No error occurred. */
  }
  return AS5018A_ERROR; /* Error occurred. */
}

as5048a_err_t as5048a_get_diag(const as5048a_handle_t *as5048a_handle,
                               uint16_t *diag)
{
  uint16_t data = read_data(as5048a_handle, AS5048A_DIAAGC);
  if (BIT_READ(data, 14) == 0)
  {
    *diag = data;
    return AS5018A_OK; /* No error occurred. */
  }
  return AS5018A_ERROR; /* Error occurred. */
}

/**
 * @brief Read current angle in degree.
 *
 * @param as5048a_handle AS5048A handle
 * @param angle_degree Current angle in degree.
 * @return Status code.
 *         AS5018A_OK: Success.
 *         AS5018A_ERROR: Error occurred.
 */
as5048a_err_t as5048a_get_angle(const as5048a_handle_t *as5048a_handle, float *angle_degree)
{
  uint16_t raw_position;
  as5048a_err_t error = as5048a_get_position(as5048a_handle, &raw_position);
  if (error == AS5018A_OK)
  {
    /* Angle in degree = value * ( 360 / 2^14). */
    *angle_degree = raw_position * (360.0 / 0x4000);
  }

  return error;
}

/**
 * @brief Set specify position as zero.
 *
 * @param as5048a_handle AS5048A handle.
 * @param position Position raw value.
 */
void as5048a_set_zero(const as5048a_handle_t *as5048a_handle, uint16_t position)
{
  /* 8 most significant bits of the zero position. */
  send_data(as5048a_handle, AS5048A_ZPOSM, ((position >> 6) & 0x00FF));

  /* 6 least significant bits of the zero position. */
  send_data(as5048a_handle, AS5048A_ZPOSL, (position & 0x003F));

  as5048a_nop(as5048a_handle);
}

/**
 * @brief No operation instruction.
 *
 * @param as5048a_handle AS5048A handle.
 */
inline void as5048a_nop(const as5048a_handle_t *as5048a_handle)
{
  /* Reading the NOP register is equivalent to a nop (no operation) instruction. */
  send_command(as5048a_handle, AS5048A_NOP, OP_READ);
}

/**
 * @brief Sending read or write command to AS5048A.
 *
 * @param as5048a_handle AS5048A handle.
 * @param address Register address.
 * @param op_read_write Read of write opration.
 */
static void send_command(const as5048a_handle_t *as5048a_handle, uint16_t address, uint8_t op_read_write)
{
  uint16_t frame = address & 0x3FFF;

  /* R/W: 0 for write, 1 for read. */
  BIT_MODITY(frame, 14, op_read_write);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  spi_transmit(as5048a_handle, frame);
}

/**
 * @brief Sending data to register.
 *
 * @param as5048a_handle AS5048A handle.
 * @param address Register address.
 * @param data Data.
 */
static void send_data(const as5048a_handle_t *as5048a_handle, uint16_t address, uint16_t data)
{
  uint16_t frame = data & 0x3FFF;

  /* Data frame bit 14 always low(0). */
  BIT_MODITY(frame, 14, 0);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  send_command(as5048a_handle, address, OP_WRITE);
  spi_transmit(as5048a_handle, frame);
}

/**
 * @brief Reading data from register.
 *
 * @param as5048a_handle AS5048A handle.
 * @param address Register address.
 * @return Data.
 */
static uint16_t read_data(const as5048a_handle_t *as5048a_handle, uint16_t address)
{
  send_command(as5048a_handle, address, OP_READ);
  return spi_receive(as5048a_handle, 0);
}

/**
 * @brief Start SPI transmit.
 *
 * @param as5048a_handle AS5048A handle.
 * @param data Data.
 */
inline void spi_transmit(const as5048a_handle_t *as5048a_handle, uint16_t data)
{
  uint8_t buf[2] = {data >> 8, data & 0xFF};

  as5048a_handle->delay();
  as5048a_handle->spi_select();
  as5048a_handle->spi_send(buf, 2);
  as5048a_handle->spi_deselect();
}

/**
 * @brief Start SPI receive.
 *
 * @param as5048a_handle AS5048A handle.
 * @return Received data.
 */
inline uint16_t spi_receive(const as5048a_handle_t *as5048a_handle, uint16_t frame)
{
  uint8_t rx_buf[2] = {0, 0};
  uint8_t tx_buf[2] = {frame >> 8, frame & 0xFF};
  as5048a_handle->delay();
  as5048a_handle->spi_select();
  as5048a_handle->spi_read((void *)0, rx_buf, 2);
  as5048a_handle->spi_deselect();
  return (rx_buf[0] << 8) + rx_buf[1];
}

/**
 * @brief Check data even parity.
 */
uint8_t is_even_parity(uint16_t data)
{
  uint8_t shift = 1;
  while (shift < (sizeof(data) * 8))
  {
    data ^= (data >> shift);
    shift <<= 1;
  }
  return !(data & 0x1);
}