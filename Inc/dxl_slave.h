/**
 * @file    dxl_slave.h
 * @brief   Dynamixel Protocol 2.0 slave on the servo bus.
 *
 * The Radxa ZERO 3W never talks to the LSM6DSV16X. It opens the Dynamixel bus
 * (/dev/ttyS2, 1 Mbps) and issues one SYNC_READ per control tick covering this
 * board and all fifteen XL330 servos, at address 124 for 12 bytes - the same
 * address the servos serve present_pwm / present_current / present_velocity /
 * present_position from, which is why the whole thing fits in one transaction.
 *
 * This board is listed first in that ID list, so it answers before the servo
 * burst. See microduck/duck-control/src/bus.rs and .../imu.rs for the host end.
 *
 * The block served at 124 is 20 bytes; the tick consumes the first 12.
 *
 * | offset | bytes | contents                                              |
 * |--------|-------|-------------------------------------------------------|
 * | 0      | 6     | gyro x/y/z, i16 LE raw counts, +/-500 dps             |
 * | 6      | 6     | SFLP game rotation vector x/y/z, IEEE binary16 LE     |
 * | 12     | 6     | accelerometer x/y/z, i16 LE raw counts, +/-4 g        |
 * | 18     | 1     | sample counter, wraps at 256                          |
 * | 19     | 1     | status flags, see DXL_STATUS_*                        |
 *
 * w of the quaternion is not sent: the host reconstructs it as
 * sqrt(1 - x^2 - y^2 - z^2), and reads an all-zero x/y/z as "SFLP has not
 * written its table yet" rather than as identity.
 */

#ifndef DXL_SLAVE_H
#define DXL_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

/** This board's Dynamixel ID. Must match IMU_DXL_ID in duck-control. */
#define DXL_ID 200u

/** Bus rate. Must match BAUD_RATE in duck-control. */
#define DXL_BAUD 1000000u

#define DXL_BROADCAST_ID 0xFEu

/** Reported by PING. Arbitrary - no real Dynamixel model shares it. */
#define DXL_MODEL_NUMBER 0xC011u

/** Bumped when the layout of the block at 124 changes. */
#define DXL_FIRMWARE_VERSION 6u

/** Start and length of the sensor block. */
#define DXL_SENSOR_ADDR 124u
#define DXL_SENSOR_LEN  20u

/** Size of the flat register file the slave serves. */
#define DXL_TABLE_SIZE (DXL_SENSOR_ADDR + DXL_SENSOR_LEN)

/* Status flags, byte 19 of the sensor block. */
#define DXL_STATUS_IMU_PRESENT  0x01u /* WHO_AM_I matched at boot            */
#define DXL_STATUS_SFLP_READY   0x02u /* at least one quaternion has arrived */
#define DXL_STATUS_FIFO_OVERRUN 0x04u /* FIFO overran since the last publish */
#define DXL_STATUS_SPI_ERROR    0x08u /* an SPI transfer timed out           */

/**
 * Bring USART1 up on the bus and start serving. How the line is shared is the
 * board's business - see DXL_BUFFERED in board.h and line_drive() in
 * dxl_slave.c.
 *
 * From here the slave runs entirely out of the USART1 interrupt; there is no
 * poll function to call.
 */
void dxl_slave_init(void);

/**
 * Publish a fresh sensor block.
 *
 * @param block  DXL_SENSOR_LEN bytes, laid out as documented above. Copied
 *               under a critical section so a SYNC_READ can never answer with
 *               half of one sample and half of the next.
 */
void dxl_slave_publish(const uint8_t *block);

/** Status packets sent since boot. Handy on a scope-less bench. */
uint32_t dxl_slave_replies(void);

#endif /* DXL_SLAVE_H */
