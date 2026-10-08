/**
 * @file    ft_slave.h
 * @brief   Feetech FT-SCS slave on the servo bus.
 *
 * The Radxa ZERO 3W never talks to the LSM6DSV16X. It opens the servo bus
 * (/dev/ttyS2, 500 kbps) and issues one SYNC_READ per control tick covering
 * this board and all fifteen HD1910 servos, at address 56 - the same address an
 * HD1910 serves present_position / present_speed / present_load / voltage /
 * temperature from, which is why the whole thing fits in one transaction.
 *
 * This board is listed first in that ID list, so it answers before the servo
 * burst.
 *
 * The block served at 56 is 20 bytes; the tick consumes the first 12.
 *
 * | offset | bytes | contents                                              |
 * |--------|-------|-------------------------------------------------------|
 * | 0      | 6     | gyro x/y/z, i16 LE raw counts, +/-500 dps             |
 * | 6      | 6     | SFLP game rotation vector x/y/z, IEEE binary16 LE     |
 * | 12     | 6     | accelerometer x/y/z, i16 LE raw counts, +/-4 g        |
 * | 18     | 1     | sample counter, wraps at 256                          |
 * | 19     | 1     | status flags, see FT_STATUS_*                         |
 *
 * w of the quaternion is not sent: the host reconstructs it as
 * sqrt(1 - x^2 - y^2 - z^2), and reads an all-zero x/y/z as "SFLP has not
 * written its table yet" rather than as identity.
 *
 * ## This is not a servo control table
 *
 * Addresses 0 to 8 are the real FT-SCS header - version, ID, baud rate,
 * return delay, response status level - because that is what a bus scan reads
 * and it costs nine bytes to be scannable. Everything above that is this
 * board's own, and an address that means `min_angle_limit` or `present_speed`
 * on an HD1910 means whatever the table below says it means here. The one
 * thing that *is* deliberately aligned is the feedback block at 56, so the
 * IMU can ride the servos' per-tick SYNC_READ.
 *
 * | addr | bytes | contents                                                |
 * |------|-------|---------------------------------------------------------|
 * | 0    | 1     | firmware major version                                  |
 * | 1    | 1     | firmware minor version                                  |
 * | 2    | 1     | END = 0, i.e. little-endian, as every FT-SCS device     |
 * | 3    | 2     | model, u16 LE - what `scan --protocol feetech` reads    |
 * | 5    | 1     | ID (200)                                                |
 * | 6    | 1     | baud rate code (1 = 500 kbps, the FT-SCS numbering)     |
 * | 7    | 1     | return delay (0)                                        |
 * | 8    | 1     | response status level (1 = answer every instruction)    |
 * | 16   | 22    | diagnostic counters, 11 x u16 LE - see ft_slave.c       |
 * | 56   | 20    | the sensor block above                                  |
 *
 * Gaps read as zero, as does anything past the end of the table: a read is
 * always answered with exactly the number of bytes it asked for, so a host
 * that reads a register this board does not have cannot desynchronise the bus
 * for the devices queued behind it.
 */

#ifndef FT_SLAVE_H
#define FT_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

/** This board's ID on the servo bus. FT-SCS allows 0..253. */
#define FT_ID 200u

/**
 * Bus rate. Must match what the host opens the port at and what every servo on
 * the wire is set to.
 *
 * 500 kbps, not the HD1910's 1 Mbps factory default - the servos have to be
 * told the same thing (address 6, the EEPROM baud rate register, code 1). At
 * 48 MHz this lands BRR on exactly 96, so the divisor carries no rounding
 * error at all and the whole rate budget is HSI48 trim, as it was at 1 Mbps.
 *
 * Everything timed in byte times below is derived from this, so changing it
 * here is enough.
 */
#define FT_BAUD 500000u

#define FT_BROADCAST_ID 0xFEu

/** FT-SCS baud rate code for FT_BAUD, served at address 6. Feetech numbers
 *  these the other way up from Dynamixel: 0 is 1 Mbps, 1 is 500 kbps, 4 is
 *  115200. */
#define FT_BAUD_CODE 1u

/**
 * Model, served at address 3 as u16 LE - where a Feetech device keeps its
 * servo major/minor version and where `servocom`'s scanner looks.
 *
 * 0xC011 is far outside the range any real Feetech servo reports (the highest
 * registered is the SM40BL at 10248), so it cannot be mistaken for one. It is
 * also what this board reported as its Dynamixel model number, which keeps a
 * board identifiable across the protocol change.
 */
#define FT_MODEL_NUMBER 0xC011u

/** Bumped when the layout of the block at 56, or the meaning of its status
 *  byte, changes. 11 dropped the mounting correction of v7: samples are now
 *  published exactly as the sensor measured them, and FT_STATUS_MOUNT_CORR
 *  (0x10) is gone rather than merely always clear. */
#define FT_FIRMWARE_VERSION 11u
#define FT_FIRMWARE_MINOR   0u

/** Start and length of the sensor block. */
#define FT_SENSOR_ADDR 56u
#define FT_SENSOR_LEN  20u

/** Size of the flat register file the slave serves. */
#define FT_TABLE_SIZE (FT_SENSOR_ADDR + FT_SENSOR_LEN)

/* Status flags, byte 19 of the sensor block. */
#define FT_STATUS_IMU_PRESENT  0x01u /* WHO_AM_I matched at boot            */
#define FT_STATUS_SFLP_READY   0x02u /* at least one quaternion has arrived */
#define FT_STATUS_FIFO_OVERRUN 0x04u /* FIFO overran since the last publish */
#define FT_STATUS_SPI_ERROR    0x08u /* an SPI transfer timed out           */
/* 0x10 was MOUNT_CORR up to v10, when every sample was rotated by a fixed
 * per-robot mounting trim before publication. The board is mounted straight
 * and nothing is applied now, so the flag is retired rather than hard-wired
 * clear - do not reuse the bit for something else, or a host that still knows
 * the old meaning will read it as a correction being applied. */

/**
 * Report a dead IMU in the FT-SCS error byte as well as in the status flags.
 *
 * Every FT-SCS status packet carries an error byte, and a Feetech-aware host
 * decodes it without being told anything about this board - `servocom` turns
 * it into a `FeetechError` on every read. Bit 1 is the magnetic encoder bit,
 * "the position sensor is not answering", which is the closest true statement
 * FT-SCS can make about an IMU that is not answering either.
 *
 * Set to 0 for a board that must never look faulted to generic tooling. The
 * authoritative answer is byte 19 of the sensor block either way; this only
 * decides whether the same fact also reaches a host that is not looking for
 * it.
 */
#define FT_ERR_MIRRORS_FAULTS 1

/** FT-SCS error bit used when FT_ERR_MIRRORS_FAULTS is set. */
#define FT_ERR_SENSOR 0x02u

/**
 * Bring USART1 up on the bus and start serving. How the line is shared is the
 * board's business - see DXL_BUFFERED in board.h and line_drive() in
 * ft_slave.c.
 *
 * From here the slave runs entirely out of the USART1 interrupt; there is no
 * poll function to call.
 */
void ft_slave_init(void);

/**
 * Publish a fresh sensor block.
 *
 * @param block  FT_SENSOR_LEN bytes, laid out as documented above. Copied
 *               under a critical section so a SYNC_READ can never answer with
 *               half of one sample and half of the next.
 */
void ft_slave_publish(const uint8_t *block);

/**
 * Set or clear the fault reported in the error byte of every status packet.
 *
 * Separate from publish() because the interesting case is the one where
 * publish() is never called: an IMU that did not answer WHO_AM_I leaves the
 * block at zero forever, and a board that then also reported no error would
 * be claiming to be healthy.
 */
void ft_slave_set_fault(bool fault);

/** Status packets sent since boot. Handy on a scope-less bench. */
uint32_t ft_slave_replies(void);

#endif /* FT_SLAVE_H */
