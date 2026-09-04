/**
 * @file    lsm6dsv16x.h
 * @brief   LSM6DSV16X over SPI1, configured for the SFLP game rotation vector.
 *
 * The host does not fuse anything: the chip's sensor-fusion low-power (SFLP)
 * block produces a game-rotation quaternion and estimates its own gyro bias,
 * and this firmware just forwards it. That is why the quaternion arrives here
 * as three IEEE half-precision components with w reconstructed downstream as
 * sqrt(1 - x^2 - y^2 - z^2) - the chip never sends w.
 *
 * The quaternion is only reachable through the FIFO (tag 0x13); there is no
 * output register for it. Raw gyro and accelerometer are read straight from
 * their output registers, so the FIFO batches nothing but SFLP.
 */

#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include <stdbool.h>
#include <stdint.h>

/** WHO_AM_I (0Fh) is fixed at 70h. */
#define LSM6DSV16X_WHO_AM_I_VALUE 0x70u

/** Gyro sensitivity at +/-500 dps, in mdps/LSB. Matches the 0.0175 the host
 *  applies in duck-control/src/imu.rs. */
#define LSM6DSV16X_GYRO_MDPS_PER_LSB 17.5f

/** Accelerometer sensitivity at +/-4 g, in mg/LSB. */
#define LSM6DSV16X_ACC_MG_PER_LSB 0.122f

typedef struct {
    /** Raw gyro counts, x/y/z, sensor frame, +/-500 dps full scale. */
    int16_t  gyro[3];
    /** Raw accelerometer counts, x/y/z, sensor frame, +/-4 g full scale. */
    int16_t  acc[3];
    /** SFLP game rotation vector x/y/z as IEEE binary16. w is implied. */
    uint16_t quat[3];
    /** True once at least one quaternion has come out of the FIFO. */
    bool     quat_valid;
    /** Set when the FIFO reported an overrun since the last poll. */
    bool     fifo_overrun;
    /** Set when an SPI transfer timed out. */
    bool     bus_error;
} lsm6dsv16x_sample_t;

/**
 * Bring SPI1 up and configure the chip.
 *
 * Software reset, then: I2C/I3C disabled, BDU on, accelerometer and gyroscope
 * at 120 Hz high-performance (+/-4 g, +/-500 dps), gyro LPF1 on, SFLP game
 * rotation vector at 120 Hz batched into a continuous-mode FIFO.
 *
 * @return true if WHO_AM_I matched and the configuration was written.
 */
bool lsm6dsv16x_init(void);

/**
 * Drain the FIFO and refresh @p out.
 *
 * Keeps only the newest quaternion in the FIFO - the block the host reads
 * carries one sample, so an older one is of no use to anybody. Gyro and
 * accelerometer come from the output registers, not the FIFO, so they are
 * whatever the chip holds right now.
 *
 * @return true if the transfers succeeded. `out->quat_valid` stays false until
 *         SFLP has actually written its first quaternion, which takes about
 *         0.7 s from power-up.
 */
bool lsm6dsv16x_poll(lsm6dsv16x_sample_t *out);

#endif /* LSM6DSV16X_H */
