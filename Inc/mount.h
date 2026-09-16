/**
 * @file    mount.h
 * @brief   Per-robot mounting correction, applied to every published sample.
 *
 * The host (duck-control::imu::SflpDecoder) assumes this board is mounted
 * exactly as its DEFAULT_MOUNT says, and has no knob to trim that. A board
 * that sits a few degrees off has the policy believing the robot leans, and
 * the policy leans the robot the other way to "fix" it. The correction here
 * is where that trim lives instead: a fixed rotation in the *sensor* frame,
 * applied to the gyro and accelerometer vectors and folded into the SFLP
 * quaternion before any of them go on the bus. From the host's side the board
 * simply appears to be mounted straight.
 *
 * How the constants were measured (2026-09-16, robot duck-a4ca):
 *   robot in the HOME pose on a level floor, joints verified within 3° of
 *   HOME, trunk vertical; the trunk-frame gravity then read
 *   (+0.212, +0.038, -0.977) instead of (0, 0, -1) - 12.4° nose-down. The
 *   rotation taking the measured vector onto (0, 0, -1), expressed in the
 *   sensor frame through the host's DEFAULT_MOUNT, is the quaternion below.
 *   Re-measure and replace after any change to how the board is mounted;
 *   set MOUNT_CORR_ENABLED to 0 for a board mounted straight.
 */
#ifndef MOUNT_H
#define MOUNT_H

#include <stdint.h>
#include <stdbool.h>

#define MOUNT_CORR_ENABLED 1

/* Scalar-first unit quaternion, sensor frame: corrected = C * measured * C^-1 */
#define MOUNT_CORR_W  0.994121f
#define MOUNT_CORR_X  0.000000f
#define MOUNT_CORR_Y  0.106578f
#define MOUNT_CORR_Z (-0.019104f)

/** Rotate a raw i16 vector (gyro or accel counts) by the correction, in place. */
void mount_correct_vec(int16_t v[3]);

/** Fold the correction into an SFLP quaternion given as three binary16 (x, y, z),
 *  w implied non-negative. Rewrites the three halves; keeps w >= 0. */
void mount_correct_quat(uint16_t q_half[3]);

/** The correction as four int16, scaled by 10000, for the control table. */
void mount_correction_i16(int16_t out[4]);

#endif
