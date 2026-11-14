# Microduck FT6 runtime contract

Verified against `duck-a4ca.local`, `/opt/robot/feetech-ft5-r5/robotd`,
version 0.15.1 (unknown revision, IPC API v40), on 2026-11-13.
The executable SHA-256 was:

```
edf182edb7bf11aeeeedfdab98170ab2ec48a9ec18417f9b68c744b0a60c6d16
```

The installed executable was inspected read-only because its FT6 source was
not present in the local Microduck checkout or on the robot. These are the
requirements of that executable, not a claim about all FT6 implementations.

## Identity and payload

ID 200, 1,000,000 baud, Feetech FT-SCS. Registers 0–4 return
`06 00 00 00 F2`: version 6.0, END 0, little-endian model `0xF200`.

The runtime reads 15 bytes starting at register 56:

| Offset | Size | Contents |
|---|---|---|
| 0 | 6 | Gyro XYZ, signed little-endian 16-bit counts, 17.5 mdps/LSB |
| 6 | 6 | SFLP quaternion XYZ, little-endian IEEE binary16; W reconstructed by host |
| 12 | 1 | Publication counter, modulo 256 |
| 13 | 1 | Status: healthy = 0 |
| 14 | 1 | Reserved: must be 0 |

The decoder at executable virtual address `0x443d28` checks the length at
`0x443d54`, reserved byte at `0x443d5c`, and status at `0x443d64`.
It rejects nonzero reserved bytes and status greater than `0x0f`, reads the
counter at `0x443d7c`, and treats `(status & 0x07) == 0` as valid.
The gyro/quaternion decode immediately preceding it uses offsets 0–11.

This firmware uses bits 0, 1 and 2 to report not-ready, FIFO overrun and SPI
failure respectively. This is our assignment of fault causes; the inspected
runtime uses these bits collectively to mark the sample invalid. Bit 3 is
unused. Startup sets not-ready before enabling the bus. No positive
IMU-present or SFLP-ready flags are transmitted.

The counter advances per publication at 500 Hz, preserving the previous
publication cadence; sensor/SFLP ODR remains 120 Hz. It is not a distinct
quaternion-sample counter. Gyro/quaternion samples remain in the sensor frame
with no mounting correction. Accelerometer readings are retained internally
but are no longer exposed at offsets 12–17: those positions conflicted with
the runtime's metadata.

## Validation

Run the native encoder regression test from the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -IInc \
  Src/ft_sample.c tests/ft_sample_test.c -o /tmp/microduck-ft6-test
/tmp/microduck-ft6-test
```

It checks a fixed wire vector, signed gyro encoding, binary16 byte order,
counter wrap, buffer boundaries, independence from accelerometer values,
startup, every fault combination, and recovery against the decoder's checks.

On 2026-11-13 the test passed, and the CubeIDE Release build completed with
zero errors/warnings (3,820 bytes of flash, 2,244 bytes of BSS). The generated
`Release/microduck_imu_firmware.hex` was flashed through STM32CubeProgrammer
2.23.0 via STLINK-V3MINIE, read-back verified, and the MCU reset.

At 01:14:49 Asia/Shanghai, the existing commissioning runtime (PID 3572,
without restarting or reconfiguring it) reported:

```
robot     healthy
  loop      50.0 of 50.0 Hz
  bus       ok
  imu       ready
```

Before this change it continually reported `unsupported FT6 IMU
status/reserved byte`. Occasional servo Sync Read timeouts (e.g. ID 33) were
also seen; this change does not claim to eliminate all shared-bus errors.
This verifies runtime acceptance and readiness, not mounting calibration or
walking performance.
