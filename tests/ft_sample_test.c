#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ft_sample.h"

/* Independent runtime contract, recovered from robotd's FT6 decoder:
 * require 15 bytes, reserved byte 14 == 0, status byte 13 <= 15;
 * valid iff (status & 7) == 0. Counter is byte 12. */
static bool runtime_valid(const uint8_t *payload, unsigned len)
{
    assert(len == 15);
    assert(payload[14] == 0);
    assert(payload[13] <= 15);
    return (payload[13] & 7) == 0;
}

int main(void)
{
    assert(FT_SENSOR_ADDR == 56 && FT_SENSOR_LEN == 15);
    assert(FT_FIRMWARE_VERSION == 6 && FT_FIRMWARE_MINOR == 0);
    assert(FT_MODEL_NUMBER == 0xf200 && FT_BAUD == 1000000);
    lsm6dsv16x_sample_t sample = {
        .gyro = {-32768, 32767, -1},
        .quat = {0x3800, 0xb400, 0x3000},
        .acc = {-12345, 23456, -32768},
        .quat_valid = true,
    };
    uint8_t guarded[17];
    memset(guarded, 0xa5, sizeof guarded);
    uint8_t *payload = &guarded[1];
    const uint8_t expected[15] = {
        0x00, 0x80, 0xff, 0x7f, 0xff, 0xff,
        0x00, 0x38, 0x00, 0xb4, 0x00, 0x30, 0xff, 0x00, 0x00,
    };
    ft_sample_encode(payload, &sample, 255);
    assert(memcmp(payload, expected, sizeof expected) == 0);
    assert(runtime_valid(payload, 15));
    assert(guarded[0] == 0xa5 && guarded[16] == 0xa5);
    /* Accelerometer values must never leak into FT6 metadata. */
    memset(sample.acc, 0xff, sizeof sample.acc);
    ft_sample_encode(payload, &sample, 255);
    assert(memcmp(payload, expected, sizeof expected) == 0);
    ft_sample_encode(payload, &sample, (uint8_t)(255u + 1u));
    assert(payload[12] == 0 && runtime_valid(payload, 15));
    /* Every startup/fault combination remains parseable but invalid. */
    for (unsigned faults = 0; faults < 8; ++faults) {
        sample.quat_valid = !(faults & 1);
        sample.fifo_overrun = (faults & 2) != 0;
        sample.bus_error = (faults & 4) != 0;
        ft_sample_encode(payload, &sample, 42);
        assert(runtime_valid(payload, 15) == (faults == 0));
        assert(payload[12] == 42);
        if (!sample.quat_valid) {
            const uint8_t zero[6] = {0};
            assert(memcmp(payload + 6, zero, 6) == 0);
        }
    }
    sample.quat_valid = true;
    sample.fifo_overrun = sample.bus_error = false;
    ft_sample_encode(payload, &sample, 43);
    assert(runtime_valid(payload, 15));
    puts("FT6 encoding: layout, signed gyro, quaternion, counter wrap, startup, faults and recovery passed");
}
