#include "ft_sample.h"

static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

void ft_sample_encode(uint8_t block[FT_SENSOR_LEN],
                      const lsm6dsv16x_sample_t *sample, uint8_t counter)
{
    for (unsigned i = 0; i < 3; ++i) {
        put_u16(&block[2u * i], (uint16_t)sample->gyro[i]);
        put_u16(&block[6u + 2u * i], sample->quat_valid ? sample->quat[i] : 0u);
    }
    block[FT_COUNTER_OFFSET] = counter;
    /* The installed FT6 decoder accepts status <= 0x0f and marks a sample
     * valid only when (status & 0x07) == 0. Never encode positive readiness
     * flags here: healthy is zero. Bit 3 is accepted but unused by us. */
    uint8_t status = sample->quat_valid ? 0u : FT_STATUS_NOT_READY;
    if (sample->fifo_overrun) {
        status |= FT_STATUS_FIFO_OVERRUN;
    }
    if (sample->bus_error) {
        status |= FT_STATUS_SPI_ERROR;
    }
    block[FT_STATUS_OFFSET] = status;
    block[FT_RESERVED_OFFSET] = 0u;
}
