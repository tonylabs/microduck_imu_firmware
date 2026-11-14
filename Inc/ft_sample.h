#ifndef FT_SAMPLE_H
#define FT_SAMPLE_H

#include "ft_slave.h"
#include "lsm6dsv16x.h"

/* Encode the Microduck FT6 56/15 payload without changing the sensor sample. */
void ft_sample_encode(uint8_t block[FT_SENSOR_LEN],
                      const lsm6dsv16x_sample_t *sample, uint8_t counter);

#endif
