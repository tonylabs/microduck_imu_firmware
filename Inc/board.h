/**
 * @file    board.h
 * @brief   Pin map and board constants for the microduck `imu_to_dxl` board.
 *
 * Taken from Schematic.pdf (TONYLABS, rev *). STM32C011F6U6TR in UFQFPN20:
 *
 * | Pin | Port        | Net       | Function                                    |
 * |-----|-------------|-----------|---------------------------------------------|
 * | 2   | VDD/VDDA    | +3.3V     | from AP2210K LDO (VIN 3.5-13.2 V)           |
 * | 3   | VSS/VSSA    | GND       |                                             |
 * | 4   | PF2/NRST    | RESET     | 10k pull-up + 100nF, TC2030 pin 3           |
 * | 5   | PA0         | IMU_INT   | <- LSM6DSV16X INT1 (pin 4)                  |
 * | 9   | PA4         | SPI_CS    | -> LSM6DSV16X CS (pin 12), 10k pull-up      |
 * | 10  | PA5         | SPI_SCK   | -> LSM6DSV16X SCL/SCK (pin 13)              |
 * | 11  | PA6         | SPI_MISO  | <- LSM6DSV16X SDO/SA0 (pin 1)               |
 * | 12  | PA7         | SPI_MOSI  | -> LSM6DSV16X SDA/SDI (pin 14)              |
 * | 13  | PA8         | LED       | D2 anode on +3.3V through R2 1k: LOW = lit  |
 * | 16  | PA13        | SWDIO     | TC2030 pin 2                                |
 * | 17  | PA14/BOOT0  | SWCLK     | TC2030 pin 4, 10k pull-down on BOOT0        |
 * | 18  | PB6         | DATA_TX   | Dynamixel half-duplex data, J2/J3 pin 3     |
 *
 * Unconnected: PA1, PA2, PA3, PA9/PA11, PA10/PA12, PB7, PC14, PC15.
 *
 * On the IMU side SDx (pin 2) and SCx (pin 3) are tied to GND and the aux SPI
 * (OCS_AUX, SDO_AUX) and INT2 are left open, so the part is a plain 4-wire SPI
 * slave with no OIS chain in use.
 *
 * The data line goes straight from PB6 to the connector - there is no bus
 * transceiver and no direction pin. See dxl_slave.c for how the line is shared.
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* A bare CMSIS project: no HAL, no CubeMX. The device selection stm32c0xx.h
 * would normally get from a -D on the command line is made here instead, so it
 * cannot go missing when build settings are regenerated. */
#ifndef STM32C011xx
#define STM32C011xx
#endif
#include "stm32c0xx.h"

/** Milliseconds since reset, counted by SysTick. Defined in main.c. */
uint32_t board_millis(void);

/* ── SPI to the IMU ──────────────────────────────────────────────────────── */

#define IMU_CS_PORT      GPIOA
#define IMU_CS_PIN       (1u << 4)
#define IMU_SCK_PIN      (1u << 5)
#define IMU_MISO_PIN     (1u << 6)
#define IMU_MOSI_PIN     (1u << 7)

/** LSM6DSV16X INT1. Data-ready / FIFO watermark; unused by the poll loop but
 *  wired, so a future revision can drive the cadence off it. */
#define IMU_INT_PORT     GPIOA
#define IMU_INT_PIN      (1u << 0)

/* ── LED ─────────────────────────────────────────────────────────────────── */

#define LED_PORT         GPIOA
#define LED_PIN          (1u << 8)

/* ── Dynamixel bus ───────────────────────────────────────────────────────── */

#define DXL_PORT         GPIOB
#define DXL_PIN_NUM      6u
#define DXL_PIN          (1u << DXL_PIN_NUM)

/** AF0 = USART1_TX on PB6. */
#define DXL_PIN_AF       0u

#endif /* BOARD_H */
