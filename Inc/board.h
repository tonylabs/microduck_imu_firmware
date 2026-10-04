/**
 * @file    board.h
 * @brief   Pin map for the microduck IMU board, per revision.
 *
 * Two revisions exist. Select with BOARD_REV, either here or with
 * -DBOARD_REV=n in the build settings. Everything in the .c files is written
 * against the macros below and never names a pin directly, so a new revision
 * is a new block here and nothing else.
 *
 * ── BOARD_REV 1 - the board on the bench ───────────────────────────────────
 *
 * STM32C011F6U6TR (U1), UFQFPN20. From Schematic.pdf, 2026-09-05.
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
 * | 13  | PA8         | LED       | TIM14_CH1. D2 anode on +3.3V: LOW = lit     |
 * | 16  | PA13        | SWDIO     | TC2030 pin 2                                |
 * | 17  | PA14/BOOT0  | SWCLK     | TC2030 pin 4, 10k pull-down on BOOT0        |
 * | 18  | PB6         | DATA_TX   | servo bus data, J2/J3 pin 3, direct         |
 *
 * DATA_TX runs from PB6 straight to the connector - no transceiver, no
 * direction pin. USART1 runs single-wire half-duplex (HDSEL) and the firmware
 * flips PB6 between open-drain (released) and push-pull (driving). See
 * dxl_slave.c.
 *
 * ── BOARD_REV 2 - buffered bus ───────────────────────────────────────────
 *
 * From the revised schematic, 2026-09-12. Adds U5 = SN74LVC1G125DBVR between
 * the MCU and the connectors. MCU stays U1.
 *
 * | Pin | Port      | Net        | Function                                       |
 * |-----|-----------|------------|------------------------------------------------|
 * | 5-8 | PA0-PA3   | -          | unconnected                                    |
 * | 13  | PA8       | IMU_INT    | <- LSM6DSV16X INT1                             |
 * | 14  | PA11/PA9  | LED        | as PA11: TIM1_CH4 (AF2). D2 anode on +3.3V     |
 * | 15  | PA12/PA10 | DXL_DIR    | as PA12: USART1_DE (AF1) -> U5 OE#, R7 10k up  |
 * | 18  | PB6       | USART1_TX  | -> U5 A                                        |
 * | 19  | PB7       | DXL_DATA   | <- U5 Y and J1/J4/J5. USART1_RX, internal pull-up |
 *
 * Pins 14 and 15 are shared: PA11/PA12 by default, PA9/PA10 only if
 * SYSCFG_CFGR1.PA11_RMP / PA12_RMP are set. The defaults are the useful ones
 * here - PA12 carries USART1_DE and PA11 carries TIM1_CH4 - so no remap is
 * done and none must be added.
 *
 * Direction is therefore **hardware**: with DEM set, USART1 asserts DE ahead of
 * every start bit and releases it after the last stop bit, timed by DEAT/DEDT.
 * DEP inverts the output to match U5's active-low OE#. The firmware never
 * toggles a GPIO to talk, and cannot get the timing wrong.
 *
 * R7's 10k pull-up on OE# is what keeps the bus safe between reset and
 * dxl_slave_init(): every GPIO is high-impedance until then, and a floating
 * OE# on a 74LVC1G125 may enable it.
 *
 * Common to both: SDx and SCx on the IMU are tied to GND and the aux SPI and
 * INT2 are open - a plain 4-wire SPI slave, no OIS chain.
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

#ifndef BOARD_REV
#define BOARD_REV 2
#endif

/** Milliseconds since reset, counted by SysTick. Defined in main.c. */
uint32_t board_millis(void);

/* ── SPI to the IMU (both revisions) ─────────────────────────────────────── */

#define IMU_CS_PORT      GPIOA
#define IMU_CS_PIN       (1u << 4)
#define IMU_SCK_PIN      (1u << 5)
#define IMU_MISO_PIN     (1u << 6)
#define IMU_MOSI_PIN     (1u << 7)

/* ── Servo bus, TX side (both revisions) ─────────────────────────────────── */

/* The DXL_* names are the schematic's: the nets really are called DXL_DATA and
 * DXL_DIR on the PCB, and the copper does not care that what rides them is now
 * FT-SCS rather than Dynamixel. Renaming them here would only make board.h
 * disagree with the board. The protocol lives in ft_slave.c. */

#define DXL_PORT         GPIOB
#define DXL_TX_PIN_NUM   6u
#define DXL_TX_PIN       (1u << DXL_TX_PIN_NUM)
#define DXL_TX_AF        0u /* USART1_TX */

#if BOARD_REV == 1

/* ── Rev 1 ───────────────────────────────────────────────────────────────── */

/** LSM6DSV16X INT1. Wired, not used to pace the loop. */
#define IMU_INT_PORT     GPIOA
#define IMU_INT_PIN_NUM  0u

#define LED_PORT         GPIOA
#define LED_PIN_NUM      8u
#define LED_AF           13u /* TIM14_CH1 */
#define LED_TIM          TIM14
#define LED_TIM_CH       1u
#define LED_TIM_RCC_EN() (RCC->APBENR2 |= RCC_APBENR2_TIM14EN)
/** TIM14 is a general-purpose timer: no BDTR, outputs drive as soon as CC1E
 *  is set. */
#define LED_TIM_HAS_MOE  0

/** No transceiver. The USART receives through its own TX pin (HDSEL) and the
 *  firmware releases the line by switching PB6 to open-drain. */
#define DXL_BUFFERED     0

#elif BOARD_REV == 2

/* ── Rev 2 ───────────────────────────────────────────────────────────────── */

#define IMU_INT_PORT     GPIOA
#define IMU_INT_PIN_NUM  8u

#define LED_PORT         GPIOA
#define LED_PIN_NUM      11u
#define LED_AF           2u /* TIM1_CH4 */
#define LED_TIM          TIM1
#define LED_TIM_CH       4u
#define LED_TIM_RCC_EN() (RCC->APBENR2 |= RCC_APBENR2_TIM1EN)
/** TIM1 has a BDTR and its outputs stay off until MOE is set. Forgetting this
 *  is silent: the timer runs, CCR updates, and the pin never moves. */
#define LED_TIM_HAS_MOE  1

/** U5 74LVC1G125 sits between PB6 and the bus. USART1 runs normal full-duplex:
 *  TX on PB6 into the buffer, RX on PB7 straight off the bus, DE on PA12 to
 *  the buffer's OE#. */
#define DXL_BUFFERED     1

#define DXL_RX_PIN_NUM   7u
#define DXL_RX_PIN       (1u << DXL_RX_PIN_NUM)
#define DXL_RX_AF        0u /* USART1_RX */

#define DXL_DE_PORT      GPIOA
#define DXL_DE_PIN_NUM   12u
#define DXL_DE_PIN       (1u << DXL_DE_PIN_NUM)
#define DXL_DE_AF        1u /* USART1_DE */

#else
#error "BOARD_REV must be 1 or 2"
#endif

/* Derived, so the .c files never repeat the shift. */
#define IMU_INT_PIN      (1u << IMU_INT_PIN_NUM)
#define LED_PIN          (1u << LED_PIN_NUM)

#endif /* BOARD_H */
