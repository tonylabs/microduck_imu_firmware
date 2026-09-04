/**
 * @file    lsm6dsv16x.c
 * @brief   LSM6DSV16X driver: bare SPI1 register access, SFLP via FIFO.
 *
 * SPI1 is set up here rather than by any generated init code, so clock polarity
 * and phase are stated where the protocol that depends on them is written down.
 * Everything is blocking; the transfers are at most 15 bytes at 6 MHz, about
 * 20 us, which is shorter than the gap between two Dynamixel bytes.
 */

#include "lsm6dsv16x.h"

#include "board.h"

/* -- register map (see DS13510 rev 4) ------------------------------------- */

#define REG_FUNC_CFG_ACCESS   0x01u
#define REG_IF_CFG            0x03u
#define REG_FIFO_CTRL3        0x09u
#define REG_FIFO_CTRL4        0x0Au
#define REG_WHO_AM_I          0x0Fu
#define REG_CTRL1             0x10u
#define REG_CTRL2             0x11u
#define REG_CTRL3             0x12u
#define REG_CTRL6             0x15u
#define REG_CTRL7             0x16u
#define REG_CTRL8             0x17u
#define REG_FIFO_STATUS1      0x1Bu
#define REG_OUTX_L_G          0x22u
#define REG_OUTX_L_A          0x28u
#define REG_FIFO_DATA_OUT_TAG 0x78u

/* Embedded-functions page, reached with EMB_FUNC_REG_ACCESS in FUNC_CFG_ACCESS. */
#define REG_EMB_FUNC_EN_A      0x04u
#define REG_EMB_FUNC_FIFO_EN_A 0x44u
#define REG_SFLP_ODR           0x5Eu
#define REG_EMB_FUNC_INIT_A    0x66u

#define EMB_FUNC_REG_ACCESS 0x80u

/** FIFO tag for the SFLP game rotation vector, in TAG_SENSOR[4:0]. */
#define FIFO_TAG_SFLP_GAME 0x13u

/** SPI read direction bit in the address byte. */
#define SPI_READ 0x80u

/* -- configuration values ------------------------------------------------- */

#define CTRL1_XL_120HZ_HP  0x06u /* OP_MODE_XL = high-performance, ODR = 120 Hz  */
#define CTRL2_G_120HZ_HP   0x06u /* OP_MODE_G  = high-performance, ODR = 120 Hz  */
#define CTRL3_BDU_IFINC    0x44u /* BDU | IF_INC                                 */
#define CTRL3_SW_RESET     0x01u
#define CTRL6_FS_500DPS    0x02u /* FS_G = +/-500 dps, LPF1 bandwidth 000        */
#define CTRL7_LPF1_G_EN    0x01u
#define CTRL8_FS_4G        0x01u /* FS_XL = +/-4 g                               */
#define IF_CFG_I2C_I3C_DIS 0x01u
#define FIFO_CTRL4_CONT    0x06u /* FIFO_MODE = continuous, nothing else batched */
#define SFLP_ODR_120HZ     0x5Bu /* reserved bits 6,1,0 set; SFLP_GAME_ODR = 011 */
#define EMB_SFLP_GAME_EN   0x02u
#define EMB_SFLP_GAME_FIFO 0x02u
#define EMB_SFLP_GAME_INIT 0x02u

/** FIFO_STATUS2 bits used here. */
#define FIFO_STATUS2_DIFF8   0x01u
#define FIFO_STATUS2_OVR_IA  0x40u

/** Spin limit on an SPI flag. One byte at 6 MHz is ~1.3 us, so this is orders
 *  of magnitude of slack; it exists only so a dead chip cannot wedge the bus
 *  task, which is the one thing on this board that must never stop. */
#define SPI_SPIN_LIMIT 20000u

static bool s_bus_error;

/* -- SPI1 ----------------------------------------------------------------- */

static void spi_gpio_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    /* CS is a plain output driven by software: hardware NSS pulses per frame,
     * which would deselect the chip between the address byte and the data of
     * the same transaction. */
    IMU_CS_PORT->BSRR = IMU_CS_PIN;
    IMU_CS_PORT->MODER = (IMU_CS_PORT->MODER & ~(3u << (4 * 2))) | (1u << (4 * 2));
    IMU_CS_PORT->OSPEEDR |= (3u << (4 * 2));

    /* SCK, MISO, MOSI are AF0 on PA5/PA6/PA7. */
    IMU_CS_PORT->AFR[0] &= ~((0xFu << (5 * 4)) | (0xFu << (6 * 4)) | (0xFu << (7 * 4)));
    IMU_CS_PORT->MODER =
        (IMU_CS_PORT->MODER & ~((3u << (5 * 2)) | (3u << (6 * 2)) | (3u << (7 * 2))))
        | ((2u << (5 * 2)) | (2u << (6 * 2)) | (2u << (7 * 2)));
    IMU_CS_PORT->OSPEEDR |= (3u << (5 * 2)) | (3u << (7 * 2));

    /* INT1 in. Wired, and deliberately not used to pace the loop yet - see
     * app.c for why the poll is free-running. */
    IMU_INT_PORT->MODER &= ~(3u << (0 * 2));
}

static void spi_periph_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;

    SPI1->CR1 = 0u;
    /* 8-bit frames, and RXNE must assert on 8 bits rather than 16 or the read
     * of DR never completes. */
    SPI1->CR2 = SPI_CR2_FRXTH | (7u << SPI_CR2_DS_Pos);
    /* Master, software NSS, mode 3 (CPOL = 1, CPHA = 1), fPCLK/8 = 6 MHz. The
     * LSM6DSV16X accepts up to 10 MHz. */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | SPI_CR1_CPOL | SPI_CR1_CPHA
              | (2u << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
}

static uint8_t spi_xfer(uint8_t tx)
{
    uint32_t spin = SPI_SPIN_LIMIT;
    while ((SPI1->SR & SPI_SR_TXE) == 0u) {
        if (--spin == 0u) {
            s_bus_error = true;
            return 0xFFu;
        }
    }
    *(volatile uint8_t *)&SPI1->DR = tx;

    spin = SPI_SPIN_LIMIT;
    while ((SPI1->SR & SPI_SR_RXNE) == 0u) {
        if (--spin == 0u) {
            s_bus_error = true;
            return 0xFFu;
        }
    }
    return *(volatile uint8_t *)&SPI1->DR;
}

static void cs_low(void)
{
    IMU_CS_PORT->BRR = IMU_CS_PIN;
    __NOP();
    __NOP();
}

static void cs_high(void)
{
    uint32_t spin = SPI_SPIN_LIMIT;
    while ((SPI1->SR & SPI_SR_BSY) != 0u) {
        if (--spin == 0u) {
            s_bus_error = true;
            break;
        }
    }
    __NOP();
    __NOP();
    IMU_CS_PORT->BSRR = IMU_CS_PIN;
}

static void reg_write(uint8_t reg, uint8_t value)
{
    cs_low();
    (void)spi_xfer(reg);
    (void)spi_xfer(value);
    cs_high();
}

static void reg_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    cs_low();
    (void)spi_xfer((uint8_t)(reg | SPI_READ));
    for (uint8_t i = 0u; i < len; i++) {
        buf[i] = spi_xfer(0xFFu);
    }
    cs_high();
}

static uint8_t reg_read8(uint8_t reg)
{
    uint8_t v = 0u;
    reg_read(reg, &v, 1u);
    return v;
}

/** Write one embedded-functions register. The page switch is a register write
 *  of its own, so each of these costs three transactions - acceptable because
 *  it only happens at init. */
static void emb_write(uint8_t reg, uint8_t value)
{
    reg_write(REG_FUNC_CFG_ACCESS, EMB_FUNC_REG_ACCESS);
    reg_write(reg, value);
    reg_write(REG_FUNC_CFG_ACCESS, 0x00u);
}

/* -- init ----------------------------------------------------------------- */

static void spin_delay(uint32_t loops)
{
    while (loops-- != 0u) {
        __NOP();
    }
}

bool lsm6dsv16x_init(void)
{
    s_bus_error = false;

    spi_gpio_init();
    spi_periph_init();

    /* Boot time is 10 ms from power-on and the MCU reaches here well inside
     * that, so the wait is not optional. */
    spin_delay(500000u);

    reg_write(REG_CTRL3, CTRL3_SW_RESET);
    for (uint32_t i = 0u; i < 1000u; i++) {
        if ((reg_read8(REG_CTRL3) & CTRL3_SW_RESET) == 0u) {
            break;
        }
        spin_delay(2000u);
    }

    if (reg_read8(REG_WHO_AM_I) != LSM6DSV16X_WHO_AM_I_VALUE) {
        return false;
    }

    /* IF_CFG survives the software reset, so it is written after it. Disabling
     * I2C and I3C matters even though only SPI is wired: the I3C state machine
     * otherwise stays live on the same pins. */
    reg_write(REG_IF_CFG, IF_CFG_I2C_I3C_DIS);
    reg_write(REG_CTRL3, CTRL3_BDU_IFINC);

    reg_write(REG_CTRL8, CTRL8_FS_4G);
    reg_write(REG_CTRL6, CTRL6_FS_500DPS);
    reg_write(REG_CTRL7, CTRL7_LPF1_G_EN);

    /* Nothing but SFLP goes into the FIFO: gyro and accelerometer are read from
     * their output registers, which keeps the FIFO shallow and means a slow
     * poll cannot make the quaternion lag behind the rates. */
    reg_write(REG_FIFO_CTRL3, 0x00u);

    emb_write(REG_SFLP_ODR, SFLP_ODR_120HZ);
    emb_write(REG_EMB_FUNC_FIFO_EN_A, EMB_SFLP_GAME_FIFO);
    emb_write(REG_EMB_FUNC_EN_A, EMB_SFLP_GAME_EN);
    emb_write(REG_EMB_FUNC_INIT_A, EMB_SFLP_GAME_INIT);

    reg_write(REG_FIFO_CTRL4, FIFO_CTRL4_CONT);

    /* Sensors last, so the fusion block is fully configured before it starts
     * seeing samples and its first output already comes from the intended
     * chain. */
    reg_write(REG_CTRL1, CTRL1_XL_120HZ_HP);
    reg_write(REG_CTRL2, CTRL2_G_120HZ_HP);

    return !s_bus_error;
}

/* -- poll ----------------------------------------------------------------- */

/** FIFO words to pull in one poll. At 120 Hz SFLP and a poll rate above 200 Hz
 *  there is at most one waiting; the ceiling only matters after a stall, where
 *  it bounds how long a single poll can hold the CPU away from the bus. */
#define FIFO_DRAIN_LIMIT 16u

bool lsm6dsv16x_poll(lsm6dsv16x_sample_t *out)
{
    uint8_t raw[6];
    uint8_t status[2];

    s_bus_error = false;

    reg_read(REG_FIFO_STATUS1, status, 2u);
    uint16_t pending = (uint16_t)(status[0] | ((uint16_t)(status[1] & FIFO_STATUS2_DIFF8) << 8));
    if ((status[1] & FIFO_STATUS2_OVR_IA) != 0u) {
        out->fifo_overrun = true;
    }

    if (pending > FIFO_DRAIN_LIMIT) {
        pending = FIFO_DRAIN_LIMIT;
    }
    for (uint16_t i = 0u; i < pending; i++) {
        uint8_t word[7];
        reg_read(REG_FIFO_DATA_OUT_TAG, word, 7u);
        if ((word[0] >> 3) == FIFO_TAG_SFLP_GAME) {
            /* Overwriting a quaternion read earlier in this same drain is the
             * point: the host reads one sample per tick and wants the newest. */
            out->quat[0] = (uint16_t)(word[1] | ((uint16_t)word[2] << 8));
            out->quat[1] = (uint16_t)(word[3] | ((uint16_t)word[4] << 8));
            out->quat[2] = (uint16_t)(word[5] | ((uint16_t)word[6] << 8));
            out->quat_valid = true;
        }
    }

    reg_read(REG_OUTX_L_G, raw, 6u);
    out->gyro[0] = (int16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    out->gyro[1] = (int16_t)(raw[2] | ((uint16_t)raw[3] << 8));
    out->gyro[2] = (int16_t)(raw[4] | ((uint16_t)raw[5] << 8));

    reg_read(REG_OUTX_L_A, raw, 6u);
    out->acc[0] = (int16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    out->acc[1] = (int16_t)(raw[2] | ((uint16_t)raw[3] << 8));
    out->acc[2] = (int16_t)(raw[4] | ((uint16_t)raw[5] << 8));

    out->bus_error = s_bus_error;
    return !s_bus_error;
}
