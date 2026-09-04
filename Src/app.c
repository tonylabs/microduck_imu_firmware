/**
 * @file    app.c
 * @brief   Poll the IMU, publish the block the control loop reads.
 *
 * The board does two things and nothing else: keep a fresh sample from the
 * LSM6DSV16X, and answer the host's SYNC_READ. The second runs entirely in the
 * USART1 interrupt (see dxl_slave.c), so this loop only has to keep the shadow
 * block current.
 *
 * The poll is paced rather than free-running. SFLP produces a quaternion at
 * 120 Hz and the host ticks at 50 Hz, so polling at 500 Hz is already four
 * times faster than the fastest thing downstream; spinning on SPI any harder
 * would only add bus traffic for the interrupt to be delayed by.
 */

#include "app.h"

#include <string.h>

#include "board.h"
#include "dxl_slave.h"
#include "lsm6dsv16x.h"

/** Poll period in milliseconds. */
#define POLL_PERIOD_MS 2u

/*
 * LED, in two states.
 *
 *   healthy  the IMU answered WHO_AM_I: a faded lub-dub every five seconds
 *   fault    it did not, or an SPI transfer is failing: ~12 Hz, hard edges
 *
 * The two are deliberately unalike in kind and not just in rate - one breathes,
 * the other strobes - so which one you are looking at is obvious from across a
 * room and after a glance, rather than needing to be timed.
 */
#define HEARTBEAT_PERIOD_MS 5000u
#define BLINK_FAULT_MS        40u

/** Envelope update interval. 4 ms gives 35 steps across the first beat's rise,
 *  which is smooth to the eye, and costs one multiply and one divide per step
 *  on a loop that is idle 95 % of the time. */
#define LED_STEP_MS 4u

/** How long a fault keeps the LED fast after the last SPI error.
 *
 *  bus_error is refreshed every poll, so an intermittent failure would
 *  otherwise raise the fault for a single 2 ms poll and drop it again - two
 *  milliseconds of fast blink that nobody can see. Holding it for two seconds
 *  makes a fault that comes and goes look like a fault. */
#define FAULT_HOLD_MS 2000u

static lsm6dsv16x_sample_t s_sample;
static bool     s_imu_present;
static uint8_t  s_counter;
static uint32_t s_next_poll;
static uint32_t s_next_blink;
static bool     s_led_on;
static uint32_t s_last_error_ms;
static bool     s_fault_shown;

/*
 * The LED is on TIM14_CH1 (PA8, AF13) as a ~1 kHz PWM, so the healthy pattern
 * can fade rather than switch.
 *
 * TIM14 and not TIM3: TIM3 is the microsecond clock the Dynamixel slave times
 * its turnaround with, and it is fixed at ARR = 0xFFFF, which as a PWM period
 * would be 15 Hz - visible flicker. TIM14 is otherwise unused and, unlike TIM1,
 * needs no MOE to drive its output.
 *
 * D2's anode sits on +3.3 V through R2, so the pin sinks: CC1P inverts the
 * channel, which lets CCR1 be read as brightness instead of as darkness.
 */
#define LED_MAX     1000u /* = ARR, so PWM is 1 MHz / 1001 = 999 Hz */
#define LED_PIN_AF    13u

/**
 * Brightness 0..LED_MAX to duty.
 *
 * Squared, because the eye is not linear: a linear ramp spends most of its
 * travel in a range that already looks fully lit, so the fade reads as a
 * switch with a soft edge rather than as breathing.
 */
static uint16_t led_duty(uint16_t level)
{
    return (uint16_t)(((uint32_t)level * (uint32_t)level) / LED_MAX);
}

static void led_level(uint16_t level)
{
    TIM14->CCR1 = led_duty(level);
}

static void led_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM14EN;

    /* Park the pin high - dark - before the timer owns it, so nothing flashes
     * between reset and the first envelope update. */
    LED_PORT->BSRR = LED_PIN;
    LED_PORT->MODER = (LED_PORT->MODER & ~(3u << (8 * 2))) | (1u << (8 * 2));

    TIM14->PSC = (uint16_t)((SystemCoreClock / 1000000u) - 1u); /* 1 MHz tick */
    TIM14->ARR = LED_MAX;
    TIM14->CCR1 = 0u;
    TIM14->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; /* PWM mode 1 */
    TIM14->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;                 /* active low  */
    TIM14->CR1 = TIM_CR1_ARPE;
    TIM14->EGR = TIM_EGR_UG;
    TIM14->CR1 |= TIM_CR1_CEN;

    LED_PORT->AFR[1] = (LED_PORT->AFR[1] & ~0xFu) | LED_PIN_AF;
    LED_PORT->MODER = (LED_PORT->MODER & ~(3u << (8 * 2))) | (2u << (8 * 2));
}

/** Linear interpolation over a segment of the heartbeat envelope. */
static uint16_t ramp(uint32_t t, uint32_t t0, uint32_t t1, uint32_t v0, uint32_t v1)
{
    if (t <= t0) {
        return (uint16_t)v0;
    }
    if (t >= t1) {
        return (uint16_t)v1;
    }
    uint32_t span = t1 - t0;
    uint32_t into = t - t0;
    return (v1 >= v0) ? (uint16_t)(v0 + ((v1 - v0) * into) / span)
                      : (uint16_t)(v0 - ((v0 - v1) * into) / span);
}

/**
 * Two beats and a long pause - lub-dub - over HEARTBEAT_PERIOD_MS.
 *
 * The second beat is deliberately weaker and starts before the first has fully
 * decayed. An even pair reads as "blinking twice"; the asymmetry is what makes
 * it read as a pulse.
 */
static uint16_t heartbeat_level(uint32_t phase_ms)
{
    if (phase_ms < 140u) {
        return ramp(phase_ms, 0u, 140u, 0u, LED_MAX);
    }
    if (phase_ms < 340u) {
        return ramp(phase_ms, 140u, 340u, LED_MAX, 0u);
    }
    if (phase_ms < 440u) {
        return ramp(phase_ms, 340u, 440u, 0u, 650u);
    }
    if (phase_ms < 700u) {
        return ramp(phase_ms, 440u, 700u, 650u, 0u);
    }
    return 0u;
}

static void put_i16(uint8_t *dst, int16_t v)
{
    dst[0] = (uint8_t)((uint16_t)v & 0xFFu);
    dst[1] = (uint8_t)((uint16_t)v >> 8);
}

static void put_u16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)(v >> 8);
}

void app_init(void)
{
    led_init();
    memset(&s_sample, 0, sizeof(s_sample));

    s_imu_present = lsm6dsv16x_init();

    /* The bus comes up whatever the IMU did. A board that answers with a zero
     * block and IMU_PRESENT clear is diagnosable from the host; one that stays
     * silent looks the same as an unplugged connector. */
    dxl_slave_init();

    s_next_poll = board_millis();
    s_next_blink = s_next_poll;
    /* Unsigned wraparound puts this exactly FAULT_HOLD_MS in the past however
     * small board_millis() currently is, so a clean start does not begin by
     * reporting a fault. */
    s_last_error_ms = s_next_poll - FAULT_HOLD_MS;
    s_fault_shown = !s_imu_present;
}

void app_task(void)
{
    uint32_t now = board_millis();

    if (s_sample.bus_error) {
        s_last_error_ms = now;
    }
    bool fault = !s_imu_present || ((uint32_t)(now - s_last_error_ms) < FAULT_HOLD_MS);

    /* Changing state re-arms the timer immediately. Without this, a fault that
     * appeared during the four-and-a-bit second gap between heartbeats would
     * not show on the LED until that gap had run out. */
    if (fault != s_fault_shown) {
        s_fault_shown = fault;
        s_next_blink = now;
    }

    if ((int32_t)(now - s_next_blink) >= 0) {
        if (fault) {
            /* Hard edges, no fade. A fault should look nothing like the healthy
             * pattern from across a room. */
            s_led_on = !s_led_on;
            led_level(s_led_on ? LED_MAX : 0u);
            s_next_blink = now + BLINK_FAULT_MS;
        } else {
            /* Free-running phase off the millisecond clock. It needs no state
             * of its own; the cost is one shortened beat when board_millis()
             * wraps, once every 49 days. */
            led_level(heartbeat_level(now % HEARTBEAT_PERIOD_MS));
            s_next_blink = now + LED_STEP_MS;
        }
    }

    if ((int32_t)(now - s_next_poll) < 0) {
        return;
    }
    s_next_poll = now + POLL_PERIOD_MS;

    if (!s_imu_present) {
        /* Retry occasionally: a brown-out on the IMU rail should not need a
         * power cycle of the whole robot to recover from. */
        if ((now % 1000u) < POLL_PERIOD_MS) {
            s_imu_present = lsm6dsv16x_init();
        }
        return;
    }

    (void)lsm6dsv16x_poll(&s_sample);

    uint8_t block[DXL_SENSOR_LEN];
    put_i16(&block[0], s_sample.gyro[0]);
    put_i16(&block[2], s_sample.gyro[1]);
    put_i16(&block[4], s_sample.gyro[2]);
    /* Left at zero until SFLP has written its table. The host reads all-zero
     * here as "orientation unknown" and holds its last good quaternion, which
     * is the only honest answer during the ~0.7 s the algorithm takes to
     * settle - reporting identity would tell fall detection the robot is
     * upright. */
    put_u16(&block[6], s_sample.quat_valid ? s_sample.quat[0] : 0u);
    put_u16(&block[8], s_sample.quat_valid ? s_sample.quat[1] : 0u);
    put_u16(&block[10], s_sample.quat_valid ? s_sample.quat[2] : 0u);
    put_i16(&block[12], s_sample.acc[0]);
    put_i16(&block[14], s_sample.acc[1]);
    put_i16(&block[16], s_sample.acc[2]);

    /* The counter is what tells the host a block is genuinely new. duck-control
     * already watches for a repeated 12-byte block and calls the orientation
     * frozen; this makes the same fault unambiguous over the full 20. */
    block[18] = s_counter++;

    uint8_t status = DXL_STATUS_IMU_PRESENT;
    if (s_sample.quat_valid) {
        status |= DXL_STATUS_SFLP_READY;
    }
    if (s_sample.fifo_overrun) {
        status |= DXL_STATUS_FIFO_OVERRUN;
        s_sample.fifo_overrun = false; /* sticky until reported, then cleared */
    }
    if (s_sample.bus_error) {
        status |= DXL_STATUS_SPI_ERROR;
    }
    block[19] = status;

    dxl_slave_publish(block);
}
