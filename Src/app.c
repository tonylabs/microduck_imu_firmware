/**
 * @file    app.c
 * @brief   Poll the IMU, publish the block the control loop reads.
 *
 * The board does two things and nothing else: keep a fresh sample from the
 * LSM6DSV16X, and answer the host's SYNC_READ. The second runs entirely in the
 * USART1 interrupt (see ft_slave.c), so this loop only has to keep the shadow
 * block current.
 *
 * The poll is paced rather than free-running. SFLP produces a quaternion at
 * 120 Hz and the host ticks at 50 Hz, so polling at 500 Hz is already four
 * times faster than the fastest thing downstream; spinning on SPI any harder
 * would only add bus traffic for the interrupt to be delayed by.
 */

#include "app.h"
#include "mount.h"

#include <string.h>

#include "board.h"
#include "ft_slave.h"
#include "lsm6dsv16x.h"

/** Poll period in milliseconds. */
#define POLL_PERIOD_MS 2u

/*
 * LED, in two states.
 *
 *   healthy  the IMU answered WHO_AM_I: a heartbeat, lub-dub at 60 bpm, hard
 *            edges
 *   fault    it did not, or an SPI transfer is failing: ~12 Hz, hard edges
 *
 * The two are deliberately unalike in kind and not just in rate - one beats,
 * the other strobes - so which one you are looking at is obvious from across a
 * room and after a glance, rather than needing to be timed.
 *
 * History: v2 was a lub-dub every five seconds, which from the bench read as
 * "blinked a few times and stopped". v3 replaced it with a 2 s breath, which
 * had no dark gap to misread but was too slow and too soft to read as alive.
 * v8 goes back to the heartbeat at a resting human rate: the gap between beats
 * is under a second, so nothing looks stopped, and the double pulse is
 * unmistakably a pulse.
 */
#define BEAT_PERIOD_MS 1000u /* 60 bpm */
#define BLINK_FAULT_MS   40u

/** The two pulses of a beat, ms into the period. Lub then dub, the dub softer
 *  and close behind - roughly the S1/S2 spacing of a real heart, then a rest
 *  for the remainder of the period. Everything between them is dark. */
#define LUB_ON_MS     0u
#define LUB_OFF_MS   80u
#define DUB_ON_MS   200u
#define DUB_OFF_MS  260u

/** Dub brightness, 0..LED_MAX. After led_duty() squares it, about a third of
 *  the lub's light - clearly the second, quieter sound. */
#define DUB_LEVEL 600u

/** Envelope update interval. The pattern is switched, not faded, so this only
 *  bounds how late an edge can be; 4 ms is invisible against 80 ms pulses. */
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
 * The LED is a ~1 kHz PWM on a timer channel, so the dub can be dimmer than
 * the lub. Which timer and pin is the board's business - board.h
 * names LED_TIM / LED_PIN_NUM / LED_AF per revision.
 *
 * Not TIM3, on any revision: TIM3 is the microsecond clock the Dynamixel slave
 * times its turnaround with, and it is fixed at ARR = 0xFFFF, which as a PWM
 * period would be 15 Hz - visible flicker.
 *
 * D2's anode sits on +3.3 V through R2, so the pin sinks: CC1P inverts the
 * channel, which lets CCR1 be read as brightness instead of as darkness.
 */
#define LED_MAX 1000u /* = ARR, so PWM is 1 MHz / 1001 = 999 Hz */

/*
 * Channel plumbing. Channels 1-2 live in CCMR1 and 3-4 in CCMR2, and every
 * channel has its own enable/polarity pair in CCER and its own CCR. Resolved
 * once here from LED_TIM_CH so the init below reads the same for any channel.
 */
#if LED_TIM_CH == 1
#define LED_CCMR      CCMR1
#define LED_OCM_POS   TIM_CCMR1_OC1M_Pos
#define LED_OCPE      TIM_CCMR1_OC1PE
#define LED_CCE       TIM_CCER_CC1E
#define LED_CCP       TIM_CCER_CC1P
#define LED_CCR       CCR1
#elif LED_TIM_CH == 4
#define LED_CCMR      CCMR2
#define LED_OCM_POS   TIM_CCMR2_OC4M_Pos
#define LED_OCPE      TIM_CCMR2_OC4PE
#define LED_CCE       TIM_CCER_CC4E
#define LED_CCP       TIM_CCER_CC4P
#define LED_CCR       CCR4
#else
#error "LED_TIM_CH: only channels 1 and 4 are wired up here"
#endif

/**
 * Brightness 0..LED_MAX to duty.
 *
 * Squared, because the eye is not linear: without it a level of 600 looks
 * nearly as bright as 1000, and the dub would not read as the softer beat.
 */
static uint16_t led_duty(uint16_t level)
{
    return (uint16_t)(((uint32_t)level * (uint32_t)level) / LED_MAX);
}

static void led_level(uint16_t level)
{
    LED_TIM->LED_CCR = led_duty(level);
}

static void led_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    LED_TIM_RCC_EN();

    /* Park the pin high - dark - before the timer owns it, so nothing flashes
     * between reset and the first envelope update. */
    LED_PORT->BSRR = LED_PIN;
    LED_PORT->MODER = (LED_PORT->MODER & ~(3u << (LED_PIN_NUM * 2)))
                    | (1u << (LED_PIN_NUM * 2));

    LED_TIM->PSC = (uint16_t)((SystemCoreClock / 1000000u) - 1u); /* 1 MHz tick */
    LED_TIM->ARR = LED_MAX;
    LED_TIM->LED_CCR = 0u;
    LED_TIM->LED_CCMR = (6u << LED_OCM_POS) | LED_OCPE; /* PWM mode 1, preload */
    LED_TIM->CCER = LED_CCE | LED_CCP;                   /* enabled, active low  */
#if LED_TIM_HAS_MOE
    /* TIM1/TIM16/TIM17 gate every output behind the main output enable. Without
     * this the timer counts, CCR1 updates, and the pin never moves - there is
     * no error, just a dark LED. */
    LED_TIM->BDTR = TIM_BDTR_MOE;
#endif
    LED_TIM->CR1 = TIM_CR1_ARPE;
    LED_TIM->EGR = TIM_EGR_UG;
    LED_TIM->CR1 |= TIM_CR1_CEN;

    /* AFR[0] covers pins 0-7, AFR[1] pins 8-15. */
    LED_PORT->AFR[LED_PIN_NUM / 8u] =
        (LED_PORT->AFR[LED_PIN_NUM / 8u] & ~(0xFu << ((LED_PIN_NUM % 8u) * 4u)))
        | ((uint32_t)LED_AF << ((LED_PIN_NUM % 8u) * 4u));
    LED_PORT->MODER = (LED_PORT->MODER & ~(3u << (LED_PIN_NUM * 2)))
                    | (2u << (LED_PIN_NUM * 2));
}

/**
 * One beat over BEAT_PERIOD_MS: lub, dark, softer dub, dark. Switched, not
 * ramped - a heartbeat is a thump, and a fade is what the v3 breath was.
 */
static uint16_t heartbeat_level(uint32_t phase_ms)
{
    if (phase_ms >= LUB_ON_MS && phase_ms < LUB_OFF_MS) {
        return LED_MAX;
    }
    if (phase_ms >= DUB_ON_MS && phase_ms < DUB_OFF_MS) {
        return DUB_LEVEL;
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
    ft_slave_init();

    /* The mounting correction in force, so a host can read what this board is
     * doing to its samples rather than infer it. */
    {
        int16_t c[4]; uint8_t raw[8];
        mount_correction_i16(c);
        for (int i = 0; i < 4; i++) { put_i16(&raw[i * 2], c[i]); }
        ft_slave_set_static(FT_MOUNT_ADDR, raw, 8u);
    }

    s_next_poll = board_millis();
    s_next_blink = s_next_poll;
    /* Unsigned wraparound puts this exactly FAULT_HOLD_MS in the past however
     * small board_millis() currently is, so a clean start does not begin by
     * reporting a fault. */
    s_last_error_ms = s_next_poll - FAULT_HOLD_MS;
    s_fault_shown = !s_imu_present;
    ft_slave_set_fault(s_fault_shown);
}

void app_task(void)
{
    uint32_t now = board_millis();

    if (s_sample.bus_error) {
        s_last_error_ms = now;
    }
    bool fault = !s_imu_present || ((uint32_t)(now - s_last_error_ms) < FAULT_HOLD_MS);

    /* Changing state re-arms the timer immediately, so a fault shows on the
     * LED on the next pass rather than at the end of the current 4 ms step. */
    if (fault != s_fault_shown) {
        s_fault_shown = fault;
        s_next_blink = now;
        /* The same fact the LED is showing, in the error byte of every status
         * packet. Driven from here rather than from the published block
         * because the case that matters most - an IMU that never answered
         * WHO_AM_I - is exactly the case where no block is ever published. */
        ft_slave_set_fault(fault);
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
            led_level(heartbeat_level(now % BEAT_PERIOD_MS));
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

    /* Per-robot mounting trim, before anything is published - mount.h.
     *
     * On copies, never on s_sample itself. The gyro and accelerometer are
     * re-read from the chip every poll, but the quaternion only changes when
     * the FIFO delivers one (120 Hz against a 500 Hz poll), so correcting it in
     * place applied the rotation again on every poll until the next sample -
     * a static robot published four or five different orientations in turn. */
    int16_t  gyro[3] = { s_sample.gyro[0], s_sample.gyro[1], s_sample.gyro[2] };
    int16_t  acc[3]  = { s_sample.acc[0], s_sample.acc[1], s_sample.acc[2] };
    uint16_t quat[3] = { s_sample.quat[0], s_sample.quat[1], s_sample.quat[2] };
    mount_correct_vec(gyro);
    mount_correct_vec(acc);
    if (s_sample.quat_valid) {
        mount_correct_quat(quat);
    }

    uint8_t block[FT_SENSOR_LEN];
    put_i16(&block[0], gyro[0]);
    put_i16(&block[2], gyro[1]);
    put_i16(&block[4], gyro[2]);
    /* Left at zero until SFLP has written its table. The host reads all-zero
     * here as "orientation unknown" and holds its last good quaternion, which
     * is the only honest answer during the ~0.7 s the algorithm takes to
     * settle - reporting identity would tell fall detection the robot is
     * upright. */
    put_u16(&block[6], s_sample.quat_valid ? quat[0] : 0u);
    put_u16(&block[8], s_sample.quat_valid ? quat[1] : 0u);
    put_u16(&block[10], s_sample.quat_valid ? quat[2] : 0u);
    put_i16(&block[12], acc[0]);
    put_i16(&block[14], acc[1]);
    put_i16(&block[16], acc[2]);

    /* The counter is what tells the host a block is genuinely new. duck-control
     * already watches for a repeated 12-byte block and calls the orientation
     * frozen; this makes the same fault unambiguous over the full 20. */
    block[18] = s_counter++;

    uint8_t status = FT_STATUS_IMU_PRESENT;
    if (s_sample.quat_valid) {
        status |= FT_STATUS_SFLP_READY;
    }
#if MOUNT_CORR_ENABLED
    status |= FT_STATUS_MOUNT_CORR;
#endif
    if (s_sample.fifo_overrun) {
        status |= FT_STATUS_FIFO_OVERRUN;
        s_sample.fifo_overrun = false; /* sticky until reported, then cleared */
    }
    if (s_sample.bus_error) {
        status |= FT_STATUS_SPI_ERROR;
    }
    block[19] = status;

    ft_slave_publish(block);
}
