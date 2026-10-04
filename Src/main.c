/**
 * @file    main.c
 * @brief   Clock, SysTick, and the main loop.
 *
 * This is a bare CMSIS project - no HAL. The firmware configures SPI1, USART1,
 * TIM3 and every GPIO at the register level anyway (see lsm6dsv16x.c and
 * ft_slave.c), so the HAL was only ever supplying three things: HAL_Init(),
 * SystemClock_Config() and HAL_GetTick(). Those are the forty lines below, and
 * dropping the rest takes the image from roughly 9 KB to 3 KB on a part with
 * 32 KB of flash.
 */

#include "app.h"
#include "board.h"

/** Milliseconds since reset. Wraps after 49 days; every comparison against it
 *  is written as a signed difference, so the wrap is harmless. */
static volatile uint32_t s_millis;

void SysTick_Handler(void)
{
    s_millis++;
}

uint32_t board_millis(void)
{
    return s_millis;
}

/**
 * SYSCLK = HSI48 undivided = 48 MHz.
 *
 * This is not the reset state: HSIDIV comes up at 0b010, dividing by 4 for a
 * 12 MHz SYSCLK. 48 MHz is load-bearing rather than a preference - USART1 needs
 * a PCLK of at least 16 x 1 Mbps to reach the bus rate with 16x oversampling,
 * and at 48 MHz BRR lands on exactly 48 with no rounding error at all.
 */
static void clock_init(void)
{
    /* One wait state first: above 24 MHz the flash cannot keep up at zero, and
     * raising the clock before the latency would fetch garbage. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_0) {
        /* the latency takes effect on a delay of its own */
    }

    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) {
    }

    RCC->CR &= ~RCC_CR_HSIDIV; /* HSIDIV = 000: divide by 1 */
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) {
    }

    /* SW = 000 selects HSISYS, which is also the reset value - set explicitly
     * so this function stands on its own if anything ran before it. */
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0u) {
    }

    /* ft_slave.c derives the USART divisor and the TIM3 prescaler from this,
     * so it has to be true and not merely plausible. */
    SystemCoreClock = 48000000u;
}

int main(void)
{
    clock_init();
    SysTick_Config(SystemCoreClock / 1000u);

    app_init();

    for (;;) {
        app_task();
    }
}
