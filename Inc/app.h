/**
 * @file    app.h
 * @brief   Firmware entry points, called from main().
 *
 * The split exists so main.c holds nothing but the clock, SysTick and the loop:
 *
 *     int main(void)
 *     {
 *         clock_init();
 *         SysTick_Config(SystemCoreClock / 1000u);
 *         app_init();
 *         for (;;) {
 *             app_task();
 *         }
 *     }
 *
 * app_init() brings up SPI1, USART1, TIM3 and every GPIO itself, at the
 * register level. Nothing here depends on the HAL or on generated init code.
 */

#ifndef APP_H
#define APP_H

/** Configure the IMU and start serving the Dynamixel bus. */
void app_init(void);

/** One pass of the main loop. Call from the for(;;) in main(). */
void app_task(void);

#endif /* APP_H */
