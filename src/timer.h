/*
 * LETIMER0.h
 *
 *  Created on: Jan 29, 2020
 *      Author: Bryan
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#include <stdint.h>

#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

/*
 * Initialize and start the timer.
 */
void timer_initialize();

/**
 * Wait for ms_wait amount of time
 */
void timer_wait_ms(uint32_t ms_wait);

/**
 * Start a timer that will expire ms_wait from now
 */
void timer_start_ms_timer(uint32_t ms_wait);

/**
 * Returns ms since the timer started
 */
uint32_t timer_get_runtime_ms(void);

/**
 * Stop the timer!
 */
void timer_stop(void);

#endif /* SRC_TIMER_H_ */
