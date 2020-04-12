/*
 * LETIMER0.h
 *
 *  Created on: Jan 29, 2020
 *      Author: Bryan
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#include "sleep.h"

#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

/*
 * Initialize and start the timer.
 *
 * Pass in the energy mode used to set up the sleep block so
 * that the timer can decide which oscillator to use.
 */
void timer_initialize(SLEEP_EnergyMode_t sleep_blocked);

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
