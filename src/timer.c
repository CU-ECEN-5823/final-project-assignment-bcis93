/*
 * LETIMER0.c
 *
 *  Created on: Jan 29, 2020
 *      Author: Bryan
 */

#include "timer.h"

#include "em_cmu.h"
#include "em_letimer.h"
#include "efr32bg13p632f512gm48.h"
#include "sleep.h"

#include "gpio.h"
#include "log.h"
#include "events.h"
#include "em_core.h"

#define TIMER_PERIOD_MS (1000)

#define LFXO_8K_PRESCALER (4)
#define LFXO_FREQUENCY_8K (8192)

/*
 * Note: the ULFRCO is ideally 1kHz, but the datasheet specifies a potential
 * range of 0.95-1.07 kHz. Through empirical testing, my particular part
 * seems to be about 1.01 kHz. This is why the frequency in the #define below
 * is not exactly 1000
 */
#define ULFRCO_PRESCALER (1)
#define ULFRCO_FREQUENCY_1K (1010)

#define MS_TO_S (1000)
#define US_TO_S (1000000)

static uint32_t frequency = LFXO_FREQUENCY_8K;
static uint32_t overflow_count = 0;

void timer_initialize(SLEEP_EnergyMode_t sleep_blocked)
{
	// Default to using the LFXO
	CMU_Osc_TypeDef osc = cmuOsc_LFXO;
	CMU_Select_TypeDef clock = cmuSelect_LFXO;
	CMU_ClkDiv_TypeDef div = LFXO_8K_PRESCALER;
	frequency = LFXO_FREQUENCY_8K;

	if (sleep_blocked == sleepEM4)
	{
		// If we're running in EM3, use the ULFRCO and its associated values
		osc = cmuOsc_ULFRCO;
		clock = cmuSelect_ULFRCO;
		div = ULFRCO_PRESCALER;
		frequency = ULFRCO_FREQUENCY_1K;
	}

	// Enable oscillator
	CMU_OscillatorEnable(osc, true, true);

	// Select, configure, and enable clock
	CMU_ClockSelectSet(cmuClock_LFA, clock);
	CMU_ClockEnable(cmuClock_LFA, true);
	CMU_ClockDivSet(cmuClock_LETIMER0, div);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	// Initialize LETIMER
	LETIMER_Init_TypeDef init = {
			false,				// enable
			false,				// debugRun
			true,				// comp0Top
			false,				// bufTop
			0,					// out0Pol
			0,					// out1Pol
			letimerUFOANone,	// ufoa0
			letimerUFOANone,	// ufoa1
			letimerRepeatFree,	// repMode
			0					// topValue
	};
	LETIMER_Init(LETIMER0, &init);

	// Set comp0 to the blink period
	LETIMER_CompareSet(LETIMER0, 0, frequency * TIMER_PERIOD_MS / MS_TO_S);

	// Enable interrupts
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);
	NVIC_EnableIRQ(LETIMER0_IRQn);

	// Start the timer!
	LETIMER_Enable(LETIMER0, true);
}

static uint32_t ms_to_ticks(uint32_t ms_wait)
{
	uint32_t ticks = ms_wait * frequency / MS_TO_S;
	if (ticks == 0)
	{
		ticks = 1; // Make sure we wait for at least one tick
	}

	return ticks;
}

static uint32_t getElapsedTicks(uint32_t start, uint32_t current)
{
	if (start >= current)
	{
		return start - current;
	}
	else
	{
		// return the maximum count - the difference between ticks and start
		return start + (frequency * TIMER_PERIOD_MS / MS_TO_S) - current;
	}
}

static uint32_t calculate_end_time(uint32_t start, uint32_t ms_wait)
{
	uint32_t ticks_to_wait = ms_to_ticks(ms_wait);
	if (start >= ticks_to_wait)
	{
		return start - ticks_to_wait;
	}
	else
	{
		// return the maximum count - the difference between ticks and start
		return start + (frequency * TIMER_PERIOD_MS / MS_TO_S) - ms_wait;
	}
}

void timer_start_ms_timer(uint32_t ms_wait)
{
	uint32_t start_time = LETIMER_CounterGet(LETIMER0);
	uint32_t end_time = calculate_end_time(start_time, ms_wait);

	// Set the compare value, clear any pending interrupt, then enable interrupt
	LETIMER_CompareSet(LETIMER0, 1, end_time);
	LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);
}

static uint32_t get_overflow_count(void)
{
	CORE_irqState_t state = CORE_EnterCritical();
	uint32_t count = overflow_count;
	CORE_ExitCritical(state);
	return count;
}

static void increment_overflow_count(void)
{
	CORE_irqState_t state = CORE_EnterCritical();
	overflow_count++;
	CORE_ExitCritical(state);
}

uint32_t timer_get_runtime_ms(void)
{
	uint32_t current_tick = LETIMER_CounterGet(LETIMER0);
	uint32_t elapsed_ticks = getElapsedTicks(0, current_tick); //get ticks elapsed since last overflow

	// return (ms since last overflow) + (period * number of overflows)
	return ms_to_ticks(elapsed_ticks) + TIMER_PERIOD_MS * get_overflow_count();
}

void timer_wait_ms(uint32_t ms_wait)
{
	uint32_t start_time = LETIMER_CounterGet(LETIMER0);
	uint32_t wait_ticks = ms_to_ticks(ms_wait);
	uint32_t ticks_waited = 0;

	while (ticks_waited < wait_ticks)
	{
		ticks_waited = getElapsedTicks(start_time, LETIMER_CounterGet(LETIMER0));
	}
}

void timer_stop(void)
{
	LETIMER_Enable(LETIMER0, false); // turn the timer off
}

void LETIMER0_IRQHandler(void)
{
	// Check which interrupt(s) occurred and immediately clear them
	int flags = LETIMER_IntGet(LETIMER0);
	LETIMER_IntClear(LETIMER0, flags);

	// Handle the various possible interrupts
	if (flags & LETIMER_IF_COMP0)
	{
		events_set_event(EVENT_TIMER_PERIOD_EXPIRED);
		increment_overflow_count();
	}

	if (flags & LETIMER_IF_COMP1)
	{
		events_set_event(EVENT_TIMER_WAIT_DONE);

		// Disable this interrupt until we need it again!
		LETIMER_IntDisable(LETIMER0, LETIMER_IF_COMP1);
	}
}
