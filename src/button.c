/*
 * button.c
 *
 *  Created on: Mar 20, 2020
 *      Author: Bryan
 */

#include "button.h"

#include "hal-config.h"
#include "log.h"
#include "events.h"
#include "gpiointerrupt.h"
#include "gatt_db.h"

static uint8_t button_pressed = 0;

/**
 * This function is called when there is an interrupt on the push button pin
 */
static void button_interrupt_callback(uint8_t pin)
{
	if (pin == BSP_BUTTON0_PIN)
	{
		// The button was pressed! Set an event for the main loop to handle
		events_set_event(EVENT_PB0_PRESS);
	}
	else
	{
		LOG_WARN("Unexpected button interrupt callback!");
	}
}

void button_init(void)
{
	GPIOINT_Init();

	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON1_PIN, gpioModeInputPullFilter, 1);

	// Set an interrupt for the rising edge of the PB0 and set the callback function
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, false, true);
	GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, button_interrupt_callback);
}

void button_update_state(void)
{
	button_pressed = !GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN);
}

bool button_get_pushbutton_state(pushbuttons_t button)
{
	if (button == PB0)
	{
		return !GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN);
	}
	else if (button == PB1)
	{
		return !GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON1_PIN);
	}
	else
	{
		LOG_ERROR("Invalid parameter");
		return false;
	}
}
