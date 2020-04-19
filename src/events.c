/*
 * events.c
 *
 *  Created on: Feb 10, 2020
 *      Author: Bryan
 *
 *  This file is based largely on the SoC - Thermometer example available in AppBuilder.
 *  I also had a lot of help from the BLE API, available here: https://www.silabs.com/documents/public/reference-manuals/bluetooth-api-reference.pdf
 */

#include "events.h"
#include "em_core.h"
#include "sleep.h"
#include "gpio.h"
#include "timer.h"
#include "gatt_db.h"
#include "log.h"
#include "gpio.h"
//#include "gecko_ble_errors.h"
#include "display.h"
//#include "ble_device_type.h"
#include <math.h>
#include "button.h"

static uint32_t events_bitmask = 0;

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

void events_set_event(uint32_t event_mask)
{
	gecko_external_signal(event_mask);
}

void events_clear_event(uint32_t event_mask)
{
	events_bitmask &= ~event_mask;
}

uint32_t events_get_events(void)
{
	return events_bitmask;
}

void events_update_bitmask(uint32_t bitmask)
{
	events_bitmask = bitmask;
}

bool events_get_event(uint32_t event_mask)
{
	return events_bitmask & event_mask;
}
