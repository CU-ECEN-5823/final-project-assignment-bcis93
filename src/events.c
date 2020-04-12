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

void events_update(struct gecko_cmd_packet *evt)
{
	switch(BGLIB_MSG_ID(evt->header))
	{

		/* This boot event is generated when the system boots up after reset.
		 * Do not call any stack commands before receiving the boot event.
		 * Here the system is set to start discovering immediately after boot procedure. */
	case gecko_evt_system_boot_id:

		logInit();
		displayInit();
		button_init();
		break;

	case gecko_evt_le_connection_opened_id:
		break;

		// This event happens when an external event (some of our user-defined events) occur
	case gecko_evt_system_external_signal_id:
		// copy the signals to a local global
		events_bitmask = evt->data.evt_system_external_signal.extsignals;

		// If a button press has occurred, handle it!
		if (events_get_event(EVENT_PB0_PRESS))
		{
			events_clear_event(EVENT_PB0_PRESS);
			LOG_DEBUG("Button press");
		}

		break;

	case gecko_evt_le_connection_closed_id:
		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		} else {

		}
		break;

		/* Events related to OTA upgrading
				         ----------------------------------------------------------------------------- */
		/* Checks if the user-type OTA Control Characteristic was written.
		 * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
	case gecko_evt_gatt_server_user_write_request_id:
		if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
			/* Set flag to enter to OTA mode */
			boot_to_dfu = 1;
			/* Send response to Write Request */
			gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

			/* Close connection to enter to DFU OTA mode */
			gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;

	default:
		break;
	}
}

