/**
 * ECEN5823
 *
 * This file is based on app.c from the soc-btmesh-light example provided by SiLabs.
 *
 * I also used elements from lightbulb.c fromt he same project, as well as code from
 * main.c in SiLabs' soc-btmesh-switch example project
 *
 * - Bryan Cisneros
 */

/***************************************************************************//**
 * @file
 * @brief Application code
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* C Standard Library headers */
#include <stdio.h>

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "mesh_generic_model_capi_types.h"
#include "mesh_lib.h"

/* GPIO peripheral library */
#include <em_gpio.h>

/* Own header */
#include "app.h"

#include "log.h"
#include "src/display.h"
#include "button.h"
#include "events.h"
#include "ble_mesh_device_type.h"
#include "gecko_ble_errors.h"
#include "timer.h"
#include "VEML6075.h"
#include "i2c.h"

#define SOFT_TIMER_1_SEC (32768)
#define FACTORY_RESET_TIMEOUT (SOFT_TIMER_1_SEC * 2) // 2 seconds
#define INIT_LPN_TIMEOUT (SOFT_TIMER_1_SEC * 30) // 30 seconds
#define RETRY_FRIENDSHIP_TIMEOUT (SOFT_TIMER_1_SEC * 2) // 2 seconds
#define DEFAULT_PUBLISH_TIMEOUT (SOFT_TIMER_1_SEC * 2) // 2 seconds

#define RESET_TIMER_ID (0)
#define LPN_TIMER_ID (1)
#define FRIEND_FIND_TIMER_ID (2)
#define PUBLISH_TIMER_ID (3)

#define UV_CONFIGURATION_KEY (0x4008)
#define DEFAULT_MEASUREMENT_INTERVAL_S (1)
#define DEFAULT_UV_ALERT_THRESHOLD (60) // 60% of max uv
#define DEFAULT_UV_CLEAR_THRESHOLD (40) // 40% of max uv

#define MAX_UV (11)

#define PUBLISH_LEVEL 1
#define PUBLISH_ON_OFF 0

#define USE_TEST_VALUES (0) // set this to use very low thresholds (for testing indoors)
#define TEST_UV_ALERT_THRESHOLD (0.2)
#define TEST_UV_CLEAR_THRESHOLD (0.1)

#define ALERT (0xFFFF)
#define CLEAR_ALERT (0x7FFF)

#define NAME_LENGTH (13)
#if DEVICE_IS_ONOFF_PUBLISHER
#define NAME_PREFIX "5823Pub"
#define DEVICE_TYPE "Publisher"
#else
#define NAME_PREFIX "5823Sub"
#define DEVICE_TYPE "Subscriber"
#endif

#define ADDR_LENGTH (6) // length of address is 6
#define STR_BYTES_PER_ADDRESS_BYTE (3) // we need 3 bytes per each address byte (2 to store the hex value in string form + 1 for a colon)
#define ADDR_BUFFER_LENGTH (18) // ADDR_LENGTH * STR_BYTES_PER_ADDRESS_BYTE
#define DISPLAY_MAX_COUNT (6)

/// Flag for indicating DFU Reset must be performed
static uint8_t boot_to_dfu = 0;

static uint16_t _primary_elem_index = 0; /* For indexing elements of the node */

static uint8_t display_count = 0;

static uint32_t publish_timeout = DEFAULT_PUBLISH_TIMEOUT;

static bool friend_connected = false;
static bool provisioned = false;

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/*******************************************************************************
 * Initialize used bgapi classes for server.
 ******************************************************************************/
void gecko_bgapi_classes_init_server_friend(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	//gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	//gecko_bgapi_class_mesh_lpn_init();
	gecko_bgapi_class_mesh_friend_init();
	gecko_bgapi_class_mesh_lc_server_init();
	gecko_bgapi_class_mesh_lc_setup_server_init();
	gecko_bgapi_class_mesh_scene_server_init();
	gecko_bgapi_class_mesh_scene_setup_server_init();
}


/*******************************************************************************
 * Initialize used bgapi classes for server.
 ******************************************************************************/
void gecko_bgapi_classes_init_client_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	gecko_bgapi_class_mesh_generic_client_init();
	//gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();
	gecko_bgapi_class_mesh_scene_client_init();
}

static PACKSTRUCT(struct uv_configuration {
	uint8_t uv_alert_threshold; // threshold level to alert at
	uint8_t uv_clear_threshold; // threshold level to clear alert
	uint8_t measurement_interval_s; // how often to take a measurement, in seconds
}) uv_configuration;

static int uv_configuration_store(void)
{
	struct gecko_msg_flash_ps_save_rsp_t* pSave;

	pSave = gecko_cmd_flash_ps_save(UV_CONFIGURATION_KEY, sizeof(struct uv_configuration), (const uint8_t*)&uv_configuration);

	if (pSave->result) {
		LOG_WARN("PS save failed, code %x", pSave->result);
		return(-1);
	}

	return 0;
}

void publish_uvi(float uvi)
{
	static uint8_t uvi_trid;
	struct mesh_generic_request req;
	const uint32_t transtime = 0; // using zero transition time by default

	LOG_DEBUG("Publishing UVI: %f", uvi);

	float uv_alert_level = uv_configuration.uv_alert_threshold;
	float uv_clear_level = uv_configuration.uv_clear_threshold;
#if USE_TEST_VALUES
	uv_alert_level = TEST_UV_ALERT_THRESHOLD;
	uv_clear_level = TEST_UV_CLEAR_THRESHOLD;
#endif // USE_TEST_VALUES

#if PUBLISH_LEVEL
	static bool alert = false;
	static bool second_alert_sent = false;
	static bool second_cleared_sent = false;

	req.kind = mesh_generic_request_level;
	req.level = (int16_t) (uvi * 100 / MAX_UV); // we will report in percent of MAX_UV

	if (uvi > uv_alert_level)
	{
		LOG_WARN("Alert! UVI at %f", uvi);
		if (!alert)
		{
			alert = true;
			second_cleared_sent = false;
			req.level = ALERT;
			LOG_INFO("Sending alert");
		}
		else if (!second_alert_sent)
		{
			second_alert_sent = true;
			req.level = ALERT;
			LOG_INFO("Sending alert");
		}
	}
	else if (uvi < uv_clear_level)
	{
		if (alert)
		{
			alert = false;
			second_alert_sent = false;
			req.level = CLEAR_ALERT;
			LOG_INFO("Clearing alert");
		}
		else if (!second_cleared_sent)
		{
			second_cleared_sent = true;
			req.level = CLEAR_ALERT;
			LOG_INFO("Clearing alert");
		}
	}


	errorcode_t err = mesh_lib_generic_client_publish(
			MESH_GENERIC_LEVEL_CLIENT_MODEL_ID,
			_primary_elem_index,
			uvi_trid,
			&req,
			transtime, // transition time in ms
			50,
			0x00   // flags
	);
#endif // PUBLISH_LEVEL
#if PUBLISH_ON_OFF
	req.kind = mesh_generic_request_on_off;

	if (uvi > uv_alert_level)
	{
		req.on_off = MESH_GENERIC_ON_OFF_STATE_ON;
	}
	else
	{
		req.on_off = MESH_GENERIC_ON_OFF_STATE_OFF;
	}

	LOG_INFO("publishing %d", req.on_off);

	errorcode_t err = mesh_lib_generic_client_publish(
			MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
			_primary_elem_index,
			uvi_trid,
			&req,
			transtime, // transition time in ms
			50,
			0x00   // flags
	);
#endif

	if (err)
	{
		LOG_WARN("client publish failed with 0x%X", err);
	}

	uvi_trid++;
}

uint8_t lpn_active = 0;
uint8_t num_connections = 0;

/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 ******************************************************************************/
void lpn_init(void)
{
	uint16_t result;

	// Do not initialize LPN if lpn is currently active
	// or any GATT connection is opened
	if (lpn_active || num_connections) {
		return;
	}

	// Initialize LPN functionality.
	result = gecko_cmd_mesh_lpn_init()->result;
	if (result) {
		LOG_ERROR("LPN init failed (0x%x)", result);
		return;
	}
	lpn_active = 1;
	LOG_INFO("LPN initialized");

	// Configure LPN Minimum friend queue length = 2
	result = gecko_cmd_mesh_lpn_config(mesh_lpn_queue_length, 2)->result;
	if (result) {
		LOG_ERROR("LPN queue configuration failed (0x%x)", result);
		return;
	}
	// Configure LPN Poll timeout = 5 seconds
	result = gecko_cmd_mesh_lpn_config(mesh_lpn_poll_timeout, 5 * 1000)->result;
	if (result) {
		LOG_ERROR("LPN Poll timeout configuration failed (0x%x)", result);
		return;
	}
	LOG_INFO("trying to find friend...");
	result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (result != 0) {
		LOG_WARN("ret.code 0x%x", result);
	}
}

/***************************************************************************//**
 * Deinitialize LPN functionality.
 ******************************************************************************/
void lpn_deinit(void)
{
  uint16_t result;

  if (!lpn_active) {
    return; // lpn feature is currently inactive
  }

  // Cancel friend finding timer
  gecko_cmd_hardware_set_soft_timer(0, FRIEND_FIND_TIMER_ID, 1);

  // Terminate friendship if exist
  result = gecko_cmd_mesh_lpn_terminate_friendship()->result;
  if (result) {
    LOG_ERROR("Friendship termination failed (0x%x)", result);
  }
  // turn off lpn feature
  result = gecko_cmd_mesh_lpn_deinit()->result;
  if (result) {
    LOG_ERROR("LPN deinit failed (0x%x)", result);
  }
  lpn_active = 0;
  LOG_INFO("LPN deinitialized");
}

static void print_name_and_address()
{
	struct gecko_msg_system_get_bt_address_rsp_t* rsp = gecko_cmd_system_get_bt_address();

	displayPrintf(DISPLAY_ROW_NAME, DEVICE_TYPE);

	uint8_t addr_buf[ADDR_BUFFER_LENGTH] = {}; // Initialize to all 0s

	// convert the raw addresses into printable strings
	for (uint8_t i = 0; i < ADDR_LENGTH; i++)
	{
		sprintf((void*)&addr_buf[i*STR_BYTES_PER_ADDRESS_BYTE], "%02X:", rsp->address.addr[i]);
	}
	// make sure the string is null terminated
	addr_buf[ADDR_BUFFER_LENGTH - 1] = 0x00;

	// print the address of the client and server to the display
	displayPrintf(DISPLAY_ROW_BTADDR, "%s", addr_buf);
}

static void print_friend_status()
{
	if (friend_connected)
	{
		displayPrintf(DISPLAY_ROW_CLIENTADDR, "friend connected");
	}
	else
	{
		displayPrintf(DISPLAY_ROW_CLIENTADDR, "No friend connected");
	}
}

static void print_provisioned_status()
{
	if (provisioned)
	{
		displayPrintf(DISPLAY_ROW_CONNECTION, "Provisioned");
	}
}

/*******************************************************************************
 * Handling of stack events. Both Bluetooth LE and Bluetooth mesh events
 * are handled here.
 * @param[in] evt_id  Incoming event ID.
 * @param[in] evt     Pointer to incoming event.
 ******************************************************************************/
/**************************** INSTRUCTIONS ************************************
 * 1. Before proceeding with assignment profile the project with attached blue
 * gecko and verify if it is being scanned by mobile mesh App.
 * 2. Use Bluetooth Mesh app from Silicon labs for the same and if you are not
 * able to get the app working checkout nRF Mesh App on play store.
 * 3. Add the necessary events for the mesh in switch (evt_id) similar to the
 * BLE assignments.
 * 4. Use the following pdf for reference
 * https://www.silabs.com/documents/public/reference-manuals/bluetooth-le-and-mesh-software-api-reference-manual.pdf
 * 5. Remember to check and log the return status for every Mesh API used.
 * 6. You can take the hints from light and switch example for mesh to know which
 * commands and events are needed and to understand the flow.
 ******************************************************************************/
void handle_ecen5823_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	if (NULL == evt) {
		return;
	}

	switch (evt_id) {
	case gecko_evt_system_boot_id:
		logInit();
		displayInit();
		display_count = DISPLAY_MAX_COUNT;
		button_init();

		// check if a button is pressed. If so, do a factory reset!
		if (button_get_pushbutton_state(PB0) || button_get_pushbutton_state(PB1))
		{
			displayPrintf(DISPLAY_ROW_ACTION, "Factory Reset");
			LOG_INFO("Factory reset!");
			gecko_cmd_flash_ps_erase_all();

			// set a timer for 2 seconds
			gecko_cmd_hardware_set_soft_timer(FACTORY_RESET_TIMEOUT, 0, true);
		}
		else // regular boot up
		{
			timer_initialize();
			i2c_init();
			veml6075_init();
			veml6075_enable(true);

			struct gecko_msg_system_get_bt_address_rsp_t* rsp = gecko_cmd_system_get_bt_address();
			char name[NAME_LENGTH] = {}; // initialize to all zeros
			sprintf(name, "%s %02X%02X", NAME_PREFIX, rsp->address.addr[0], rsp->address.addr[1]);
			LOG_INFO("Device name: %s", name);

			print_name_and_address();

			struct gecko_msg_gatt_server_write_attribute_value_rsp_t* response = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, NAME_LENGTH, name);
			if (response->result)
			{
				LOG_WARN("write attribute failed with 0x%X", response->result);
			}
			BTSTACK_CHECK_RESPONSE(
					gecko_cmd_mesh_node_init());

		}
		break;

	case gecko_evt_hardware_soft_timer_id:
		LOG_DEBUG("Timer %d event", evt->data.evt_hardware_soft_timer.handle);

		if (evt->data.evt_hardware_soft_timer.handle == RESET_TIMER_ID)
		{
			// reset!
			gecko_cmd_system_reset(0);
		}
		else if (evt->data.evt_hardware_soft_timer.handle == LPN_TIMER_ID)
		{
			if (!lpn_active) {
				LOG_INFO("trying to initialize lpn...");
				lpn_init();
			}
		}
		else if (evt->data.evt_hardware_soft_timer.handle == FRIEND_FIND_TIMER_ID)
		{
			LOG_INFO("trying to find friend...");
			BTSTACK_CHECK_RESPONSE(
					gecko_cmd_mesh_lpn_establish_friendship(0));
		}
		else if (evt->data.evt_hardware_soft_timer.handle == PUBLISH_TIMER_ID)
		{
			LOG_INFO("Publishing UV light data");
			publish_uvi(veml6075_get_last_uvi());
		}

		break;

	case gecko_evt_mesh_node_initialized_id:
		if(!evt->data.evt_mesh_node_initialized.provisioned)
		{
			BTSTACK_CHECK_RESPONSE(
					gecko_cmd_mesh_generic_client_init());
			BTSTACK_CHECK_RESPONSE(
					gecko_cmd_mesh_scene_client_init(0));
			gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
		}
		else
		{
			displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
			LOG_INFO("Already provisioned");
			provisioned = true;

			lpn_init();

			gecko_cmd_hardware_set_soft_timer(publish_timeout, PUBLISH_TIMER_ID, false);

			struct gecko_msg_flash_ps_load_rsp_t* rsp = gecko_cmd_flash_ps_load(UV_CONFIGURATION_KEY);
			if (rsp->result)
			{
				LOG_WARN("PS load failed with code 0x%X", rsp->result);
				uv_configuration.measurement_interval_s = DEFAULT_MEASUREMENT_INTERVAL_S;
				uv_configuration.uv_alert_threshold = DEFAULT_UV_ALERT_THRESHOLD;
				uv_configuration.uv_clear_threshold = DEFAULT_UV_CLEAR_THRESHOLD;
			}
			else
			{
				LOG_INFO("PS load successful");
				memcpy(&uv_configuration, rsp->value.data, sizeof(struct uv_configuration));
			}

			BTSTACK_CHECK_RESPONSE(
					gecko_cmd_mesh_generic_client_init());

			mesh_lib_init(malloc,free,8);
		}
		break;

	case gecko_evt_mesh_node_provisioning_started_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Provisioning");
		LOG_INFO("Provisioning");
		break;

	case gecko_evt_mesh_node_provisioned_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
		LOG_INFO("Provisioned");

		provisioned = true;

		mesh_lib_init(malloc,free,9);

		gecko_cmd_hardware_set_soft_timer(INIT_LPN_TIMEOUT, LPN_TIMER_ID, 1);
		gecko_cmd_hardware_set_soft_timer(publish_timeout, PUBLISH_TIMER_ID, false);

		uv_configuration.measurement_interval_s = DEFAULT_MEASUREMENT_INTERVAL_S;
		uv_configuration.uv_alert_threshold = DEFAULT_UV_ALERT_THRESHOLD;
		uv_configuration.uv_clear_threshold = DEFAULT_UV_CLEAR_THRESHOLD;
		uv_configuration_store();
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Provision failed");
		LOG_INFO("Provision failed with result 0x%X", evt->data.evt_mesh_node_provisioning_failed.result);
		break;

	case gecko_evt_mesh_generic_server_client_request_id:
		LOG_INFO("Server client request id");
		break;

	case gecko_evt_mesh_generic_server_state_changed_id:
		LOG_INFO("Server state changed");
		break;

	case gecko_evt_mesh_node_model_config_changed_id:
		LOG_INFO("model config changed");
		break;

	case gecko_evt_mesh_node_reset_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Factory Reset");
		LOG_INFO("Factory reset by provisioner!");
		gecko_cmd_flash_ps_erase_all();

		// set a timer for 2 seconds
		gecko_cmd_hardware_set_soft_timer(FACTORY_RESET_TIMEOUT, RESET_TIMER_ID, true);
		break;

	case gecko_evt_le_connection_opened_id:
		displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
		LOG_INFO("Connected!");
		num_connections++;
		// turn off lpn feature after GATT connection is opened
		lpn_deinit();
		break;

	case gecko_evt_le_connection_closed_id:
		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		}
		else
		{
			displayPrintf(DISPLAY_ROW_CONNECTION, "");
			LOG_INFO("Disconnected!");
			if (num_connections > 0)
			{
				num_connections--;
				if (num_connections == 0)
				{
					lpn_init();
				}
			}
		}
		break;

	case gecko_evt_system_external_signal_id:
		LOG_DEBUG("external event! %d", evt->data.evt_system_external_signal.extsignals);
		events_update_bitmask(evt->data.evt_system_external_signal.extsignals);

		// If a button press has occurred, handle it!
		if (events_get_event(EVENT_PB0_PRESS))
		{
			events_clear_event(EVENT_PB0_PRESS);
			LOG_DEBUG("Button pressed");
		}
		if (events_get_event(EVENT_PB0_RELEASE))
		{
			events_clear_event(EVENT_PB0_RELEASE);
			LOG_DEBUG("Button released");

			if (!displayEnabled())
			{
				displayInit();
				print_name_and_address();
				print_friend_status();
				print_provisioned_status();
			}
			display_count = DISPLAY_MAX_COUNT;
		}

		if (events_get_event(EVENT_TIMER_PERIOD_EXPIRED))
		{
			if (provisioned && display_count > 0)
			{
				display_count--;
				if (display_count == 0)
				{
					LOG_INFO("Turning off display");
					displayDeinit();
				}
			}
			if (displayEnabled())
			{
				displayUpdate();
			}
		}

		if (events_get_events())
		{
			veml6075_run();
		}
		break;

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

	case gecko_evt_mesh_lpn_friendship_established_id:
		LOG_INFO("friendship established");
		friend_connected = true;
		print_friend_status();
		break;

	case gecko_evt_mesh_lpn_friendship_failed_id:
		LOG_INFO("friendship failed with code 0x%X", evt->data.evt_mesh_lpn_friendship_failed.reason);
		friend_connected = false;
		print_friend_status();

		// try again in a few seconds
		gecko_cmd_hardware_set_soft_timer(RETRY_FRIENDSHIP_TIMEOUT, FRIEND_FIND_TIMER_ID, 1);
		break;

	case gecko_evt_mesh_lpn_friendship_terminated_id:
		LOG_INFO("friendship terminated");
		friend_connected = false;
		print_friend_status();

		if (num_connections == 0) {
			// try again in a few seconds
			gecko_cmd_hardware_set_soft_timer(RETRY_FRIENDSHIP_TIMEOUT, FRIEND_FIND_TIMER_ID, 1);
		}
		break;
	}
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
