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
#define RESET_TIMER_ID (0)
#define LPN_TIMER_ID (1)

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
#define DISPLAY_MAX_COUNT (4)

/// Flag for indicating DFU Reset must be performed
static uint8_t boot_to_dfu = 0;

static uint16_t _primary_elem_index = 0; /* For indexing elements of the node */

/// on/off transaction identifier
static uint8_t onoff_trid = 0;

static uint8_t display_count = 0;

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

/// button state
static PACKSTRUCT(struct button_state {
	// On/Off Server state
	uint8_t onoff_current;          /**< Current generic on/off value */
	uint8_t onoff_target;           /**< Target generic on/off value */

	// Transition Time Server state
	uint8_t transtime;              /**< Transition time */

	// On Power Up Server state
	uint8_t onpowerup;              /**< On Power Up value */
}) button_state;

/***************************************************************************//**
 * This function saves the current button state in Persistent Storage so that
 * the data is preserved over reboots and power cycles.
 * The button state is hold in a global variable button_state.
 * A PS key with ID 0x4004 is used to store the whole struct.
 *
 * @return 0 if saving succeed, -1 if saving fails.
 ******************************************************************************/
static int button_state_store(void)
{
	struct gecko_msg_flash_ps_save_rsp_t* pSave;

	pSave = gecko_cmd_flash_ps_save(0x4004, sizeof(struct button_state), (const uint8_t*)&button_state);

	if (pSave->result) {
		LOG_WARN("button_state_store(): PS save failed, code %x", pSave->result);
		return(-1);
	}

	return 0;
}

static void button_state_changed(void)
{
	button_state_store();
}


/***************************************************************************//**
 * Update generic on/off state.
 *
 * @param[in] element_index  Server model element index.
 * @param[in] remaining_ms   The remaining time in milliseconds.
 *
 * @return Status of the update operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
static errorcode_t onoff_update(uint16_t element_index, uint32_t remaining_ms)
{
	struct mesh_generic_state current, target;

	current.kind = mesh_generic_state_on_off;
	current.on_off.on = button_state.onoff_current;

	target.kind = mesh_generic_state_on_off;
	target.on_off.on = button_state.onoff_target;

	return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
			element_index,
			&current,
			&target,
			remaining_ms);
}


/***************************************************************************//**
 * Update generic on/off state and publish model state to the network.
 *
 * @param[in] element_index  Server model element index.
 * @param[in] remaining_ms   The remaining time in milliseconds.
 *
 * @return Status of the update and publish operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
static errorcode_t onoff_update_and_publish(uint16_t element_index,
		uint32_t remaining_ms)
{
	errorcode_t e;

	e = onoff_update(element_index, remaining_ms);
	if (e == bg_err_success) {
		e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
				element_index,
				mesh_generic_state_on_off);
	}

	return e;
}


/***************************************************************************//**
 * Response to generic on/off request.
 *
 * @param[in] element_index  Server model element index.
 * @param[in] client_addr    Address of the client model which sent the message.
 * @param[in] appkey_index   The application key index used in encrypting.
 * @param[in] remaining_ms   The remaining time in milliseconds.
 *
 * @return Status of the response operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
static errorcode_t onoff_response(uint16_t element_index,
		uint16_t client_addr,
		uint16_t appkey_index,
		uint32_t remaining_ms)
{
	struct mesh_generic_state current, target;

	current.kind = mesh_generic_state_on_off;
	current.on_off.on = button_state.onoff_current;

	target.kind = mesh_generic_state_on_off;
	target.on_off.on = button_state.onoff_target;

	return mesh_lib_generic_server_response(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
			element_index,
			client_addr,
			appkey_index,
			&current,
			&target,
			remaining_ms,
			0x00);
}

/***************************************************************************//**
 * This function process the requests for the generic on/off model.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] client_addr    Address of the client model which sent the message.
 * @param[in] server_addr    Address the message was sent to.
 * @param[in] appkey_index   The application key index used in encrypting the request.
 * @param[in] request        Pointer to the request structure.
 * @param[in] transition_ms  Requested transition time (in milliseconds).
 * @param[in] delay_ms       Delay time (in milliseconds).
 * @param[in] request_flags  Message flags. Bitmask of the following:
 *                           - Bit 0: Nonrelayed. If nonzero indicates
 *                                    a response to a nonrelayed request.
 *                           - Bit 1: Response required. If nonzero client
 *                                    expects a response from the server.
 ******************************************************************************/
static void onoff_request(uint16_t model_id,
		uint16_t element_index,
		uint16_t client_addr,
		uint16_t server_addr,
		uint16_t appkey_index,
		const struct mesh_generic_request *request,
		uint32_t transition_ms,
		uint16_t delay_ms,
		uint8_t request_flags)
{
	LOG_INFO("ON/OFF request: requested state=<%s>, transition=%lu, delay=%u",
			request->on_off ? "ON" : "OFF", transition_ms, delay_ms);

	if (button_state.onoff_current == request->on_off) {
		LOG_INFO("Request for current state received; no op");
	} else {
		LOG_INFO("Turning Button <%s>", request->on_off ? "ON" : "OFF");
		displayPrintf(DISPLAY_ROW_ACTION, "Button %s", request->on_off ? "pressed" : "released");

		button_state.onoff_current = request->on_off;
		button_state.onoff_target = request->on_off;

		button_state_changed();
	}

	uint32_t remaining_ms = delay_ms + transition_ms;
	if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
		onoff_response(element_index, client_addr, appkey_index, remaining_ms);
	}
	onoff_update_and_publish(element_index, remaining_ms);
}

/***************************************************************************//**
 * This function is called when a button on/off request
 * with non-zero transition time has completed.
 ******************************************************************************/
static void onoff_transition_complete(void)
{
	// transition done -> set state, update and publish
	button_state.onoff_current = button_state.onoff_target;

	LOG_INFO("transition complete. New state is %s", button_state.onoff_current ? "ON" : "OFF");

	button_state_changed();
	onoff_update_and_publish(_primary_elem_index, 0);
}


/*******************************************************************************
 * Button state initialization.
 * This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 ******************************************************************************/
void button_state_init(void)
{
	_primary_elem_index = 0;   // index of primary element is zero.

	memset(&button_state, 0, sizeof(struct button_state));

	LOG_INFO("On power up state is OFF");
	button_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
	button_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_OFF;


}

/***************************************************************************//**
 * This function is a handler for generic on/off change event.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] current        Pointer to current state structure.
 * @param[in] target         Pointer to target state structure.
 * @param[in] remaining_ms   Time (in milliseconds) remaining before transition
 *                           from current state to target state is complete.
 ******************************************************************************/
static void onoff_change(uint16_t model_id,
		uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target,
		uint32_t remaining_ms)
{
	if (current->on_off.on != button_state.onoff_current) {
		LOG_INFO("on-off state changed %u to %u", button_state.onoff_current, current->on_off.on);

		button_state.onoff_current = current->on_off.on;
		button_state_changed();
	} else {
		LOG_INFO("dummy onoff change - same state as before");
	}
}

/***************************************************************************//**
 * This function is a handler for generic on/off recall event.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] current        Pointer to current state structure.
 * @param[in] target         Pointer to target state structure.
 * @param[in] transition_ms  Transition time (in milliseconds).
 ******************************************************************************/
static void onoff_recall(uint16_t model_id,
		uint16_t element_index,
		const struct mesh_generic_state *current,
		const struct mesh_generic_state *target,
		uint32_t transition_ms)
{
	LOG_INFO("Generic On/Off recall");
	if (transition_ms == 0) {
		button_state.onoff_target = current->on_off.on;
	} else {
		button_state.onoff_target = target->on_off.on;
	}

	if (button_state.onoff_current == button_state.onoff_target) {
		LOG_INFO("Request for current state received; no op");
	} else {
		LOG_INFO("recall ON/OFF state <%s> with transition=%lu ms",
				button_state.onoff_target ? "ON" : "OFF",
						transition_ms);

		if (transition_ms == 0) {
			button_state.onoff_current = current->on_off.on;
		} else {
			if (button_state.onoff_target == MESH_GENERIC_ON_OFF_STATE_ON) {
				button_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_ON;
			}
			onoff_transition_complete();    }
		button_state_changed();
	}

	onoff_update_and_publish(element_index, transition_ms);
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
		LOG_ERROR("LPN init failed (0x%x)\r\n", result);
		return;
	}
	lpn_active = 1;
	LOG_INFO("LPN initialized\r\n");
	//DI_Print("LPN on", DI_ROW_LPN);

	// Configure LPN Minimum friend queue length = 2
	result = gecko_cmd_mesh_lpn_config(mesh_lpn_queue_length, 2)->result;
	if (result) {
		LOG_ERROR("LPN queue configuration failed (0x%x)\r\n", result);
		return;
	}
	// Configure LPN Poll timeout = 5 seconds
	result = gecko_cmd_mesh_lpn_config(mesh_lpn_poll_timeout, 5 * 1000)->result;
	if (result) {
		LOG_ERROR("LPN Poll timeout configuration failed (0x%x)\r\n", result);
		return;
	}
	LOG_INFO("trying to find friend...\r\n");
	result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (result != 0) {
		LOG_WARN("ret.code 0x%x\r\n", result);
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
//  result = gecko_cmd_hardware_set_soft_timer(TIMER_STOP,
//                                             TIMER_ID_FRIEND_FIND,
//                                             1)->result;

  // Terminate friendship if exist
  result = gecko_cmd_mesh_lpn_terminate_friendship()->result;
  if (result) {
    LOG_ERROR("Friendship termination failed (0x%x)\r\n", result);
  }
  // turn off lpn feature
  result = gecko_cmd_mesh_lpn_deinit()->result;
  if (result) {
    LOG_ERROR("LPN deinit failed (0x%x)\r\n", result);
  }
  lpn_active = 0;
  LOG_INFO("LPN deinitialized\r\n");
  //DI_Print("LPN off", DI_ROW_LPN);
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
		timer_initialize();
		i2c_init();
		veml6075_init();
		veml6075_begin(VEML6075_100MS, false, true);
		veml6075_enable(true);

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
			struct gecko_msg_system_get_bt_address_rsp_t* rsp = gecko_cmd_system_get_bt_address();
			char name[NAME_LENGTH] = {}; // initialize to all zeros
			sprintf(name, "%s %02X%02X", NAME_PREFIX, rsp->address.addr[0], rsp->address.addr[1]);
			LOG_INFO("Device name: %s", name);

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

			lpn_init();

			if (DeviceUsesClientModel())
			{
				BTSTACK_CHECK_RESPONSE(
						gecko_cmd_mesh_generic_client_init());
			}
			if (DeviceUsesServerModel())
			{
				BTSTACK_CHECK_RESPONSE(
						gecko_cmd_mesh_generic_server_init());
			}

			if (DeviceIsOnOffPublisher())
			{
				mesh_lib_init(malloc,free,8);
			}
			if (DeviceIsOnOffSubscriber())
			{
				mesh_lib_init(malloc,free,9);

				button_state_init();
				//    		  mesh_lib_generic_server_register_handler();
				//    		  mesh_lib_generic_server_update();
				//    		  mesh_lib_generic_server_publish();
				errorcode_t err = mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
						0,
						onoff_request,
						onoff_change,
						onoff_recall);
				if (err)
				{
					LOG_WARN("register handler returned 0x%X", err);
				}
				onoff_update_and_publish(_primary_elem_index, 0);
			}
		}
		break;

	case gecko_evt_mesh_node_provisioning_started_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Provisioning");
		LOG_INFO("Provisioning");
		break;

	case gecko_evt_mesh_node_provisioned_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
		LOG_INFO("Provisioned");



		mesh_lib_init(malloc,free,9);
		button_state_init();

		mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
				0,
				onoff_request,
				onoff_change,
				onoff_recall);

		onoff_update_and_publish(_primary_elem_index, 0);
		button_state_changed();

		//lpn_init();
		gecko_cmd_hardware_set_soft_timer(INIT_LPN_TIMEOUT, LPN_TIMER_ID, 1);
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Provision failed");
		LOG_INFO("Provision failed with result %d", evt->data.evt_mesh_node_provisioning_failed.result);
		break;

	case gecko_evt_mesh_generic_server_client_request_id:
		LOG_INFO("Server client request id");
		if (DeviceUsesServerModel())
		{
			mesh_lib_generic_server_event_handler(evt);
		}
		break;

	case gecko_evt_mesh_generic_server_state_changed_id:
		LOG_INFO("Server state changed");
		if (DeviceUsesServerModel())
		{
			mesh_lib_generic_server_event_handler(evt);
		}
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

			//			if (DeviceIsOnOffPublisher())
			//			{
			//				struct mesh_generic_request req;
			//				const uint32_t transtime = 0; // using zero transition time by default
			//
			//				req.kind = mesh_generic_request_on_off;
			//				req.on_off = MESH_GENERIC_ON_OFF_STATE_ON;
			//
			//
			//				LOG_DEBUG("Publishing button state (on)");
			//				errorcode_t err = mesh_lib_generic_client_publish(
			//						MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
			//						_primary_elem_index,
			//						onoff_trid,
			//						&req,
			//						transtime, // transition time in ms
			//						50,
			//						0x00   // flags
			//				);
			//				if (err)
			//				{
			//					LOG_WARN("client publish failed with 0x%X", err);
			//				}
			//
			//				onoff_trid++;
			//			}
		}
		if (events_get_event(EVENT_PB0_RELEASE))
		{
			events_clear_event(EVENT_PB0_RELEASE);
			LOG_DEBUG("Button released");

			//			if (DeviceIsOnOffPublisher())
			//			{
			//				struct mesh_generic_request req;
			//				const uint32_t transtime = 0; // using zero transition time by default
			//
			//				req.kind = mesh_generic_request_on_off;
			//				req.on_off = MESH_GENERIC_ON_OFF_STATE_OFF;
			//
			//
			//				LOG_DEBUG("Publishing button state (off)");
			//				errorcode_t err = mesh_lib_generic_client_publish(
			//						MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
			//						_primary_elem_index,
			//						onoff_trid,
			//						&req,
			//						transtime, // transition time in ms
			//						50,
			//						0x00   // flags
			//				);
			//				if (err)
			//				{
			//					LOG_WARN("client publish failed with 0x%X", err);
			//				}
			//
			//				onoff_trid++;
			//			}
			if (!displayEnabled())
			{
				displayInit();
			}
			display_count = DISPLAY_MAX_COUNT;
		}

		if (events_get_event(EVENT_TIMER_PERIOD_EXPIRED))
		{
			if (display_count > 0)
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

	}
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
