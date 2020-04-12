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

#define SOFT_TIMER_1_SEC (32768)
#define FACTORY_RESET_TIMEOUT (SOFT_TIMER_1_SEC * 2) // 2 seconds

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

/// Flag for indicating DFU Reset must be performed
static uint8_t boot_to_dfu = 0;

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
    		struct gecko_msg_system_get_bt_address_rsp_t* rsp = gecko_cmd_system_get_bt_address();
    		char name[NAME_LENGTH] = {}; // initialize to all zeros
    		sprintf(name, "%s %02X%02X", NAME_PREFIX, rsp->address.addr[0], rsp->address.addr[1]);
    		LOG_DEBUG("Device name: %s", name);

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

    		gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, NAME_LENGTH, name);
    		gecko_cmd_mesh_node_init();

    	}
      break;

    case gecko_evt_hardware_soft_timer_id:
    	LOG_DEBUG("Timer %d event", evt->data.evt_hardware_soft_timer.handle);
    	// reset!
    	gecko_cmd_system_reset(0);
    	break;

    case gecko_evt_mesh_node_initialized_id:
      if(!evt->data.evt_mesh_node_initialized.provisioned)
    	  gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
      break;

    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }
      break;

    case gecko_evt_system_external_signal_id:
    	events_update_bitmask(evt->data.evt_system_external_signal.extsignals);
    	// If a button press has occurred, handle it!
    	if (events_get_event(EVENT_PB0_PRESS))
    	{
    		events_clear_event(EVENT_PB0_PRESS);
    		LOG_DEBUG("Button press");
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
