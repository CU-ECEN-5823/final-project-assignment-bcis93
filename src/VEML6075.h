/*
 * VEML6075.h
 *
 *  Created on: Apr 17, 2020
 *      Author: Bryan
 */

#ifndef SRC_VEML6075_H_
#define SRC_VEML6075_H_

#include <stdbool.h>
#include <stdint.h>

/*!
 * @file Adafruit_VEML6075.h
 *
 * Designed specifically to work with the VEML6075 sensor from Adafruit
 * ----> https://www.adafruit.com/products/3964
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#define VEML6075_ADDR         (0x10) ///< I2C address (cannot be changed)
#define VEML6075_REG_CONF     (0x00) ///< Configuration register
#define VEML6075_REG_UVA      (0x07) ///< UVA band raw measurement
#define VEML6075_REG_DARK     (0x08) ///< Dark current (?) measurement
#define VEML6075_REG_UVB      (0x09) ///< UVB band raw measurement
#define VEML6075_REG_UVCOMP1  (0x0A) ///< UV1 compensation value
#define VEML6075_REG_UVCOMP2  (0x0B) ///< UV2 compensation value
#define VEML6075_REG_ID       (0x0C) ///< Manufacture ID

#define VEML6075_DEFAULT_UVA_A_COEFF 2.22     ///< Default for no coverglass
#define VEML6075_DEFAULT_UVA_B_COEFF 1.33     ///< Default for no coverglass
#define VEML6075_DEFAULT_UVB_C_COEFF 2.95     ///< Default for no coverglass
#define VEML6075_DEFAULT_UVB_D_COEFF 1.74     ///< Default for no coverglass
#define VEML6075_DEFAULT_UVA_RESPONSE 0.001461     ///< Default for no coverglass
#define VEML6075_DEFAULT_UVB_RESPONSE 0.002591     ///< Default for no coverglass

/**************************************************************************/
/*!
    @brief  integration time definitions
 */
/**************************************************************************/
typedef enum veml6075_integrationtime {
	VEML6075_50MS,
	VEML6075_100MS,
	VEML6075_200MS,
	VEML6075_400MS,
	VEML6075_800MS,
} veml6075_integrationtime_t;


/**************************************************************************/
/*!
    @brief  CMSIS style register bitfield for commands
 */
/**************************************************************************/
typedef union {
	struct {
		uint8_t SD:1;              ///< Shut Down
		uint8_t UV_AF:1;           ///< Auto or forced
		uint8_t UV_TRIG:1;         ///< Trigger forced mode
		uint8_t UV_HD:1;           ///< High dynamic
		uint8_t UV_IT:3;           ///< Integration Time
		uint8_t high_byte;         ///< unused
	} bit;                       ///< Bitfield of 16 bits
	uint16_t reg;                ///< The raw 16 bit register data
} veml6075_commandRegister;




void veml6075_init();

/**
 * Runs the VEML6075's state machine
 */
void veml6075_run(void);

/*
 * This function enables or disables the VEML6075. When disabled, calling veml6075_run() does nothing.
 * The default state is disabled, so this function needs to be called before attempting a UV
 * measurement
 */
void veml6075_enable(bool enable);

float veml6075_get_last_uvi(void);

#endif /* SRC_VEML6075_H_ */
