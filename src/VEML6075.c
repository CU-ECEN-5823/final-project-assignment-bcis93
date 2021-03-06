/*
 * VEML6075.c
 *
 *  Created on: Apr 17, 2020
 *      Author: Bryan
 */

#include "VEML6075.h"
#include "timer.h"
#include "events.h"
#include "em_gpio.h"
#include "log.h"
#include "i2c.h"
#include "sleep.h"
#include "display.h"

#define SENSOR_ENABLE_PORT (gpioPortD)
#define SENSOR_ENABLE_PIN (15)
#define POWER_ON_TIME_MS (80)

enum veml6075_state
{
	veml6075_power_off,
	veml6075_wait_for_powerup,
	veml6075_wait_for_i2c_write,
	veml6075_wait_for_conversion_time,
	veml6075_wait_for_i2c_read_uva,
	veml6075_wait_for_i2c_read_uvb,
	veml6075_wait_for_i2c_read_uv_comp1,
	veml6075_wait_for_i2c_read_uv_comp2,
};

static enum veml6075_state current_state = veml6075_power_off;
static enum veml6075_state prev_state = veml6075_power_off;

static bool enabled = false;

static uint8_t write_buf[3] = {};
static uint8_t read_buf[2] = {};

/*!
 * @file Adafruit_VEML6075.cpp
 *
 * @mainpage Adafruit VEML6075 UV sensor
 *
 * @section intro_sec Introduction
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
 * @section dependencies Dependencies
 *
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

static I2C_TransferReturn_TypeDef veml6075_writeRegister(void);
static I2C_TransferReturn_TypeDef veml6075_readRegister(uint8_t reg, uint16_t read_length);
static void veml6075_setIntegrationTime(veml6075_integrationtime_t itime);
static void veml6075_setHighDynamic(bool hd);
static void veml6075_setForcedMode(bool flag);
static void veml6075_setCoefficients(float UVA_A, float UVA_B, float UVA_C, float UVA_D,
		float UVA_response, float UVB_response);
static void veml6075_shutdown(bool sd);

uint16_t _read_delay;

// coefficients
static float _uva_a, _uva_b, _uvb_c, _uvb_d, _uva_resp, _uvb_resp;

// values read from sensor
static float uva, uvb, uv_comp1, uv_comp2;

// calculated UV index
static float uvi_calc;

static veml6075_commandRegister _commandRegister;


/**************************************************************************/
/*!
    @brief constructor initializes default configuration value
 */
/**************************************************************************/
void veml6075_init() {
	veml6075_setCoefficients(VEML6075_DEFAULT_UVA_A_COEFF, VEML6075_DEFAULT_UVA_B_COEFF,
			VEML6075_DEFAULT_UVB_C_COEFF, VEML6075_DEFAULT_UVB_D_COEFF,
			VEML6075_DEFAULT_UVA_RESPONSE, VEML6075_DEFAULT_UVB_RESPONSE);

	_commandRegister.reg = 0;
}

void veml6075_powerOn(void)
{
	LOG_DEBUG("Powering on veml6075...");

	GPIO_PinModeSet(SENSOR_ENABLE_PORT, SENSOR_ENABLE_PIN, gpioModePushPull, true);
	GPIO_DriveStrengthSet(SENSOR_ENABLE_PORT, gpioDriveStrengthStrongAlternateStrong);

	LOG_DEBUG("Complete!");
}

void veml6075_powerOff(void)
{
	LOG_DEBUG("Powering off veml6075...");
	GPIO_PinModeSet(SENSOR_ENABLE_PORT, SENSOR_ENABLE_PIN, gpioModeDisabled, true);
	LOG_DEBUG("Complete!");
}

static void veml6075_unexpected_error(void)
{
	veml6075_powerOff();
	i2c_deInit();
	current_state = veml6075_power_off;
}

void veml6075_run(void)
{
	if (enabled)
	{
		switch (current_state)
		{
		case veml6075_power_off:
			if (events_get_event(EVENT_TIMER_PERIOD_EXPIRED))
			{
				// period timer has expired, time to start taking a measurement!
				// First, clear event
				events_clear_event(EVENT_TIMER_PERIOD_EXPIRED);

				veml6075_powerOn();
				i2c_init();

				// start a timer to wait for the Si7021 to fully power up
				timer_start_ms_timer(POWER_ON_TIME_MS);

				current_state = veml6075_wait_for_powerup;
			}
			break;
		case veml6075_wait_for_powerup:
			if (events_get_event(EVENT_TIMER_WAIT_DONE)) // Power-up complete!
			{
				// First, clear event
				events_clear_event(EVENT_TIMER_WAIT_DONE);

				_commandRegister.reg = 0;


				// Force readings
				veml6075_setForcedMode(true);

				// Set integration time
				veml6075_setIntegrationTime(VEML6075_100MS);

				// Set high dynamic
				veml6075_setHighDynamic(false);

				veml6075_shutdown(false); // Enable

				I2C_TransferReturn_TypeDef status = veml6075_writeRegister();

				if (status == i2cTransferInProgress) // if transfer started successfully
				{
					// sleep block! allow sleep in EM1, but block deeper sleep while I2C is active
					SLEEP_SleepBlockBegin(sleepEM2);

					current_state = veml6075_wait_for_i2c_write;
				}
				else
				{
					LOG_ERROR("I2C transfer failed to start!");
					veml6075_unexpected_error();
				}
			}
			break;
		case veml6075_wait_for_i2c_write:
			if (events_get_event(EVENT_I2C_COMPLETE)) // I2C write done!
			{
				events_clear_event(EVENT_I2C_COMPLETE);

				// I2C is done (for now), so we can end the sleep block!
				SLEEP_SleepBlockEnd(sleepEM2);

				// Check the return status of the transfer
				I2C_TransferReturn_TypeDef status = i2c_get_status();

				if (status == i2cTransferDone) // if the transfer completed successfully
				{
					// start a timer to wait for the veml6075 to complete a temperature conversion
					timer_start_ms_timer(_read_delay * 1.1);

					current_state = veml6075_wait_for_conversion_time;
				}
				else
				{
					LOG_ERROR("I2C transfer failed!");
					veml6075_unexpected_error();
				}
			}
			break;
		case veml6075_wait_for_conversion_time:
			if (events_get_event(EVENT_TIMER_WAIT_DONE)) // Conversion time complete!
			{
				// First, clear event
				events_clear_event(EVENT_TIMER_WAIT_DONE);

				// start i2c read
				I2C_TransferReturn_TypeDef status = veml6075_readRegister(VEML6075_REG_UVA, 2);

				if (status == i2cTransferInProgress) // if transfer started successfully
				{
					// sleep block! allow sleep in EM1, but block deeper sleep while I2C is active
					SLEEP_SleepBlockBegin(sleepEM2);

					current_state = veml6075_wait_for_i2c_read_uva;
				}
				else
				{
					LOG_ERROR("I2C transfer failed to start!");
					veml6075_unexpected_error();
				}
			}
			break;
		case veml6075_wait_for_i2c_read_uva:
			if (events_get_event(EVENT_I2C_COMPLETE)) // I2C read done!
			{
				events_clear_event(EVENT_I2C_COMPLETE);

				// Check the return status of the transfer
				I2C_TransferReturn_TypeDef status = i2c_get_status();

				if (status == i2cTransferDone) // if the transfer completed successfully
				{
					// combine the read bytes into an unsigned 16 bit value. Then cast this to a signed int, and finally to a float
					uva = (float)(int16_t)((uint16_t)read_buf[0] + ((uint16_t)read_buf[1] << 8));

					LOG_DEBUG("uva: %f", uva);

					I2C_TransferReturn_TypeDef status = veml6075_readRegister(VEML6075_REG_UVB, 2);

					if (status == i2cTransferInProgress) // if transfer started successfully
					{
						current_state = veml6075_wait_for_i2c_read_uvb;
					}
					else
					{
						LOG_ERROR("I2C transfer failed to start!");
						veml6075_unexpected_error();
					}
				}
				else
				{
					LOG_ERROR("I2C transfer failed!");
					veml6075_unexpected_error();
				}
			}
			break;
		case veml6075_wait_for_i2c_read_uvb:
			if (events_get_event(EVENT_I2C_COMPLETE)) // I2C read done!
			{
				events_clear_event(EVENT_I2C_COMPLETE);

				// Check the return status of the transfer
				I2C_TransferReturn_TypeDef status = i2c_get_status();

				if (status == i2cTransferDone) // if the transfer completed successfully
				{
					// combine the read bytes into an unsigned 16 bit value. Then cast this to a signed int, and finally to a float
					uvb = (float)(int16_t)((uint16_t)read_buf[0] + ((uint16_t)read_buf[1] << 8));

					LOG_DEBUG("uvb: %f", uvb);

					I2C_TransferReturn_TypeDef status = veml6075_readRegister(VEML6075_REG_UVCOMP1, 2);

					if (status == i2cTransferInProgress) // if transfer started successfully
					{
						current_state = veml6075_wait_for_i2c_read_uv_comp1;
					}
					else
					{
						LOG_ERROR("I2C transfer failed to start!");
						veml6075_unexpected_error();
					}
				}
				else
				{
					LOG_ERROR("I2C transfer failed!");
					veml6075_unexpected_error();
				}
			}
			break;
		case veml6075_wait_for_i2c_read_uv_comp1:
			if (events_get_event(EVENT_I2C_COMPLETE)) // I2C read done!
			{
				events_clear_event(EVENT_I2C_COMPLETE);

				// Check the return status of the transfer
				I2C_TransferReturn_TypeDef status = i2c_get_status();

				if (status == i2cTransferDone) // if the transfer completed successfully
				{
					// combine the read bytes into an unsigned 16 bit value. Then cast this to a signed int, and finally to a float
					uv_comp1 = (float)(int16_t)((uint16_t)read_buf[0] + ((uint16_t)read_buf[1] << 8));

					LOG_DEBUG("uv_comp1: %f", uv_comp1);

					I2C_TransferReturn_TypeDef status = veml6075_readRegister(VEML6075_REG_UVCOMP2, 2);

					if (status == i2cTransferInProgress) // if transfer started successfully
					{
						current_state = veml6075_wait_for_i2c_read_uv_comp2;
					}
					else
					{
						LOG_ERROR("I2C transfer failed to start!");
						veml6075_unexpected_error();
					}
				}
				else
				{
					LOG_ERROR("I2C transfer failed!");
					veml6075_unexpected_error();
				}
			}
			break;
		case veml6075_wait_for_i2c_read_uv_comp2:
			if (events_get_event(EVENT_I2C_COMPLETE)) // I2C read done!
			{
				events_clear_event(EVENT_I2C_COMPLETE);

				// I2C is done, so we can end the sleep block!
				SLEEP_SleepBlockEnd(sleepEM2);

				// Check the return status of the transfer
				I2C_TransferReturn_TypeDef status = i2c_get_status();

				if (status == i2cTransferDone) // if the transfer completed successfully
				{
					veml6075_powerOff();
					i2c_deInit();

					// combine the read bytes into an unsigned 16 bit value. Then cast this to a signed int, and finally to a float
					uv_comp2 = (float)(int16_t)((uint16_t)read_buf[0] + ((uint16_t)read_buf[1] << 8));

					LOG_DEBUG("uv_comp2: %f", uv_comp2);

					// Equasion 1 & 2 in App note, without 'golden sample' calibration
					float uva_calc = uva - (_uva_a * uv_comp1) - (_uva_b * uv_comp2);
					float uvb_calc = uvb - (_uvb_c * uv_comp1) - (_uvb_d * uv_comp2);
					uvi_calc = ((uva_calc * _uva_resp) + (uvb_calc * _uvb_resp)) / 2;

					LOG_DEBUG("UVA: %f", uva_calc);
					LOG_DEBUG("UVB: %f", uvb_calc);
					LOG_INFO("UVI: %f", uvi_calc);

					if (displayEnabled())
					{
						int16_t integer = (int16_t)uva_calc;
						int16_t decimal = (uva_calc - integer) * 1000;
						displayPrintf(DISPLAY_ROW_PASSKEY, "UVA: %d.%03d", integer, decimal);
						integer = (int16_t)uvb_calc;
						decimal = (uvb_calc - integer) * 1000;
						displayPrintf(DISPLAY_ROW_ACTION, "UVB: %d.%03d", integer, decimal);
						integer = (int16_t)uvi_calc;
						decimal = (uvi_calc - integer) * 1000;
						displayPrintf(DISPLAY_ROW_TEMPVALUE, "UVI: %d.%03d", integer, decimal);
					}

					current_state = veml6075_power_off;
				}
				else
				{
					LOG_ERROR("I2C transfer failed!");
					veml6075_unexpected_error();
				}
			}
			break;
		default:
			break;
		}

		// This logging idea came from the lecture slides
		if (current_state != prev_state)
		{
			LOG_DEBUG("VEML6075 transitioned from state %d to %d", prev_state, current_state);
			prev_state = current_state;
		}
	}
}


static I2C_TransferReturn_TypeDef veml6075_writeRegister(void)
{
	// I2C write!
	const uint16_t command_length = 3;
	write_buf[0] = VEML6075_REG_CONF;
	write_buf[1] = _commandRegister.reg >> 8 & 0xFF;
	write_buf[2] = _commandRegister.reg & 0xFF;
	return i2c_write_start(VEML6075_ADDR, command_length, write_buf);
}

static I2C_TransferReturn_TypeDef veml6075_readRegister(uint8_t reg, uint16_t read_length)
{
	// I2C write/read!
	const uint16_t write_length = 1;
	write_buf[0] = reg;
	return i2c_write_read_start(VEML6075_ADDR, write_length, write_buf, read_length, read_buf);
}

/*
 * This function enables or disables the VEML6075. When disabled, calling veml6075_run() does nothing.
 * The default state is disabled, so this function needs to be called before attempting a UV
 * measurement
 */
void veml6075_enable(bool enable)
{
	enabled  =  enable;
}


/**************************************************************************/
/*!
    @brief Set the UVI calculation coefficients, see datasheet for some example values. We use the default "no cover glass" values if none are specified when initializing the sensor.
    @param UVA_A  the UVA visible coefficient
    @param UVA_B  the UVA IR coefficient
    @param UVB_C  the UVB visible coefficient
    @param UVB_D  the UVB IR coefficient
    @param UVA_response the UVA responsivity
    @param UVB_response the UVB responsivity
 */
/**************************************************************************/
static void veml6075_setCoefficients(float UVA_A, float UVA_B, float UVB_C, float UVB_D,
		float UVA_response, float UVB_response) {
	_uva_a = UVA_A;
	_uva_b = UVA_B;
	_uvb_c = UVB_C;
	_uvb_d = UVB_D;
	_uva_resp = UVA_response;
	_uvb_resp = UVB_response;
}


/**************************************************************************/
/*!
    @brief Set the time over which we sample UV data per read
    @param itime How many milliseconds to integrate over
 */
/**************************************************************************/
static void veml6075_setIntegrationTime(veml6075_integrationtime_t itime) {
	// Set integration time
	_commandRegister.bit.UV_IT = (uint8_t)itime;
	// I2C write
	//Config_Register->write(_commandRegister.reg);

	_read_delay = 0;
	switch (itime) {
	case VEML6075_50MS: _read_delay = 50; break;
	case VEML6075_100MS: _read_delay = 100; break;
	case VEML6075_200MS: _read_delay = 200; break;
	case VEML6075_400MS: _read_delay = 400; break;
	case VEML6075_800MS: _read_delay = 800; break;
	}
}


/**************************************************************************/
/*!
    @brief Sets whether to take readings in 'high dynamic' mode
    @param hd True if you want high dynamic readings, False for normal dynamic
 */
/**************************************************************************/
static void veml6075_setHighDynamic(bool hd) {
	// Set HD mode
	_commandRegister.bit.UV_HD = hd;
	// I2C write
	//Config_Register->write(_commandRegister.reg);
}


/**************************************************************************/
/*!
    @brief Sets whether to take readings on request
    @param flag True if you want readings on request, False for continuous reads
 */
/**************************************************************************/
static void veml6075_setForcedMode(bool flag) {
	// Set forced mode
	_commandRegister.bit.UV_AF = flag;
}

/**************************************************************************/
/*!
    @brief Shut down the sensor
    @param sd True if you want to shut down, false to enable
 */
/**************************************************************************/
static void veml6075_shutdown(bool sd) {
	_commandRegister.bit.SD = sd;
}

float veml6075_get_last_uvi(void)
{
	return uvi_calc;
}
