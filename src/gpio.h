/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>

#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1

/**
 * Initialize the GPIO
 */
void gpioInit();

/**
 * Turn LED0 on
 */
void gpioLed0SetOn();

/**
 * Turn LED0 off
 */
void gpioLed0SetOff();

/**
 * Turn LED1 on
 */
void gpioLed1SetOn();

/**
 * Turn LED1 off
 */
void gpioLed1SetOff();

/**
 * Enable the GPIO pin needed for the display
 */
void gpioEnableDisplay(void);

/**
 * Disable the GPIO pin needed for the display
 */
void gpioDisableDisplay(void);

/**
 * Set the EXTCOMIN pin high or low, depending on the bool input
 */
void gpioSetDisplayExtcomin(bool high);

#endif /* SRC_GPIO_H_ */
