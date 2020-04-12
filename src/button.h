/*
 * button.h
 *
 *  Created on: Mar 20, 2020
 *      Author: Bryan
 */

#ifndef SRC_BUTTON_H_
#define SRC_BUTTON_H_

#include <stdbool.h>

typedef enum
{
	PB0,
	PB1,
} pushbuttons_t;

void button_init(void);

void button_update_state(void);

bool button_get_pushbutton_state(pushbuttons_t button);

#endif /* SRC_BUTTON_H_ */
