/*
 * events.h
 *
 *  Created on: Feb 10, 2020
 *      Author: Bryan
 */

#ifndef SRC_EVENTS_H_
#define SRC_EVENTS_H_

#include "stdint.h"
#include "stdbool.h"
#include "native_gecko.h"

#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1

// Define the bits for each event
#define EVENT_TIMER_PERIOD_EXPIRED	(1 << 0)
#define EVENT_TIMER_WAIT_DONE		(1 << 1)
#define EVENT_I2C_COMPLETE			(1 << 2)
#define EVENT_PB0_PRESS				(1 << 3)

/**
 * Safely set an event
 */
void events_set_event(uint32_t event_mask);

/**
 * Safely clear an event
 */
void events_clear_event(uint32_t event_mask);

/**
 * This function returns the events bitmask.
 *
 * Note that while there is protection internal to this function to prevent
 * race conditions on the data access, the user should probably also use this
 * function within a critical section in order to handle handle events/sleep
 * correctly.
 */
uint32_t events_get_events(void);

/**
 * This function returns true if the event in event_mask has occurred, false otherwise
 */
bool events_get_event(uint32_t event_mask);

/**
 * This function updates the event bitmask with the data received from the stack
 */
void events_update_bitmask(uint32_t bitmask);

/**
 * Handle any events that have occurred. This function should be called once gecko_wait_event() has returned
 */
void events_update(struct gecko_cmd_packet *evt);

#endif /* SRC_EVENTS_H_ */
