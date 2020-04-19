/*
 * i2c.h
 *
 *  Created on: Feb 10, 2020
 *      Author: Bryan
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "em_i2c.h"

/**
 * Initialize the I2C interface
 */
void i2c_init(void);

/**
 * De-initialize the I2C interface
 */
void i2c_deInit(void);

/**
 * Start an I2C read at addr. Read len bytes into buf.
 * Returns i2cTransferInProgress on success, other values on error
 */
I2C_TransferReturn_TypeDef i2c_read_start(uint16_t addr, uint16_t len, uint8_t* buf);

/**
 * Start an I2C write at addr. Write len bytes from buf.
 * Returns i2cTransferInProgress on success, other values on error
 */
I2C_TransferReturn_TypeDef i2c_write_start(uint16_t addr, uint16_t len, uint8_t* buf);

/**
 * Start an I2C write_read at addr. Write write_len bytes from write_buf,
 * then read read_len bytes into read_buf.
 * Returns i2cTransferInProgress on success, other values on error
 */
I2C_TransferReturn_TypeDef i2c_write_read_start(uint16_t addr, uint16_t write_len, uint8_t* write_buf, uint16_t read_len, uint8_t* read_buf);


/**
 * This function returns the status of the latest I2C transaction
 */
I2C_TransferReturn_TypeDef i2c_get_status(void);

#endif /* SRC_I2C_H_ */
