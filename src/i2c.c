/*
 * i2c.c
 *
 *  Created on: Feb 10, 2020
 *      Author: Bryan
 */

#include "i2cspm.h"
#include "log.h"
#include "events.h"
#include "em_core.h"

static I2C_TransferSeq_TypeDef transfer;
static volatile I2C_TransferReturn_TypeDef transfer_status;

void i2c_init(void)
{
	LOG_DEBUG("Initializing I2C...");
	I2CSPM_Init_TypeDef i2cInit;
	i2cInit.port = I2C0;
	i2cInit.sclPort = gpioPortC;
	i2cInit.sclPin = 10;
	i2cInit.sdaPort = gpioPortC;
	i2cInit.sdaPin = 11;
	i2cInit.portLocationScl = 14;
	i2cInit.portLocationSda = 16;
	i2cInit.i2cRefFreq = 0; // Use currently configured reference clock
	i2cInit.i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
	i2cInit.i2cClhr = i2cClockHLRStandard;

	I2CSPM_Init(&i2cInit);
	NVIC_EnableIRQ(I2C0_IRQn);

	LOG_DEBUG("Complete!");
}

void i2c_deInit(void)
{
	LOG_DEBUG("De-initializing I2C...");
	I2C_Reset(I2C0);
	NVIC_DisableIRQ(I2C0_IRQn);
	LOG_DEBUG("Complete!");
}


I2C_TransferReturn_TypeDef i2c_read_start(uint16_t addr, uint16_t len, uint8_t* buf)
{
	transfer.addr = addr << 1; // Shift one to the left so that the 7 bit address is in the upper 7 bits
	transfer.flags = I2C_FLAG_READ;
	transfer.buf[0].len = len;
	transfer.buf[0].data = buf;

	I2C_TransferReturn_TypeDef status = I2C_TransferInit(I2C0, &transfer);

	if (status != i2cTransferInProgress)
		{
			LOG_ERROR("Failed to start i2c read! Return value was %d", status);
		}

	return status;
}

I2C_TransferReturn_TypeDef i2c_write_start(uint16_t addr, uint16_t len, uint8_t* buf)
{

	transfer.addr = addr << 1; // Shift one to the left so that the 7 bit address is in the upper 7 bits
	transfer.flags = I2C_FLAG_WRITE;
	transfer.buf[0].len = len;
	transfer.buf[0].data = buf;

	I2C_TransferReturn_TypeDef status = I2C_TransferInit(I2C0, &transfer);

	if (status != i2cTransferInProgress)
	{
		LOG_ERROR("Failed to start i2c write! Return value was %d", status);
	}

	return status;
}

I2C_TransferReturn_TypeDef i2c_write_read_start(uint16_t addr, uint16_t write_len, uint8_t* write_buf, uint16_t read_len, uint8_t* read_buf)
{

	transfer.addr = addr << 1; // Shift one to the left so that the 7 bit address is in the upper 7 bits
	transfer.flags = I2C_FLAG_WRITE_READ;
	transfer.buf[0].len = write_len;
	transfer.buf[0].data = write_buf;
	transfer.buf[1].len = read_len;
	transfer.buf[1].data = read_buf;

	I2C_TransferReturn_TypeDef status = I2C_TransferInit(I2C0, &transfer);

	if (status != i2cTransferInProgress)
	{
		LOG_ERROR("Failed to start i2c write_read! Return value was %d", status);
	}

	return status;
}

I2C_TransferReturn_TypeDef i2c_get_status(void)
{
	CORE_irqState_t state = CORE_EnterCritical();
	I2C_TransferReturn_TypeDef status = transfer_status;
	CORE_ExitCritical(state);
	return status;
}

void I2C0_IRQHandler(void)
{
	I2C_TransferReturn_TypeDef local_status = I2C_Transfer(I2C0);

	CORE_irqState_t state = CORE_EnterCritical();
	transfer_status = local_status;
	CORE_ExitCritical(state);

	if (transfer_status != i2cTransferInProgress)
	{
		events_set_event(EVENT_I2C_COMPLETE);

		if (transfer_status != i2cTransferDone)
		{
			LOG_ERROR("I2C transfer failed with status %d", transfer_status);
		}
	}
}
