/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    imu_read.c
 * @brief   Application entry point.
 */
#include <terminal_func.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "time_memory_func.h"
#include "chat_app_main.h"
#include "serialterminal.h"
#include "bluetoothterminal.h"
#include "lcd_func.h"


/*******************************************************************************
* DEFINITIONS
******************************************************************************/
#define EEPROM_SLAVE_ADDRESS 0x50
#define EEPROM_ADDRESS_SIZE 2

#define RTC_SLAVE_ADDRESS 0x51
#define RTC_ADDRESS_SIZE 3
#define RTC_DATA_SIZE 3
#define RTC_SUBADDRESS_HOUR 0x02
#define RTC_CONTROL_REGISTER 0x00
#define ONE_BYTE_SIZE 1

#define RTC_DATA_SIZE_DATE 2
#define RTC_SUBADDRESS_DATE 0x05
#define RTC_ADDRESS_SIZE_DATE 2



typedef struct
{
	uint8_t years   : 2;
	uint8_t d_day   : 2;
	uint8_t u_day   : 4;
	uint8_t d_month : 1;
	uint8_t u_month : 4;

} i2c_dateClock_type;

typedef struct
{
	uint8_t u_seconds : 4;
	uint8_t d_seconds : 4;
	uint8_t u_minutes : 4;
	uint8_t d_minutes : 4;
	uint8_t u_hour    : 4;
	uint8_t d_hour :    2;
	uint8_t fam_or_pm : 1;
	uint8_t f24_or_12 : 1;

} i2c_timeClock_type;

typedef struct
{
	uint8_t slaveAddress;
	uint32_t subaddress;
	uint8_t data_buffer[2];
	uint8_t data_size;
	i2c_master_handle_t* handle;

} i2c_type;

void i2c_init_peripherals();

/*******************************************************************************
* global variables
******************************************************************************/
SemaphoreHandle_t transfer_i2c_semaphore;
SemaphoreHandle_t mutex_transfer_i2c;
i2c_master_handle_t g_m_handle;
QueueHandle_t time_mailbox;
QueueHandle_t date_mailbox;
/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/
/**return the queue to another file**/
QueueHandle_t get_time_mailbox(void)
{
	return time_mailbox;
}

/**return the queue to another file**/
QueueHandle_t get_date_mailbox(void)
{
	return date_mailbox;
}

/*******************************************************************************
 * check the state of peripheral and
 * let continue whit execution
 ******************************************************************************/
void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void * userData)
{
    userData = userData;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (status == kStatus_Success)
    {
    	xSemaphoreGiveFromISR(transfer_i2c_semaphore, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*******************************************************************************
 * task that read the RTC in specific spaces of circuit and
 * write new values in a buffer each second
 ******************************************************************************/
void read_time(void * pvParameters)
{
	TickType_t xLastWakeTime;
	static i2c_master_transfer_t masterXfer;
	uint8_t buffer[3] = {0};

	/*time for refresh the buffer*/
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	//////////////////////////////////////////read time ready//////////////////////////////////////////////////////////////////////
	/*parameters for use the protocol i2c */
	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = RTC_SUBADDRESS_HOUR;
    masterXfer.subaddressSize = RTC_ADDRESS_SIZE;
    masterXfer.data = buffer;
    masterXfer.dataSize = RTC_DATA_SIZE;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{
		/*protection the resources whit using mutex and semaphores */
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	    xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	    xSemaphoreGive(mutex_transfer_i2c);

	    /*use the queue for the project*/
	    xQueueSendToBack(time_mailbox, buffer, 0);
	    xQueueSendToBack(get_serial_time_queue(), buffer, 0);
	    xQueueSendToBack(get_bt_time_queue(), buffer, 0);

	    vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

/*******************************************************************************
 * task that initial the RTC with zeros
 * create the task read time for start to use
 * this task is deleted for don't use heap
 ******************************************************************************/
void init_clk(void * pvParameters)
{
	static i2c_master_transfer_t masterXfer;

	static uint8_t buffer = 0x00; /*initial value of RTC*/

	/*create new task*/
	xTaskCreate(read_time, "read_time", configMINIMAL_STACK_SIZE, (void *) 0,
	        configMAX_PRIORITIES, NULL);

	/*parameter to use protocol i2c*/
	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = RTC_CONTROL_REGISTER;
	masterXfer.subaddressSize = ONE_BYTE_SIZE;
	masterXfer.data = &buffer;
	masterXfer.dataSize = ONE_BYTE_SIZE;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/**protection for resources **/
	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

	/*delete task*/
	vTaskDelete(NULL);

}

/*******************************************************************************
 * PROTOTYPES
 ******************************************************************************/
void i2c_init_peripherals(void)
{
    static i2c_master_config_t masterConfig;

    /*clock of every peripheral*/
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_I2c0);

	/*configuration at i2c protocol*/
	port_pin_config_t config_i2c = { kPORT_PullUp, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister };

	/*pins that are defined as SCL and SDA for i2c protocol*/
	PORT_SetPinConfig(PORTB, 2, &config_i2c);
	PORT_SetPinConfig(PORTB, 3, &config_i2c);

	/*parameter that are necessary at i2c protocol*/
	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
			NULL);

	xTaskCreate(init_clk, "init_clk", configMINIMAL_STACK_SIZE, (void *) 0,
			configMAX_PRIORITIES, NULL);

	/*interruptions vector*/
    NVIC_EnableIRQ(I2C0_IRQn);
    NVIC_SetPriority(I2C0_IRQn, 5);

    /*created the shemaphore and mutex to i2c */
	transfer_i2c_semaphore = xSemaphoreCreateBinary();
	mutex_transfer_i2c = xSemaphoreCreateMutex();

	/*created queue*/
	time_mailbox = xQueueCreate(1, (3*sizeof(uint8_t)));
	date_mailbox = xQueueCreate(1, (2*sizeof(uint8_t)));

}

/*******************************************************************************
 * write strings of characters in the external eprom
 ******************************************************************************/
void write_mem(int16_t subaddress, uint8_t buffer[], uint8_t dataSize)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA LISTO//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;

	/*parameters for use the protocol i2c */
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = subaddress;
	masterXfer.subaddressSize = EEPROM_ADDRESS_SIZE;
	masterXfer.data = buffer;
	masterXfer.dataSize = dataSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/*protection the resurces whit using mutex and semaphores */
	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

}

/*******************************************************************************
 * read the buffer of external eeprom
 ******************************************************************************/
uint8_t * read_mem(int16_t subaddress, uint16_t dataSize)
{
	static i2c_master_transfer_t masterXfer;
	uint8_t * data_buffer;

	/*parameters for use the protocol i2c */
	data_buffer = (uint8_t *)pvPortMalloc(dataSize * sizeof(uint8_t));
	//////////////////////////////////////////LEER MEMORIA LISTO//////////////////////////////////////////////////////////////////////
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = subaddress;
	masterXfer.subaddressSize = EEPROM_ADDRESS_SIZE;
	masterXfer.data = data_buffer;
	masterXfer.dataSize = dataSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/*protection the resurces whit using mutex and semaphores */
	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

	return data_buffer;
}

/*******************************************************************************
 * write in the specific space of rtc for configure the date
 ******************************************************************************/
void write_time(uint8_t buffer[])
{
	//////////////////////////////////////////ESCRIBIR  medio listo//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;

	/*parameters for use the protocol i2c */
	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = RTC_SUBADDRESS_HOUR;
	masterXfer.subaddressSize = RTC_ADDRESS_SIZE;
	masterXfer.data = buffer;
	masterXfer.dataSize = RTC_DATA_SIZE;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/*protection the resurces whit using mutex and semaphores */
	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

}

/*******************************************************************************
 * read the buffer of external eeprom
 ******************************************************************************/
uint8_t * read_date(int16_t subaddress, uint16_t dataSize)
{
	static i2c_master_transfer_t masterXfer;
	uint8_t * buffer;
	buffer = (uint8_t *)pvPortMalloc(dataSize * sizeof(uint8_t));

	/*parameters for use the protocol i2c */
	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = RTC_SUBADDRESS_DATE;
    masterXfer.subaddressSize = RTC_ADDRESS_SIZE_DATE;
    masterXfer.data = buffer;
    masterXfer.dataSize = RTC_DATA_SIZE_DATE;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	/*protection the resurces whit using mutex and semaphores */
	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

	/*protection the resurces whit using mutex and semaphores */
	xQueueSendToBack(date_mailbox, buffer, 0);
	xQueueSendToBack(get_serial_date_queue(), buffer, 0);
	xQueueSendToBack(get_bt_date_queue(), buffer, 0);

	return buffer;
}

/*******************************************************************************
 * write in the specific space of rtc for configure the date
 ******************************************************************************/
void write_date(uint8_t buffer[])
{
	static i2c_master_transfer_t masterXfer;

	/*parameters for use the protocol i2c */
	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = RTC_SUBADDRESS_DATE;
	masterXfer.subaddressSize = RTC_ADDRESS_SIZE_DATE;
	masterXfer.data = buffer;
	masterXfer.dataSize = RTC_DATA_SIZE_DATE;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/*protection the resurces whit using mutex and semaphores */
	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);
}
