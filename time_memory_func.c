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
#include "terminal.h"
#include "serialterminal.h"
#include "bluetoothterminal.h"
#include "lcd_func.h"



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


#define EEPROM_SLAVE_ADDRESS 0x50
#define EEPROM_ADDRESS_SIZE 2

#define RTC_SLAVE_ADDRESS 0x51
#define RTC_ADDRESS_SIZE 3
#define RTC_DATA_SIZE 3
#define RTC_SUBADDRESS_HOUR 0x02
#define RTC_CONTROL_REGISTER 0x00
#define ONE_BYTE_SIZE 1


#define MASK_FORMAT_AM_OR_PM_HOURS (0X40U)
#define MASK_FORMAT_24_OR_12_HOURS (0X80U)
#define MASK_FORMAT_Y_YEAR         (0XC0U)
#define MASK_FORMAT_U_MONTH        (0X0FU)
#define MASK_FORMAT_D_MONTH        (0X30U)
#define MASK_FORMAT_U_DAY          (0X0FU)
#define MASK_FORMAT_D_MONTH        (0X30U)
#define SLADE4 					   (A<<4)
#define SLADE6 					   (A<<6)
#define SLADE7 					   (A<<7)

//void separation_Of_Units(uint8_t var);

void i2c_init_peripherals();


SemaphoreHandle_t transfer_i2c_semaphore;
SemaphoreHandle_t mutex_transfer_i2c;
EventGroupHandle_t i2c_events_g;
i2c_master_handle_t g_m_handle;
QueueHandle_t time_mailbox;

QueueHandle_t get_time_mailbox(void)
{
	return time_mailbox;
}

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

void write_mem(int16_t subaddress, uint8_t buffer[], uint8_t dataSize)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA LISTO//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;

	masterXfer.slaveAddress = EEPROM_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = subaddress;
	masterXfer.subaddressSize = EEPROM_ADDRESS_SIZE;
	masterXfer.data = buffer;
	masterXfer.dataSize = dataSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

}

uint8_t * read_mem(int16_t subaddress, uint16_t dataSize)
{
	static i2c_master_transfer_t masterXfer;
	uint8_t * data_buffer;

	data_buffer = (uint8_t *)pvPortMalloc(dataSize * sizeof(uint8_t));
	//////////////////////////////////////////LEER MEMORIA LISTO//////////////////////////////////////////////////////////////////////
	masterXfer.slaveAddress = EEPROM_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = subaddress;
	masterXfer.subaddressSize = EEPROM_ADDRESS_SIZE;
	masterXfer.data = data_buffer;
	masterXfer.dataSize = dataSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

	return data_buffer;
}


void read_time(void * pvParameters)
{
	TickType_t xLastWakeTime;
	static i2c_master_transfer_t masterXfer;
	uint8_t buffer[3];
	const TickType_t xPeriod = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();
	//////////////////////////////////////////LEER TIEMPO LISTO//////////////////////////////////////////////////////////////////////
	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = RTC_SUBADDRESS_HOUR;
    masterXfer.subaddressSize = RTC_ADDRESS_SIZE;
    masterXfer.data = buffer;
    masterXfer.dataSize = RTC_DATA_SIZE;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	    xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	    xSemaphoreGive(mutex_transfer_i2c);
	    //send to buffer
	    xQueueSendToBack(time_mailbox, buffer, 0);
	    xQueueSendToBack(get_serial_time_queue(), buffer, 0);
	    xQueueSendToBack(get_bt_time_queue(), buffer, 0);

	    vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}


void write_time(uint8_t buffer[])
{
	//////////////////////////////////////////ESCRIBIR  medio listo//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;

	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = RTC_SUBADDRESS_HOUR;
	masterXfer.subaddressSize = RTC_ADDRESS_SIZE;
	masterXfer.data = buffer;
	masterXfer.dataSize = RTC_DATA_SIZE;
	masterXfer.flags = kI2C_TransferDefaultFlag;


	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);

}
//
//void format_of_hour(void * pvParameters)
//{
//	//////////////////////////////////////////ESCRIBIR escribir formato, medio liisto //////////////////////////////////////////////////////////////////////
//	static i2c_master_transfer_t masterXfer;
//	i2c_type * f_time = (i2c_type*) pvParameters;
//
//		masterXfer.slaveAddress = f_time->slaveAddress;
//	    masterXfer.direction = kI2C_Write;
//	    masterXfer.subaddress = f_time->subaddress;
//	    masterXfer.subaddressSize = 1;
//	    masterXfer.data = f_time->data_buffer;
//	    masterXfer.dataSize =  f_time->data_size;
//	    masterXfer.flags = kI2C_TransferDefaultFlag;
//
//
//	for(;;)
//	{
//		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
//		I2C_MasterTransferNonBlocking(I2C0,  f_time->handle, &masterXfer);
//		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
//		xSemaphoreGive(mutex_transfer_i2c);
//		vTaskDelete(NULL);
//}
//
//}
///**
// *
// */
//void read_date(void * pvParameters)
//{
//		static i2c_master_transfer_t masterXfer;
//		i2c_type * r_date = (i2c_type *) pvParameters;
//
//		typedef struct
//		{
//			uint8_t u_dias : 2;
//			uint8_t d_dias : 4;
//			uint8_t anos   : 2;
//			uint8_t u_meses : 4;
//			uint8_t d_meses : 1;
//
//		} separacion;
//
//		separacion fecha;
//		//////////////////////////////////////////LEER FECHA LISTO//////////////////////////////////////////////////////////////////////
//		masterXfer.slaveAddress = r_date->slaveAddress;
//	    masterXfer.direction = kI2C_Read;
//	    masterXfer.subaddress = r_date->subaddress;
//	    masterXfer.subaddressSize = 1;
//	    masterXfer.data = r_date->data_buffer;
//	    masterXfer.dataSize = r_date->data_size;
//	    masterXfer.flags = kI2C_TransferDefaultFlag;
//
//		for(;;)
//		{
//			xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
//		    I2C_MasterTransferNonBlocking(I2C0, (r_date->handle), &masterXfer);
//		    xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
//		    xSemaphoreGive(mutex_transfer_i2c);
//
//		    PRINTF("\r%x / %x / %x", r_date->data_buffer[0],r_date->data_buffer[2], (r_date->data_buffer[1]+2));
//		}
//}
//
//
//
//void write_date(void * pvParameters)
//{
//	//////////////////////////////////////////ESCRIBIR MEMORIA//////////////////////////////////////////////////////////////////////
//static i2c_master_transfer_t masterXfer;
//	i2c_type * w_date = (i2c_type*) pvParameters;
//
//		masterXfer.slaveAddress = w_date->slaveAddress;
//	    masterXfer.direction = kI2C_Write;
//	    masterXfer.subaddress = w_date->subaddress;
//	    masterXfer.subaddressSize = 1;
//	    masterXfer.data = w_date->data_buffer;
//	    masterXfer.dataSize =  w_date->data_size;
//	    masterXfer.flags = kI2C_TransferDefaultFlag;
//
//
//	for(;;)
//	{
//		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
//		I2C_MasterTransferNonBlocking(I2C0,  w_date->handle, &masterXfer);
//		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
//		xSemaphoreGive(mutex_transfer_i2c);
//		vTaskDelete(NULL);
//}
//
//}


void init_clk(void * pvParameters)
{
	static i2c_master_transfer_t masterXfer;
	static uint8_t buffer = 0x00;

	xTaskCreate(read_time, "read_time", configMINIMAL_STACK_SIZE, (void *) 0,
	        configMAX_PRIORITIES-1, NULL);

	masterXfer.slaveAddress = RTC_SLAVE_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = RTC_CONTROL_REGISTER;
	masterXfer.subaddressSize = ONE_BYTE_SIZE;
	masterXfer.data = &buffer;
	masterXfer.dataSize = ONE_BYTE_SIZE;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	xSemaphoreGive(mutex_transfer_i2c);
	vTaskDelete(NULL);

}


void i2c_init_peripherals(void)
{
    static i2c_master_config_t masterConfig;



//  date.d_day = 1;
//	date.u_day = 3;
//	date.d_month = 1;
//	date.u_month = 1;
//	date.years = 0;
//
//	time.u_seconds = 5;
//	time.d_seconds = 2;
//	time.u_minutes = 5;
//	time.d_minutes = 5;
//	time.u_hour = 1;
//	time.d_hour = 1;
//	time.f24_or_12 = 1; //0 -> 24, 1->12
//	time.fam_or_pm = 0; //0 -> am, 1->pm

	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_I2c0);

	port_pin_config_t config_i2c = { kPORT_PullUp, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister };

	PORT_SetPinConfig(PORTB, 2, &config_i2c);
	PORT_SetPinConfig(PORTB, 3, &config_i2c);

	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
			NULL);

	xTaskCreate(init_clk, "init_clk", configMINIMAL_STACK_SIZE, (void *) 0,
			configMAX_PRIORITIES, NULL);

    NVIC_EnableIRQ(I2C0_IRQn);
    NVIC_SetPriority(I2C0_IRQn, 5);

	transfer_i2c_semaphore = xSemaphoreCreateBinary();
	i2c_events_g = xEventGroupCreate();
	mutex_transfer_i2c = xSemaphoreCreateMutex();
	time_mailbox = xQueueCreate(1, (3*sizeof(uint8_t)));

}
