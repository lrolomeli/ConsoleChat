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
#if 0
/**
 * @file    imu_read.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef enum
{

	HOUR=0, DATE

} rtc_clock_type;

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
	i2c_master_handle_t handle;

} i2c_type;


#define MASK_U_MINUTES             (0X0FU)
#define MASK_D_MINUTES             (0XF0U)
#define MASK_U_SECONDS             (0X0FU)
#define MASK_D_SECONDS             (0XF0U)
#define MASK_U_HOURS               (0X0FU)
#define MASK_D_HOURS               (0XF0U)
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
void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void * userData);
void i2c_init_peripherals();


SemaphoreHandle_t transfer_i2c_semaphore;
SemaphoreHandle_t mutex_transfer_i2c;

/**
void write_mem(void * pvParameters)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;
	i2c_type * w_mem = (i2c_type*) pvParameters;

		masterXfer.slaveAddress = w_mem->slaveAddress;
	    masterXfer.direction = kI2C_Write;
	    masterXfer.subaddress = w_mem->subaddress;
	    masterXfer.subaddressSize = 2;
	    masterXfer.data = w_mem->data_buffer;
	    masterXfer.dataSize = w_mem->data_size;
	    masterXfer.flags = kI2C_TransferDefaultFlag;


	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
		I2C_MasterTransferNonBlocking(I2C0,  &(w_mem->handle), &masterXfer);
		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
		xSemaphoreGive(mutex_transfer_i2c);
		vTaskDelete(NULL);
}

}

void read_mem(void * pvParameters)
{
	static i2c_master_transfer_t masterXfer;
	i2c_type * r_mem = (i2c_type *) pvParameters;

	//////////////////////////////////////////LEER MEMORIA//////////////////////////////////////////////////////////////////////
	masterXfer.slaveAddress = r_mem->slaveAddress;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = r_mem->subaddress;
    masterXfer.subaddressSize = 2;
    masterXfer.data = r_mem->data_buffer;
    masterXfer.dataSize = r_mem->data_size;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	    I2C_MasterTransferNonBlocking(I2C0, &(r_mem->handle), &masterXfer);
	    xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	    xSemaphoreGive(mutex_transfer_i2c);

	}
}
*/
void init_clk(void * pvParameters)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;
	i2c_type * w_clk = (i2c_type*) pvParameters;

	masterXfer.slaveAddress = w_clk->slaveAddress;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = w_clk->subaddress;
	masterXfer.subaddressSize = 1;
	masterXfer.data = w_clk->data_buffer;
	masterXfer.dataSize = w_clk->data_size;
	masterXfer.flags = kI2C_TransferDefaultFlag;


	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
		I2C_MasterTransferNonBlocking(I2C0,  &(w_clk->handle), &masterXfer);
		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
		xSemaphoreGive(mutex_transfer_i2c);
		vTaskDelete(NULL);
}

}

void read_time(void * pvParameters)
{
	static i2c_master_transfer_t masterXfer;
	i2c_type * r_time = (i2c_type *) pvParameters;

	//////////////////////////////////////////LEER MEMORIA//////////////////////////////////////////////////////////////////////
	masterXfer.slaveAddress = r_time->slaveAddress;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = r_time->subaddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = r_time->data_buffer;
    masterXfer.dataSize = r_time->data_size;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	    I2C_MasterTransferNonBlocking(I2C0, &(r_time->handle), &masterXfer);
	    xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	    xSemaphoreGive(mutex_transfer_i2c);
	    PRINTF("%x\r", r_time->data_buffer);

	}
}
/**
void write_time(void * pvParameters)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;
	i2c_type * w_mem = (i2c_type*) pvParameters;

		masterXfer.slaveAddress = w_mem->slaveAddress;
	    masterXfer.direction = kI2C_Write;
	    masterXfer.subaddress = w_mem->subaddress;
	    masterXfer.subaddressSize = 2;
	    masterXfer.data = w_mem->data_buffer;
	    masterXfer.dataSize = 2;
	    masterXfer.flags = kI2C_TransferDefaultFlag;


	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
		I2C_MasterTransferNonBlocking(I2C0,  w_mem->handle, &masterXfer);
		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
		xSemaphoreGive(mutex_transfer_i2c);
		vTaskDelete(NULL);
}

}

void write_date(void * pvParameters)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;
	i2c_type * w_mem = (i2c_type*) pvParameters;

		masterXfer.slaveAddress = w_mem->slaveAddress;
	    masterXfer.direction = kI2C_Write;
	    masterXfer.subaddress = w_mem->subaddress;
	    masterXfer.subaddressSize = 2;
	    masterXfer.data = w_mem->data_buffer;
	    masterXfer.dataSize = 2;
	    masterXfer.flags = kI2C_TransferDefaultFlag;


	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
		I2C_MasterTransferNonBlocking(I2C0,  w_mem->handle, &masterXfer);
		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
		xSemaphoreGive(mutex_transfer_i2c);
		vTaskDelete(NULL);
}

}

void format_of_hour(void * pvParameters)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA//////////////////////////////////////////////////////////////////////
	static i2c_master_transfer_t masterXfer;
	i2c_type * w_mem = (i2c_type*) pvParameters;

		masterXfer.slaveAddress = w_mem->slaveAddress;
	    masterXfer.direction = kI2C_Write;
	    masterXfer.subaddress = w_mem->subaddress;
	    masterXfer.subaddressSize = 2;
	    masterXfer.data = w_mem->data_buffer;
	    masterXfer.dataSize = 2;
	    masterXfer.flags = kI2C_TransferDefaultFlag;


	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
		I2C_MasterTransferNonBlocking(I2C0,  w_mem->handle, &masterXfer);
		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
		xSemaphoreGive(mutex_transfer_i2c);
		vTaskDelete(NULL);
}

}

void read_date(void * pvParameters)
{
	static i2c_master_transfer_t masterXfer;
	i2c_type * r_mem = (i2c_type *) pvParameters;

	//////////////////////////////////////////LEER MEMORIA//////////////////////////////////////////////////////////////////////
	masterXfer.slaveAddress = r_mem->slaveAddress;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = r_mem->subaddress;
    masterXfer.subaddressSize = 2;
    masterXfer.data = r_mem->data_buffer;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{
		xSemaphoreTake(mutex_transfer_i2c, portMAX_DELAY);
	    I2C_MasterTransferNonBlocking(I2C0, r_mem->handle, &masterXfer);
	    xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	    xSemaphoreGive(mutex_transfer_i2c);

	}
}
*/
/*
 * @brief   Application entry point.
 */
int main(void)
{



    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

    i2c_init_peripherals();


	vTaskStartScheduler();



    /* Enter an infinite loop, just incrementing a counter. */
	for(;;)
	{

	}
    return 0;
}

void i2c_init_peripherals()
{
    static i2c_master_config_t masterConfig;
	static i2c_master_handle_t g_m_handle;
	static i2c_type w_mem;
	static i2c_type r_mem;
	static i2c_type w_clk;
	static i2c_type r_time;

/**
    date.d_day = 1;
    date.u_day = 3;
    date.d_month = 1;
    date.u_month = 1;
    date.years = 0;

    time.u_seconds = 5;
    time.d_seconds = 2;
    time.u_minutes = 5;
    time.d_minutes = 5;
    time.u_hour = 1;
    time.d_hour = 1;
    time.f24_or_12 = 1; //0 -> 24, 1->12
    time.fam_or_pm = 0; //0 -> am, 1->pm
    */

    ///////////////////////////////parametros para escribir en memoria////////////////////////////////////////////////////////
    w_mem.slaveAddress = 0x50;
    w_mem.subaddress = 0x00;			 //variable para la direccion a escribir
    w_mem.data_size = 2;				 //variable de cantidad de datos a escribir
    w_mem.data_buffer[0] = 1;			 //son el contenido del arreglo en el buffer
    w_mem.data_buffer[1] = 'a';          //son el contenido del arreglo en el buffer
    w_mem.handle = g_m_handle;

    ///////////////////////////////parametros para leer en memoria////////////////////////////////////////////////////////
    r_mem.slaveAddress = 0x50;
    r_mem.subaddress = 0x00;
    r_mem.data_size = 2;
    r_mem.handle = g_m_handle;

    ///////////////////////////////parametros para escribir en clock////////////////////////////////////////////////////////
    w_clk.slaveAddress = 0x51;
    w_clk.subaddress = 0x00;			 //variable para la direccion a escribir
    w_clk.data_size = 1;				 //variable de cantidad de datos a escribir
    w_clk.data_buffer[0] = 0x00;			 //son el contenido del arreglo en el buffer
    w_clk.handle = g_m_handle;

    ///////////////////////////////parametros para escribir en clock////////////////////////////////////////////////////////
    r_time.slaveAddress = 0x51;
    r_time.subaddress = 0x02;			 //variable para la direccion a escribir
    r_time.data_size = 1;				 //variable de cantidad de datos a escribir
    r_time.handle = g_m_handle;



	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_I2c0);

	port_pin_config_t config_i2c = { kPORT_PullUp, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister };

	PORT_SetPinConfig(PORTB, 2, &config_i2c);
	PORT_SetPinConfig(PORTB, 3, &config_i2c);

	I2C_MasterGetDefaultConfig(&masterConfig);
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	I2C_MasterTransferCreateHandle(I2C0, &(w_clk.handle), i2c_master_callback,
			NULL);

	//xTaskCreate(write_mem, "write_mem", 150, (void *) &w_mem, 4, NULL);

	//xTaskCreate(read_mem,  "read_mem" , 150, (void *) &r_mem, 3, NULL);

	xTaskCreate(init_clk, "init_clk", 150, (void *) &w_clk, 4, NULL);

	xTaskCreate(read_time,  "read_time" , 150, (void *) &r_time, 3, NULL);

	/**
	 *
	xTaskCreate(write_time, "write_time", 150, (void *) &w_mem, 4, NULL);

	xTaskCreate(write_date, "write_date" , 150, (void *) &r_mem, 3, NULL);

	xTaskCreate(format_of_hour, "format_of_hour", 150, (void *) &w_mem, 4, NULL);

	xTaskCreate(write_date, "write_date" , 150, (void *) &r_mem, 3, NULL);


	xTaskCreate(read_date,  "read_date" , 150, (void *) &r_mem, 3, NULL);

	*/

	transfer_i2c_semaphore = xSemaphoreCreateBinary();
	mutex_transfer_i2c = xSemaphoreCreateMutex();
    NVIC_EnableIRQ(I2C0_IRQn);
    NVIC_SetPriority(I2C0_IRQn, 5);

}
/**void digiToAscii(uint8_t var)
{
	if(var >= 10 && var <= 99)
	{
		b = (var / 10);
		a = ((var % 10));
	}

	if(var >= 0 && var <= 9)
	{

		a = var;
	}
}
*/
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
#endif
