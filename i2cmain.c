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
#include "freeRTOS.h"
#include "task.h"
#include "semphr.h"




static i2c_master_handle_t g_m_handle;
SemaphoreHandle_t transfer_i2c_semaphore;
i2c_master_transfer_t masterXfer;

/* TODO: insert other definitions and declarations here. */

volatile bool g_MasterCompletionFlag = false;
void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void * userData);


void write_mem(void * pvParameters)
{
	//////////////////////////////////////////ESCRIBIR MEMORIA//////////////////////////////////////////////////////////////////////
	uint8_t data_buffer = 'a';

    masterXfer.slaveAddress = 0x50;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0x00FF;
    masterXfer.subaddressSize = 2;
    masterXfer.data = &data_buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{

		I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle, &masterXfer);
		xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
		vTaskDelete(NULL);
	}

}


void read_mem(void * pvParameters)
{
	//////////////////////////////////////////LEER MEMORIA//////////////////////////////////////////////////////////////////////
	uint8_t read_buffer;

	masterXfer.slaveAddress = 0x50;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0x00FF;
    masterXfer.subaddressSize = 2;
    masterXfer.data = &read_buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{
	    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	    xSemaphoreTake(transfer_i2c_semaphore, portMAX_DELAY);
	    PRINTF("%x", read_buffer);

	}
}

/**
//////////////////////////////////////////ESCRIBIR CLK//////////////////////////////////////////////////////////////////////
void write_clk(void * pvParameters)
{
    uint8_t data_buffer = 0x00;

    masterXfer.slaveAddress = 0x51;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0x00;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &data_buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{
		       I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle, &masterXfer);
		       xEventGroupWaitBits(write_read_i2c, EVENT_WRITE_CLK_I2C, pdTRUE, pdTRUE, portMAX_DELAY);
		       g_MasterCompletionFlag = false;
	}
}

//////////////////////////////////////////LEER SEG CLK//////////////////////////////////////////////////////////////////////
void read_clk(void * pvParameters)
{
	uint8_t read_buffer;

    masterXfer.slaveAddress = 0x51;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0x02;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &read_buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

	for(;;)
	{

	   I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	   xEventGroupWaitBits(write_read_i2c, EVENT_WRITE_CLK_I2C, pdTRUE, pdTRUE, portMAX_DELAY);
	    g_MasterCompletionFlag = false;
	    PRINTF("%x\r", read_buffer);
	}
}
*/



/*
 * @brief   Application entry point.
 */
int main(void)
{

    i2c_master_config_t masterConfig;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_I2c0);

    port_pin_config_t config_i2c =
    { kPORT_PullUp, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
            kPORT_UnlockRegister };


    PORT_SetPinConfig(PORTB, 2, &config_i2c);
    PORT_SetPinConfig(PORTB, 3, &config_i2c);

    I2C_MasterGetDefaultConfig(&masterConfig);
    I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
    I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback, NULL);

	xTaskCreate(write_mem, "write_mem", 150, (void *) 0, 4, NULL);
	xTaskCreate(read_mem,  "read_mem" , 150, (void *) 0, 3, NULL);
	//xTaskCreate(write_clk, "write_clk", 110, (void *) 0, 2, NULL);
	//xTaskCreate(write_mem, "read_clk" , 110, (void *) 0, 1, NULL);

	transfer_i2c_semaphore = xSemaphoreCreateBinary();
    NVIC_EnableIRQ(I2C0_IRQn);
    NVIC_SetPriority(I2C0_IRQn, 5);
	vTaskStartScheduler();



    /* Enter an infinite loop, just incrementing a counter. */
	for(;;)
	{

	}
    return 0;
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
