/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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
#include "bluetoothterminal.h"
#include "serialterminal.h"
#include "terminal.h"
#include "board.h"
#include "menu.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "chat_app_main.h"

/*******************************************************************************
 * Structures
 ******************************************************************************/
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * TASKS CODE
 ******************************************************************************/
void print(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group, uint8_t printmsg[], uint16_t size)
{

	uart_transfer_t xfer;

	/* Send message out. */
	xfer.data = printmsg;
	xfer.dataSize = size;

	UART_TransferSendNonBlocking(xuart, uart_handle, &xfer);

	xEventGroupWaitBits(event_group, EVENT_TX, pdTRUE, pdTRUE,
	portMAX_DELAY);

}

/*******************************************************************************
 * TASKS CODE
 ******************************************************************************/


/*******************************************************************************
 * TASKS CODE
 ******************************************************************************/
uint8_t read_from_keyboard(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group)
{
	static uart_transfer_t sendXfer;
	static uart_transfer_t receiveXfer;
	static uint8_t g_txBuffer;
	static uint8_t g_rxBuffer;

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = 1;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = 1;

	/* The first UART task which is called, prepares to receive but the buffer is not ready
	 * until an interrupt occurs. */
	UART_TransferReceiveNonBlocking(xuart, uart_handle, &receiveXfer, NULL);

	/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
	 * it could be interrupted in the middle of setting values and other task may corrupt the
	 * receive buffer for this reason we use MUTEX to protect the UART. */
	xEventGroupWaitBits(event_group, EVENT_RX, pdTRUE, pdTRUE,
	portMAX_DELAY);

	/* Fills the transmission buffer. */
	g_txBuffer = g_rxBuffer;

	if (validate_number(g_txBuffer))
	{
		/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
		UART_TransferSendNonBlocking(xuart, uart_handle, &sendXfer);

		/* In case another task want to send the MUTEX must be given before */
		xEventGroupWaitBits(event_group,
		EVENT_TX, pdTRUE, pdTRUE, portMAX_DELAY);

		return g_txBuffer - '0';

	}

	g_txBuffer = 0;
	return FALSE;

}

void wait_for_esc(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group)
{
	static uart_transfer_t receiveXfer;
	static uint8_t g_rxBuffer;

    /* Start to echo. */
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = 1;
	while('\r' != g_rxBuffer)
	{
	/* The first UART task which is called, prepares to receive but the buffer is not ready
	 * until an interrupt occurs. */
	UART_TransferReceiveNonBlocking(xuart, uart_handle, &receiveXfer, NULL);

	/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
	 * it could be interrupted in the middle of setting values and other task may corrupt the
	 * receive buffer for this reason we use MUTEX to protect the UART. */
	xEventGroupWaitBits(event_group, EVENT_RX, pdTRUE, pdTRUE,
	portMAX_DELAY);
	}

	g_rxBuffer = 0;

}

int16_t read_from_keyboard1(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group)
{
	static uart_transfer_t sendXfer;
	static uart_transfer_t receiveXfer;
	static uint8_t g_txBuffer;
	static uint8_t g_rxBuffer;
	uint8_t times = 0;
	uint16_t subaddress = 0;

	/* Start to echo. */
	sendXfer.data = &g_txBuffer;
	sendXfer.dataSize = 1;
	receiveXfer.data = &g_rxBuffer;
	receiveXfer.dataSize = 1;

	while (4 != times)
	{

		/* The first UART task which is called, prepares to receive but the buffer is not ready
		 * until an interrupt occurs. */
		UART_TransferReceiveNonBlocking(xuart, uart_handle, &receiveXfer, NULL);

		/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
		 * it could be interrupted in the middle of setting values and other task may corrupt the
		 * receive buffer for this reason we use MUTEX to protect the UART. */
		xEventGroupWaitBits(event_group, EVENT_RX, pdTRUE, pdTRUE,
		portMAX_DELAY);

		/* Fills the transmission buffer. */
		g_txBuffer = g_rxBuffer;

		if (-1 != validation_address(g_txBuffer) && 4 > times)
		{

			/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
			UART_TransferSendNonBlocking(xuart, uart_handle, &sendXfer);

			/* In case another task want to send the MUTEX must be given before */
			xEventGroupWaitBits(event_group,
			EVENT_TX, pdTRUE, pdTRUE, portMAX_DELAY);

			subaddress += validation_address(g_txBuffer) << (4 * (3 - times));
			times++;
		}

		g_txBuffer = 0;

	}

	return subaddress;
}


uart_transfer_t read_from_keyboard2(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group)
{
	static uart_transfer_t sendXfer;
	static uart_transfer_t receiveXfer;
	uart_transfer_t returnXfer;
	static uint8_t g_txBuffer;
	static uint8_t g_rxBuffer;
	uint8_t buffer[50];
	uint8_t buffer_index = 0;

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = 1;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = 1;

    while('\r'!=g_txBuffer)
    {


    		/* The first UART task which is called, prepares to receive but the buffer is not ready
    		 * until an interrupt occurs. */
    		UART_TransferReceiveNonBlocking(xuart, uart_handle, &receiveXfer, NULL);

    		/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
    		 * it could be interrupted in the middle of setting values and other task may corrupt the
    		 * receive buffer for this reason we use MUTEX to protect the UART. */
    		xEventGroupWaitBits(event_group, EVENT_RX, pdTRUE, pdTRUE,
    		portMAX_DELAY);


    		/* Fills the transmission buffer. */
    		g_txBuffer = g_rxBuffer;

    		if('\r' == g_rxBuffer)
    		{

    		}
    		else if ('\b' == g_txBuffer)
    		{
    			if(0 < buffer_index)
    			{
    				buffer_index--;
    			}
    			/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
    			UART_TransferSendNonBlocking(xuart, uart_handle, &sendXfer);

    			/* In case another task want to send the MUTEX must be given before */
    			xEventGroupWaitBits(event_group,
    			EVENT_TX, pdTRUE, pdTRUE, portMAX_DELAY);

    		}

    		else if (50 > buffer_index)
    		{
    			buffer[buffer_index] = g_txBuffer;
    			buffer_index++;

    			/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
    			UART_TransferSendNonBlocking(xuart, uart_handle, &sendXfer);

    			/* In case another task want to send the MUTEX must be given before */
    			xEventGroupWaitBits(event_group,
    			EVENT_TX, pdTRUE, pdTRUE, portMAX_DELAY);
    		}

    }
    g_txBuffer = 0;
    returnXfer.data = storing_phrase(buffer_index, buffer);
    returnXfer.dataSize = buffer_index;

    return returnXfer;

}


uint16_t read_from_keyboard3(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group)
{
	static uart_transfer_t sendXfer;
	static uart_transfer_t receiveXfer;
	static uint8_t g_txBuffer;
	static uint8_t g_rxBuffer;
	uint8_t index;
	uint16_t multiplier = 1;
	uint16_t length_bytes = 0;
	uint8_t times = 0;
	uint8_t numbytes[5] = {0};

	/* Start to echo. */
	sendXfer.data = &g_txBuffer;
	sendXfer.dataSize = 1;
	receiveXfer.data = &g_rxBuffer;
	receiveXfer.dataSize = 1;

	while ('\r' != g_txBuffer)
	{

		/* The first UART task which is called, prepares to receive but the buffer is not ready
		 * until an interrupt occurs. */
		UART_TransferReceiveNonBlocking(xuart, uart_handle, &receiveXfer, NULL);

		/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
		 * it could be interrupted in the middle of setting values and other task may corrupt the
		 * receive buffer for this reason we use MUTEX to protect the UART. */
		xEventGroupWaitBits(event_group, EVENT_RX, pdTRUE, pdTRUE,
		portMAX_DELAY);

		/* Fills the transmission buffer. */
		g_txBuffer = g_rxBuffer;

		if (-1 != validatation_number(g_txBuffer) && 5 > times)
		{

			/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
			UART_TransferSendNonBlocking(xuart, uart_handle, &sendXfer);

			/* In case another task want to send the MUTEX must be given before */
			xEventGroupWaitBits(event_group,
			EVENT_TX, pdTRUE, pdTRUE, portMAX_DELAY);

			numbytes[times] = validation_address(g_txBuffer);
			times++;
		}

	}
	g_txBuffer = 0;
	for(index = times; index; index--)
	{
		length_bytes += numbytes[index-1] * (multiplier);
		multiplier *= 10;
	}

	return length_bytes;
}

uint8_t validate_number(uint8_t keyboard_data)
{
	if ('0' < keyboard_data && '9' >= keyboard_data)
	{
		return TRUE;
	}

	else
	{
		return FALSE;
	}
}


int8_t validatation_number(uint8_t keyboard_data)
{

	if ('0' <= keyboard_data && '9' >= keyboard_data)
	{
		return keyboard_data - '0';
	}

	else
	{
		return -1;
	}

}

int8_t validation_address(uint8_t keyboard_data)
{
	if ('0' <= keyboard_data && '9' >= keyboard_data)
	{
		return keyboard_data - '0';
	}

	else if('A' <= keyboard_data && 'F' >= keyboard_data)
	{
		return keyboard_data - 55;
	}

	else if ('a' <= keyboard_data && 'f' >= keyboard_data)
	{
		return keyboard_data - 87;
	}

	else
	{
		return -1;
	}

}

uint8_t* storing_phrase(uint8_t phrase_length, uint8_t * phrase)
{
	uint8_t str_index = 0;
	uint8_t * string;

	string = (uint8_t *)pvPortMalloc(phrase_length * sizeof(uint8_t));

	if(string == NULL)
	{
		return NULL;
	}

	for(str_index = 0; str_index < phrase_length; str_index++)
	{
		*(string + str_index) = *(phrase + str_index);
	}

	return string;
}
/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/


