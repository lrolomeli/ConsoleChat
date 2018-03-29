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
static uint8_t terminal_menu[] =
				"\r\n(1) Leer Memoria I2C\r\n(2) Escribir Memoria I2C"
		        "\r\n(3) Establecer Hora\r\n(4) Establecer Fecha"
		        "\r\n(5) Formato de Hora\r\n(6) Leer Hora"
		        "\r\n(7) Leer Fecha\r\n(8) Comunicacion con terminal 2"
		        "\r\n(9) Eco en LCD\r\n";
/*******************************************************************************
 * TASKS CODE
 ******************************************************************************/
void send_menu_task(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group)
{

	static uart_transfer_t xfer;

	/* Send message out. */
	xfer.data = terminal_menu;
	xfer.dataSize = sizeof(terminal_menu) - 1;

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
uint8_t read_menu_from_keyboard(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group)
{
	static uart_transfer_t sendXfer;
	static uart_transfer_t receiveXfer;
	static uint8_t g_txBuffer;
	static uint8_t g_rxBuffer;
	static uint8_t menu;

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = 1;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = 1;

    /* When no receiving transfers are happening this task will be suspend and whoever. */
    for(;;)
    {

		/* The first UART task which is called, prepares to receive but the buffer is not ready
		 * until an interrupt occurs. */
		UART_TransferReceiveNonBlocking(xuart,
		        uart_handle, &receiveXfer, NULL);

		/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
		 * it could be interrupted in the middle of setting values and other task may corrupt the
		 * receive buffer for this reason we use MUTEX to protect the UART. */
		xEventGroupWaitBits(event_group, EVENT_RX, pdTRUE, pdTRUE,
		        portMAX_DELAY);

		/* Fills the transmission buffer. */
		g_txBuffer = g_rxBuffer;

		menu = validate_number(g_txBuffer);

		if (menu)
		{
			/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
			UART_TransferSendNonBlocking(xuart,
			        uart_handle, &sendXfer);

			/* In case another task want to send the MUTEX must be given before */
			xEventGroupWaitBits(event_group, EVENT_TX, pdTRUE,
			        pdTRUE,
			        portMAX_DELAY);

			return menu;

		}

    }

}

uint8_t validate_number(uint8_t keyboard_data)
{
	if ('0' < keyboard_data && '9' >= keyboard_data)
	{
		return keyboard_data - '0';
	}

	else
	{
		return FALSE;
	}
}

#if 0
uint8_t validate_number(uint8_t keyboard_data)
{
	static uint8_t counter = 0;

	if ('0' < keyboard_data && '9' >= keyboard_data && counter == 0)
	{

		menu = keyboard_data - '0';
		counter++;

		return menu;

	}
	else if ('\r' == keyboard_data)
	{
		counter = 0;

		return menu;

	}
	else if ('\b' == keyboard_data)
	{
		/*reduce */
		counter--;
		return FALSE;
	}
	else if ('\e' == keyboard_data)
	{
		counter = 0;
		return FALSE;
	}

	else
	{
		return FALSE;
	}

}
#endif
/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/


