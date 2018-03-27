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

#include "terminal.h"
#include "board.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "chat_app_main.h"

/*******************************************************************************
 * Structures
 ******************************************************************************/
typedef enum {

	FALSE = 0, TRUE

} boolean_type;

typedef struct {

	UART_Type * xuart;
	uart_handle_t uart_handle;
	uint8_t rx_event;
	uint8_t tx_event;
	uint8_t menu_event;

} uart_parameters_type;
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART0
#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)
#define ECHO_BUFFER_LENGTH 1

#define EVENT_RX (1 << 0)
#define EVENT_TX (1 << 1)

#define EVENT_BT_RX (1 << 2)
#define EVENT_BT_TX (1 << 3)

#define EVENT_ECHO (1 << 4)
#define EVENT_ECHO_BLUETOOTH (1 << 5)

#define EVENT_MENU_CPU (1 << 6)
#define EVENT_MENU_BT (1 << 7)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
uint8_t validation_routine(uint8_t * variable, uint8_t length);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t terminal_menu[] =
        "\r\n(1) Escribir en Memoria\r\n(1) Escribir en Memoria"
        "\r\n(1) Escribir en Memoria\r\n(1) Escribir en Memoria"
        "\r\n(1) Escribir en Memoria\r\n(1) Escribir en Memoria"
        "\r\n(1) Escribir en Memoria\r\n(1) Escribir en Memoria\r\n";
static uint8_t error_message[] =
        "\r\nWrong Selection, try again\r\n";
EventGroupHandle_t uart_events_g;

/*******************************************************************************
 * CALLBACKS
 ******************************************************************************/

/* UART terminal user callback */
void UART_UserCallback_ter(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_UART_TxIdle == status)
    {
    	//event group set bits from isr for tx
    	xEventGroupSetBitsFromISR(uart_events_g, EVENT_TX, &xHigherPriorityTaskWoken);
    }

    if (kStatus_UART_RxIdle == status)
    {
    	//event group set bits from isr for rx
    	xEventGroupSetBitsFromISR(uart_events_g, EVENT_RX, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/* UART bluetooth user callback */
void UART_UserCallback_bt(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_UART_TxIdle == status)
    {
    	//event group set bits from isr for tx
    	xEventGroupSetBitsFromISR(uart_events_g, EVENT_BT_TX, &xHigherPriorityTaskWoken);
    }

    if (kStatus_UART_RxIdle == status)
    {
    	//event group set bits from isr for rx
    	xEventGroupSetBitsFromISR(uart_events_g, EVENT_BT_RX, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/*******************************************************************************
 * TASKS CODE
 ******************************************************************************/
void uart_transmitter_task(void * pvParameters)
{

	static uart_transfer_t xfer;

	uart_parameters_type * uart_param = (uart_parameters_type *) pvParameters;

	/* Send message out. */
	xfer.data = terminal_menu;
	xfer.dataSize = sizeof(terminal_menu) - 1;


	for(;;)
	{
	    UART_TransferSendNonBlocking(uart_param->xuart,
	            &(uart_param->uart_handle), &xfer);

	    xEventGroupWaitBits(uart_events_g, uart_param->tx_event, pdTRUE, pdTRUE,
	            portMAX_DELAY);

	    xEventGroupWaitBits(uart_events_g, uart_param->menu_event, pdTRUE, pdTRUE,
	            portMAX_DELAY);
	}

}


void uart_transceiver_task(void * pvParameters)
{
	static uint8_t menu = 0;
	static uart_transfer_t sendXfer;
	static uart_transfer_t receiveXfer;
	static uart_transfer_t errorXfer;
	static uint8_t g_txBuffer;
	static uint8_t g_rxBuffer;
	static uint8_t phrase_size = 0;
	static uint8_t phrase[50] =
	{ 0 };

	uart_parameters_type * uart_param = (uart_parameters_type *) pvParameters;

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
    errorXfer.data = error_message;
    errorXfer.dataSize = sizeof(error_message) - 1;

    /* When no receiving transfers are happening this task will be suspend and whoever. */
    for(;;)
    {

    	/* UART0 and UART1 are different peripherals
    	 * in case we are using the same UART for both tasks
    	 * we should protect it with a MUTEX*/

        /* The first UART task which is called, prepares to receive but the buffer is not ready
         * until an interrupt occurs. */
    	UART_TransferReceiveNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &receiveXfer, NULL);

    	/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
    	 * it could be interrupted in the middle of setting values and other task may corrupt the
    	 * receive buffer for this reason we use MUTEX to protect the UART. */
    	xEventGroupWaitBits(uart_events_g, uart_param->rx_event, pdTRUE, pdTRUE, portMAX_DELAY);

    	if ('\r' == g_rxBuffer)
		{
			//rutina de validacion de variable
			if (0 == menu && validation_routine(phrase, phrase_size, menu))
			{
				//manda a llamar a la tarea que elegiste
				menu = phrase[0] - '0';
				xEventGroupSetBits(get_menu_event(), 1 << (menu));
				phrase_size = 0;
			}

			else if(1 == menu && validation_routine(phrase, phrase_size, menu))
			{
				//queue send to back non blocking

				phrase_size = 0;
			}

			else
			{
				phrase_size = 0;
		    	/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
		    	UART_TransferSendNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &errorXfer);

		    	/* In case another task want to send the MUTEX must be given before */
		    	xEventGroupWaitBits(uart_events_g, uart_param->tx_event, pdTRUE, pdTRUE, portMAX_DELAY);
			}
		}

		else if ('\b' == g_rxBuffer)
		{
			phrase_size--;
		}
		else if ('\e' == g_rxBuffer)
		{
			phrase_size = 0;
			//verificas el menu y regresas al anterior
			//en caso de estar en el menu principal
			//mandas llamar a la tarea imprimir menu
		}
		else
		{
	    	/* Fills the transmission buffer. */
	    	g_txBuffer = g_rxBuffer;

	    	/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
	    	UART_TransferSendNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &sendXfer);

	    	/* In case another task want to send the MUTEX must be given before */
	    	xEventGroupWaitBits(uart_events_g, uart_param->tx_event, pdTRUE, pdTRUE, portMAX_DELAY);

			if (15 > phrase_size)
			{
				phrase[phrase_size] = g_txBuffer;
				phrase_size++;
			}

		}

    }

}

/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
void terminal_init(void)
{
	static uart_parameters_type cpu;
	static uart_parameters_type bluetooth;

	static uart_config_t config_ter;
    static uart_config_t config_bt;

	/*******************************************************************************
	 * TASKS PARAMETERS
	 ******************************************************************************/
	cpu.rx_event = EVENT_RX;
	cpu.tx_event = EVENT_TX;
	cpu.xuart = UART0;
	cpu.menu_event = EVENT_MENU_CPU;

	bluetooth.rx_event = EVENT_BT_RX;
	bluetooth.tx_event = EVENT_BT_TX;
	bluetooth.xuart = UART1;
	bluetooth.menu_event = EVENT_MENU_BT;

	/*******************************************************************************
	 * GPIO UART CONFIGURATION
	 ******************************************************************************/
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_Uart0);

    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_Uart1);

	PORT_SetPinMux(PORTB, 10, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTB, 11, kPORT_MuxAlt3);

	PORT_SetPinMux(PORTC, 3, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTC, 4, kPORT_MuxAlt3);

	/*******************************************************************************
	 * UART BLUETOOTH OR TERMINAL CONFIGURATION
	 ******************************************************************************/
    UART_GetDefaultConfig(&config_ter);
    config_ter.enableTx = true;
    config_ter.enableRx = true;

    UART_GetDefaultConfig(&config_bt);
    config_bt.enableTx = true;
    config_bt.enableRx = true;
    config_bt.baudRate_Bps = 9600;

	/*******************************************************************************
	 * BLUETOOTH AND TERMINAL INITIALIZATION AND HANDLER CREATION
	 ******************************************************************************/
    UART_Init(DEMO_UART, &config_ter, DEMO_UART_CLK_FREQ);
	UART_Init(UART1, &config_bt, CLOCK_GetFreq(UART1_CLK_SRC));


	UART_TransferCreateHandle(DEMO_UART, &cpu.uart_handle, UART_UserCallback_ter,
			NULL);
	UART_TransferCreateHandle(UART1, &bluetooth.uart_handle, UART_UserCallback_bt,
			NULL);

	/*******************************************************************************
	 * TASKS OF THE MODULE
	 ******************************************************************************/
	xTaskCreate(uart_transceiver_task, "echo_task", configMINIMAL_STACK_SIZE,
			(void *) &cpu,
			configMAX_PRIORITIES - 1, NULL);

	xTaskCreate(uart_transceiver_task, "bluetooth", configMINIMAL_STACK_SIZE,
			(void *) &bluetooth,
			configMAX_PRIORITIES - 1, NULL);

	xTaskCreate(uart_transmitter_task, "teraterm_menu", 110, (void *) &cpu,
	configMAX_PRIORITIES, NULL);

	xTaskCreate(uart_transmitter_task, "bluetooth_menu", 110, (void *) &bluetooth,
	configMAX_PRIORITIES, NULL);

	/*******************************************************************************
	 * INTERRUPT HABILITATION
	 ******************************************************************************/
    NVIC_EnableIRQ(UART0_RX_TX_IRQn);
    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);

    NVIC_EnableIRQ(UART1_RX_TX_IRQn);
    NVIC_SetPriority(UART1_RX_TX_IRQn, 6);

	/*******************************************************************************
	 * EVENTS CREATION
	 ******************************************************************************/
	uart_events_g = xEventGroupCreate();

}

uint8_t validation_routine(uint8_t * variable, uint8_t length, uint8_t menu)
{
	static uint8_t index;



	//checar cual es el menu encendido

	if(0 == menu)
	{
		if(1 == length)
		{
			if('0' < variable[0] && '9' >= variable[0])
			{
				return TRUE;
			}
		}
		return FALSE;
	}

	if(1 == menu)
	{
		//en caso de ser el menu 1 submenu 0
		if(5 > length)
		{
			for(index=0; index<length; i++)
			{
				variable[index] = variable[index] - '0';
			}
			return TRUE;
		}
		return FALSE;
	}






	//en caso de ser el menu 2 submenu 0
}

#ifdef dynamic_alloc
uint8_t* storing_phrase(uint8_t phrase_length, uint8_t * phrase)
{
	uint8_t str_index = 0;
	uint8_t * string;

	string = (uint8_t *)pvPortMalloc(phrase_length * sizeof(uint8_t));

	if(string == NULL)
	{
		return;
	}

	for(str_index = 0; str_index < phrase_length; str_index++)
	{
		*(string + str_index) = *(phrase + str_index);
	}

	//vPortFree(string);
	return string;
}
#endif

EventGroupHandle_t get_uart_event(void)
{

	return uart_events_g;

}
