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
#include "pin_mux.h"
#include "clock_config.h"
#include "chat_app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
/*******************************************************************************
 * Structures
 ******************************************************************************/
typedef struct {

	UART_Type * xuart;
	uart_handle_t uart_handle;
	uint8_t rx_event;
	uint8_t tx_event;

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
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void UART_UserCallback_ter(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

void UART_UserCallback_bt(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
EventGroupHandle_t uart_events_g;

/*******************************************************************************
 * TASKS CODE
 ******************************************************************************/
//void uart_transmitter_task(void * pvParameters)
//{
//
//	static uart_transfer_t xfer;
//	static uart_transfer_t receiveXfer;
//	static uint8_t data;
//	static uint8_t g_tipString[] =
//	    "\r\nYou have entered a valid value\r\n";
//	uart_parameters_type * uart_param = (uart_parameters_type *) pvParameters;
//
//	/* Send g_tipString out. */
//	xfer.data = g_tipString;
//	xfer.dataSize = sizeof(g_tipString) - 1;
//	receiveXfer.data = &data;
//	receiveXfer.dataSize = 1;
//    UART_TransferSendNonBlocking(uart_param->xuart,
//            &(uart_param->uart_handle), &xfer);
//
//    xEventGroupWaitBits(uart_events_g, uart_param->tx_event, pdTRUE, pdTRUE,
//            portMAX_DELAY);
//	for(;;)
//	{
//    	/* UART0 and UART1 are different peripherals
//    	 * in case we are using the same UART for both tasks
//    	 * we should protect it with a MUTEX*/
//
//        /* The first UART task which is called, prepares to receive but the buffer is not ready
//         * until an interrupt occurs. */
//    	UART_TransferReceiveNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &receiveXfer, NULL);
//
//    	/* This will sleep the task till callback set the event bit. As this is not an atomic instruction,
//    	 * it could be interrupted in the middle of setting values and other task may corrupt the
//    	 * receive buffer for this reason we use MUTEX to protect the UART. */
//    	xEventGroupWaitBits(uart_events_g, uart_param->rx_event, pdTRUE, pdTRUE, portMAX_DELAY);
//    	if('0' < data && '9' >= data)
//    	{
//            xEventGroupSetBits(uart_events_g, EVENT_ECHO);
//            vTaskDelay(portMAX_DELAY);
//    	}
//
//	}
//
//}


void uart_transceiver_task(void * pvParameters)
{
	static uart_transfer_t sendXfer;
	static uart_transfer_t receiveXfer;
	static uint8_t g_txBuffer;
	static uint8_t g_rxBuffer;

	uart_parameters_type * uart_param = (uart_parameters_type *) pvParameters;

#ifdef debug_store_rx_uart
	static uint8_t phrase_size = 0;
    static uint8_t phrase[50] = {0};
#endif

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

    //xEventGroupWaitBits(uart_events_g, EVENT_ECHO, pdTRUE, pdFALSE, portMAX_DELAY);
    /* When no receiving transfers are happening this task will be suspend and whoever
     * take the semaphore first would be executed first. */
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

    	/* Fills the transmission buffer. */
    	g_txBuffer = g_rxBuffer;

#ifdef debug_store_rx_uart
    	if (('\r' != *(sendXfer.data)) && ('\b' != *(sendXfer.data))
                && (phrase_size < 50))
    	{
            *(phrase + phrase_size) = *(sendXfer.data);
            phrase_size++;
        }
        else if ('\b' == *(sendXfer.data) && phrase_size < 50)
        {
            phrase_size = (phrase_size > 0) ? phrase_size - 1 : 0;
        }
        else if ('\r' == *(sendXfer.data) && phrase_size < 50)
        {
            phrase_size = 0;
            /* Save phrase and use it for something. */
        }
        else
        {
            phrase_size = 0;
        }
#endif

    	/* Prepares to send it. Then again only 1 task should be able to use this resource at the time. */
    	UART_TransferSendNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &sendXfer);

    	/* In case another task want to send the MUTEX must be given before */
    	xEventGroupWaitBits(uart_events_g, uart_param->tx_event, pdTRUE, pdTRUE, portMAX_DELAY);

    	xEventGroupSetBits(get_menu_event(), 1<<(g_txBuffer - '0'));

    	/**In case that reception buffer is equal to [ESC] */
    	//if('\e' == g_txBuffer)
    	//{
    		vTaskDelay(portMAX_DELAY);
    	//}

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
	//cpu.uart_handle = g_uartHandle_ter;
	cpu.xuart = UART0;

	bluetooth.rx_event = EVENT_BT_RX;
	bluetooth.tx_event = EVENT_BT_TX;
	//bluetooth.uart_handle = g_uartHandle_bt;
	bluetooth.xuart = UART1;

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

//	xTaskCreate(uart_transmitter_task, "print_task", 110, (void *) &cpu,
//	configMAX_PRIORITIES, NULL);

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

