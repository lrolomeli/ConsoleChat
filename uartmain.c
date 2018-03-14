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

#include "board.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "freeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

void UART_UserCallback1(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uart_handle_t g_uartHandle;
uart_parameters_type cpu;

uart_handle_t g_uartHandle1;

SemaphoreHandle_t tx_semaphore;
EventGroupHandle_t uart_events_g;

uint8_t g_tipString[] =
    "\r\nYou have entered a valid value\r\n";

uint8_t g_txBuffer = 0;
uint8_t g_rxBuffer = 0;


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */

void uart_transmitter_task(void * pvParameters)
{
	/**
	 * Cuando queramos mandar un mensaje por pantalla
	 * es necesario despertar esta funcion por medio de un give
	 * al semaforo tx_semaphore
	 */
	static uart_transfer_t xfer;
	uart_parameters_type * uart_param = (uart_parameters_type *) pvParameters;
	/* Send g_tipString out. */
	xfer.data = g_tipString;
	xfer.dataSize = sizeof(g_tipString) - 1;

#if debug_print_task_mutex_sem
	volatile bool first_time = true;
#endif

	for(;;)
	{
#if debug_print_task_mutex_sem
		/* example_printf. */
		if(first_time)//si se quiere imprimir se manda a la funcion un booleano casteado
		{
			first_time = false;
		}
#endif
	    	UART_TransferSendNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &xfer);
	    	xEventGroupWaitBits(uart_events_g, uart_param->tx_event, pdTRUE, pdTRUE, portMAX_DELAY);
	    	vTaskDelete(NULL);

	}

}


void uart_transceiver_task(void * pvParameters)
{
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;
	uart_parameters_type * uart_param = (uart_parameters_type *) pvParameters;

//	static uint8_t phrase_size = 0;
//	static uint8_t phrase[50] =
//	{ 0 };

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;



    for(;;)
    {
    	UART_TransferReceiveNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &receiveXfer, NULL);
    	xEventGroupWaitBits(uart_events_g, uart_param->rx_event, pdTRUE, pdTRUE, portMAX_DELAY);
    	g_txBuffer = g_rxBuffer;
    	UART_TransferSendNonBlocking(uart_param->xuart, &(uart_param->uart_handle), &sendXfer);
    	xEventGroupWaitBits(uart_events_g, uart_param->tx_event, pdTRUE, pdTRUE, portMAX_DELAY);
    }


//	for(;;)
//	{
//		/* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
//		    /* Si no se encuentra recibiendo y el buffer de recepcion esta vacio. */
//		    if ((!rxOnGoing) && rxBufferEmpty)
//		    {
//		        /**entonces puedes decir que ahora ya se encuentra recibiendo*/
//		        rxOnGoing = true;
//		        /**puedes empezar a recibir*/
//		        UART_TransferReceiveNonBlocking(DEMO_UART, &g_uartHandle, &receiveXfer, NULL);
//		    }
//
//		    /* If TX is idle and g_txBuffer is full, start to send data. */
//		    /* Si no se encuentra transmitiendo y el buffer de transmision esta lleno. */
//		    if ((!txOnGoing) && txBufferFull)
//		    {
//
//		    	if('\r' != *(sendXfer.data) && '\b' != *(sendXfer.data) && phrase_size < 50)
//		    	{
//		    		*(phrase + phrase_size) = *(sendXfer.data);
//		    		phrase_size++;
//		    	}
//		    	else if('\b' == *(sendXfer.data) && phrase_size < 50)
//		    	{
//		    		phrase_size = (phrase_size > 0) ? phrase_size - 1 : 0;
//		    	}
//		    	else if('\r' == *(sendXfer.data) && phrase_size < 50)
//		    	{
//		    		phrase_size = 0;
//		    		//guardar frase y utilizarla
//		    	}
//		    	else
//		    	{
//		    		phrase_size = 0;
//		    	}
//
//		        /**entonces ya se encuentra transmitiendo*/
//		        txOnGoing = true;
//		        /**comienza a transmitir lo que hay en el buffer*/
//		        UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &sendXfer);
//
//		    }
//
//		    /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
//		    /* Si el buffer de recepcion no esta vacio y el buffer de transmision tampoco. */
//		    if ((!rxBufferEmpty) && (!txBufferFull))
//		    {
//		        /**entonces copia lo que hay en el buffer de recepcion al de transmision*/
//		        memcpy(&g_txBuffer, &g_rxBuffer, ECHO_BUFFER_LENGTH);
//		        /**el buffer de recepcion esta vacio*/
//		        rxBufferEmpty = true;
//		        /**el buffer de transmision esta lleno*/
//		        txBufferFull = true;
//		    }
//	}


}

int main(void)
{
    uart_config_t config;

    uart_config_t config1;



	uart_parameters_type bluetooth;

	cpu.rx_event = EVENT_RX;
	cpu.tx_event = EVENT_TX;
	cpu.uart_handle = g_uartHandle;
	cpu.xuart = UART0;

	bluetooth.rx_event = EVENT_BT_RX;
	bluetooth.tx_event = EVENT_BT_TX;
	bluetooth.uart_handle = g_uartHandle1;
	bluetooth.xuart = UART1;

    BOARD_InitPins();
    BOARD_BootClockRUN();

    //reloj uart clockenable
    //pins tx and rx config as uart

    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_Uart0);

    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_Uart1);

	PORT_SetPinMux(PORTB, 10, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTB, 11, kPORT_MuxAlt3);

	PORT_SetPinMux(PORTC, 3, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTC, 4, kPORT_MuxAlt3);


    UART_GetDefaultConfig(&config);
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    UART_TransferCreateHandle(DEMO_UART, &cpu.uart_handle, UART_UserCallback, NULL);

	xTaskCreate(uart_transceiver_task, "echo_task", configMINIMAL_STACK_SIZE, (void *) &cpu,
	        configMAX_PRIORITIES, NULL);


    UART_GetDefaultConfig(&config1);
    config1.enableTx = true;
    config1.enableRx = true;
    config1.baudRate_Bps = 9600;

    UART_Init(UART1, &config1, CLOCK_GetFreq(UART1_CLK_SRC));
    UART_TransferCreateHandle(UART1, &bluetooth.uart_handle, UART_UserCallback1, NULL);

	xTaskCreate(uart_transceiver_task, "bluetooth", configMINIMAL_STACK_SIZE, (void *) &bluetooth,
	        configMAX_PRIORITIES-1, NULL);

	xTaskCreate(uart_transmitter_task, "print_task", 110, (void *) &cpu,
	        configMAX_PRIORITIES - 2, NULL);


	uart_events_g = xEventGroupCreate();
    NVIC_EnableIRQ(UART0_RX_TX_IRQn);
    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);

    NVIC_EnableIRQ(UART1_RX_TX_IRQn);
    NVIC_SetPriority(UART1_RX_TX_IRQn, 6);

	vTaskStartScheduler();

    for(;;)
    {

    }
}

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
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

void UART_UserCallback1(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
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

