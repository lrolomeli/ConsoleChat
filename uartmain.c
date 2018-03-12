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

#include "pin_mux.h"
#include "clock_config.h"
#include "freeRTOS.h"
#include "task.h"
#include "semphr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART3
#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)
#define ECHO_BUFFER_LENGTH 1

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uart_handle_t g_uartHandle;
SemaphoreHandle_t tx_semaphore;

uint8_t g_tipString[] =
    "\r\nYou have entered a valid value\r\n";

uint8_t g_txBuffer = 0;
uint8_t g_rxBuffer = 0;
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
#if debug_print_task_mutex_sem
void uart_transmitter_task(void * pvParameters)
{
	/**
	 * Cuando queramos mandar un mensaje por pantalla
	 * es necesario despertar esta funcion por medio de un give
	 * al semaforo tx_semaphore
	 */
	static uart_transfer_t xfer;
	/* Send g_tipString out. */
	xfer.data = g_tipString;
	xfer.dataSize = sizeof(g_tipString) - 1;
	volatile bool first_time = true;

	for(;;)
	{
		/* example_printf. */
		if(first_time)//si se quiere imprimir se manda a la funcion un booleano casteado
		{
			xSemaphoreTake(tx_semaphore, portMAX_DELAY);
			txOnGoing = true;   //EL DATO ESTA LISTO PARA SER ENVIADO
			UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);
			first_time = false;
		}

	}

}
#endif

void uart_transceiver_task(void * pvParameters)
{
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;
	static uint8_t phrase_size = 0;
	static uint8_t phrase[50] =
	{ 0 };

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

	for(;;)
	{
		/* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
		    /* Si no se encuentra recibiendo y el buffer de recepcion esta vacio. */
		    if ((!rxOnGoing) && rxBufferEmpty)
		    {
		        /**entonces puedes decir que ahora ya se encuentra recibiendo*/
		        rxOnGoing = true;
		        /**puedes empezar a recibir*/
		        UART_TransferReceiveNonBlocking(DEMO_UART, &g_uartHandle, &receiveXfer, NULL);
		    }

		    /* If TX is idle and g_txBuffer is full, start to send data. */
		    /* Si no se encuentra transmitiendo y el buffer de transmision esta lleno. */
		    if ((!txOnGoing) && txBufferFull)
		    {

		    	if('\r' != *(sendXfer.data) && '\b' != *(sendXfer.data) && phrase_size < 50)
		    	{
		    		*(phrase + phrase_size) = *(sendXfer.data);
		    		phrase_size++;
		    	}
		    	else if('\b' == *(sendXfer.data) && phrase_size < 50)
		    	{
		    		phrase_size = (phrase_size > 0) ? phrase_size - 1 : 0;
		    	}
		    	else if('\r' == *(sendXfer.data) && phrase_size < 50)
		    	{
		    		phrase_size = 0;
		    		//guardar frase y utilizarla
		    	}
		    	else
		    	{
		    		phrase_size = 0;
		    	}

		        /**entonces ya se encuentra transmitiendo*/
		        txOnGoing = true;
		        /**comienza a transmitir lo que hay en el buffer*/
		        UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &sendXfer);

		    }

		    /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
		    /* Si el buffer de recepcion no esta vacio y el buffer de transmision tampoco. */
		    if ((!rxBufferEmpty) && (!txBufferFull))
		    {
		        /**entonces copia lo que hay en el buffer de recepcion al de transmision*/
		        memcpy(&g_txBuffer, &g_rxBuffer, ECHO_BUFFER_LENGTH);
		        /**el buffer de recepcion esta vacio*/
		        rxBufferEmpty = true;
		        /**el buffer de transmision esta lleno*/
		        txBufferFull = true;
		    }
	}


}

int main(void)
{
    uart_config_t config;

    BOARD_InitPins();
    BOARD_BootClockRUN();

    //reloj uart clockenable
    //pins tx and rx config as uart

    CLOCK_EnableClock(kCLOCK_PortB);


    UART_GetDefaultConfig(&config);
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    UART_TransferCreateHandle(DEMO_UART, &g_uartHandle, UART_UserCallback, NULL);

    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
    //NVICinterruptuartirqpriority > 5

    tx_semaphore = xSemaphoreCreateMutex();

	xTaskCreate(uart_transceiver_task, "echo_task", configMINIMAL_STACK_SIZE, NULL,
	        configMAX_PRIORITIES, NULL);

#if debug_print_task_mutex_sem
	xTaskCreate(uart_transmitter_task, "print_task", configMINIMAL_STACK_SIZE, NULL,
	        configMAX_PRIORITIES - 1, NULL);
#endif

	vTaskStartScheduler();

    for(;;)
    {

    }
}

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    //basetype  higher priority
    if (kStatus_UART_TxIdle == status)
    {
    	//event group set bits from isr for tx
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_UART_RxIdle == status)
    {
    	//event group set bits from isr for rx
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}
