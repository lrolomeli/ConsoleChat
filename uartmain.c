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
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART0
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
int main(void)
{
    uart_config_t config;
    uart_transfer_t xfer;
    uart_transfer_t sendXfer;
    uart_transfer_t receiveXfer;
    static uint8_t phrase[50] = {0};
    static uint8_t i;


    BOARD_InitPins();
    BOARD_BootClockRUN();

    UART_GetDefaultConfig(&config);
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    UART_TransferCreateHandle(DEMO_UART, &g_uartHandle, UART_UserCallback, NULL);

    /* Send g_tipString out. */
    xfer.data = g_tipString;
    xfer.dataSize = sizeof(g_tipString) - 1;

#if example_printf
    txOnGoing = true;   //EL DATO ESTA LISTO PARA SER ENVIADO
    UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);

    /* Wait send finished */
    while (txOnGoing)
    {
    }
#endif

    /* Start to echo. */
    sendXfer.data = &g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = &g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

    while (1)
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
        	*(phrase) = *(sendXfer.data);
			switch (phrase[i]) {
			case '\r':
				UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);

				/* Wait send finished */
				while (txOnGoing) {
				}
				break;
			case '\0':
				break;
			default:
				break;
			}
			i++;

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

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_UART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_UART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}
