/*
 * serialterminal.c
 *
 *  Created on: Mar 28, 2018
 *      Author: lrolo
 */
#include "serialterminal.h"
#include "terminal.h"
#include "board.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "menu.h"

static EventGroupHandle_t serialterm_events_g;

/*******************************************************************************
 * CALLBACK
 ******************************************************************************/
void UART_UserCallback_ter(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_UART_TxIdle == status)
    {
    	//event group set bits from isr for tx
    	xEventGroupSetBitsFromISR(serialterm_events_g, EVENT_TX, &xHigherPriorityTaskWoken);
    }

    if (kStatus_UART_RxIdle == status)
    {
    	//event group set bits from isr for rx
    	xEventGroupSetBitsFromISR(serialterm_events_g, EVENT_RX, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void serial_terminal_init(void)
{
	static uart_config_t config_ter;
	static terminal_type serialterm;

	serialterm.xuart = UART0;
	/*******************************************************************************
	 * GPIO UART CONFIGURATION
	 ******************************************************************************/
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_Uart0);

	PORT_SetPinMux(PORTB, 10, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTB, 11, kPORT_MuxAlt3);

	/*******************************************************************************
	 * UART TERMINAL CONFIGURATION
	 ******************************************************************************/

    UART_GetDefaultConfig(&config_ter);
    config_ter.enableTx = true;
    config_ter.enableRx = true;

	/*******************************************************************************
	 * TERMINAL INITIALIZATION AND HANDLER CREATION
	 ******************************************************************************/

    UART_Init(UART0, &config_ter, CLOCK_GetFreq(UART0_CLK_SRC));
	UART_TransferCreateHandle(UART0, &serialterm.uart_handle, UART_UserCallback_ter,
			NULL);

	/*******************************************************************************
	 * INTERRUPT HABILITATION
	 ******************************************************************************/

    NVIC_EnableIRQ(UART0_RX_TX_IRQn);
    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);

    serialterm_events_g = xEventGroupCreate();

    serialterm.event_group = serialterm_events_g;

	xTaskCreate(main_menu_task, "terminal_select_menu",
			configMINIMAL_STACK_SIZE, (void *) &serialterm,
			configMAX_PRIORITIES-4, NULL);



}

EventGroupHandle_t get_serialterm_event(void)
{

	return serialterm_events_g;

}

