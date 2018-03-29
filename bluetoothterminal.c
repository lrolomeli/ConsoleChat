/*
 * bluetooth.c
 *
 *  Created on: Mar 28, 2018
 *      Author: lrolo
 */
#include "bluetoothterminal.h"
#include "terminal.h"
#include "board.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "menu.h"

static EventGroupHandle_t bluetoothterm_events_g;

/*******************************************************************************
 * CALLBACK
 ******************************************************************************/
void UART_UserCallback_bt(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_UART_TxIdle == status)
    {
    	//event group set bits from isr for tx
    	xEventGroupSetBitsFromISR(bluetoothterm_events_g, EVENT_TX, &xHigherPriorityTaskWoken);
    }

    if (kStatus_UART_RxIdle == status)
    {
    	//event group set bits from isr for rx
    	xEventGroupSetBitsFromISR(bluetoothterm_events_g, EVENT_RX, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void bt_terminal_init(void)
{

	static uart_config_t config_bt;
	static terminal_type bluetooth;

	bluetooth.xuart = UART1;
	/*******************************************************************************
	 * GPIO UART CONFIGURATION
	 ******************************************************************************/
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_Uart1);

	PORT_SetPinMux(PORTC, 3, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTC, 4, kPORT_MuxAlt3);

	/*******************************************************************************
	 * UART BLUETOOTH CONFIGURATION
	 ******************************************************************************/

	UART_GetDefaultConfig(&config_bt);
	config_bt.enableTx = true;
	config_bt.enableRx = true;
	config_bt.baudRate_Bps = 9600;

	/*******************************************************************************
	 * BLUETOOTH INITIALIZATION AND HANDLER CREATION
	 ******************************************************************************/

	UART_Init(UART1, &config_bt, CLOCK_GetFreq(UART1_CLK_SRC));
	UART_TransferCreateHandle(UART1, &bluetooth.uart_handle,
	        UART_UserCallback_bt,
	        NULL);

	/*******************************************************************************
	 * INTERRUPT HABILITATION
	 ******************************************************************************/

	NVIC_EnableIRQ(UART1_RX_TX_IRQn);
	NVIC_SetPriority(UART1_RX_TX_IRQn, 6);

	bluetoothterm_events_g = xEventGroupCreate();

	bluetooth.event_group = bluetoothterm_events_g;

	xTaskCreate(main_menu_task, "bluetooth_select_menu",
	configMINIMAL_STACK_SIZE, (void *) &bluetooth,
	configMAX_PRIORITIES - 4, NULL);

}

EventGroupHandle_t get_bluetoothterm_event(void)
{

	return bluetoothterm_events_g;

}

