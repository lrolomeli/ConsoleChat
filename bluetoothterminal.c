/*
 * bluetooth.c
 *
 *  Created on: Mar 28, 2018
 *      Author: Luis Roberto Lomeli Plascencia
 */
#include "bluetoothterminal.h"
#include "serialterminal.h"
#include "board.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "menu.h"

/*******************************************************************************
 * Global OS Variables
 ******************************************************************************/
EventGroupHandle_t bluetoothterm_events_g;
QueueHandle_t bt_term_queue;
QueueHandle_t bt_time_queue;
QueueHandle_t bt_msg_queue;

/*******************************************************************************
 * BLUETOOTH CALLBACK EITHER SEND OR RECEIVE INTERRUPT
 ******************************************************************************/
void UART_UserCallback_bt(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_UART_TxIdle == status)
    {
    	/*EventGroup set bits from ISR for TX*/
    	xEventGroupSetBitsFromISR(bluetoothterm_events_g, EVENT_TX, &xHigherPriorityTaskWoken);
    }

    if (kStatus_UART_RxIdle == status)
    {
    	/*EventGroup set bits from ISR for RX*/
    	xEventGroupSetBitsFromISR(bluetoothterm_events_g, EVENT_RX, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*******************************************************************************
 * CALL_INIT TASK
 ******************************************************************************/
void call_bt_init_task(void * pvParameters)
{
	terminal_type * uart_param = (terminal_type *) pvParameters;

	/**As foreign queue BLUETOOTH has TERMINAL queue*/
	uart_param->foreign_queue = get_serial_msg_queue();

	xTaskCreate(main_menu_task, "bluetooth_select_menu",
	configMINIMAL_STACK_SIZE, pvParameters,
	configMAX_PRIORITIES - 1, NULL);

	vTaskDelete(NULL);
}

/*******************************************************************************
 * RETURNS FINISH ATEMPT QUEUE
 ******************************************************************************/
QueueHandle_t get_bt_msg_queue(void)
{
	return bt_msg_queue;
}

/*******************************************************************************
 * RETURNS THE QUEUE WHERE TIME IS READED
 ******************************************************************************/
QueueHandle_t get_bt_time_queue(void)
{
	return bt_time_queue;
}

/*******************************************************************************
 * INITIALIZE TERMINAL REQUIREMENTS
 ******************************************************************************/
void bluetooth_init(void)
{

	static uart_config_t config_bt;
	static terminal_type bluetooth;

	bluetooth.xuart = UART1;
	/*******************************************************************************
	 * MUX UART CONFIGURATION
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

	/*******************************************************************************
	 * BLUETOOTH EVENTS
	 ******************************************************************************/
	bluetoothterm_events_g = xEventGroupCreate();
	bluetooth.event_group = bluetoothterm_events_g;

	/*******************************************************************************
	 * BLUETOOTH QUEUES
	 ******************************************************************************/
	bt_term_queue = xQueueCreate(1, sizeof(uint8_t));
	bluetooth.queue = bt_term_queue;
	bt_time_queue = xQueueCreate(1, (3*sizeof(uint8_t)));
	bluetooth.queue2 = bt_time_queue;
	bt_msg_queue = xQueueCreate(1, (sizeof(uart_transfer_t)));
	bluetooth.actual_queue = bt_msg_queue;

	/*******************************************************************************
	 * CREATE BT TASK
	 ******************************************************************************/
	xTaskCreate(call_bt_init_task, "bt_call_init_task",
	configMINIMAL_STACK_SIZE, (void *) &bluetooth,
	configMAX_PRIORITIES - 1, NULL);
}
