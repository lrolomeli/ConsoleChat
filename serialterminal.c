/*
 * serialterminal.c
 *
 *  Created on: Mar 28, 2018
 *      Author: lrolo
 */
#include "serialterminal.h"
#include "bluetoothterminal.h"
#include "terminal.h"
#include "board.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "menu.h"

static EventGroupHandle_t serialterm_events_g;
static QueueHandle_t serial_term_queue;
static QueueHandle_t serial_time_queue;
static QueueHandle_t serial_msg_queue;

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

void create_serial_queue(void * pvParameters)
{

	terminal_type * uart_param = (terminal_type *) pvParameters;

	uart_param->foreign_queue = get_bt_msg_queue();

	xTaskCreate(main_menu_task, "terminal_select_menu",
			configMINIMAL_STACK_SIZE, pvParameters,
			configMAX_PRIORITIES - 1, NULL);

	vTaskDelete(NULL);
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
    NVIC_SetPriority(UART0_RX_TX_IRQn, 6);

    serialterm_events_g = xEventGroupCreate();
    serialterm.event_group = serialterm_events_g;

    serial_term_queue = xQueueCreate(1, sizeof(uint8_t));
    serialterm.queue = serial_term_queue;
    serial_time_queue = xQueueCreate(1, (3*sizeof(uint8_t)));
    serialterm.queue2 = serial_time_queue;
    serial_msg_queue = xQueueCreate(1, (sizeof(uart_transfer_t)));
    serialterm.actual_queue = serial_msg_queue;

	xTaskCreate(create_serial_queue, "serial_queue_ready",
	configMINIMAL_STACK_SIZE, (void *) &serialterm,
	configMAX_PRIORITIES - 1, NULL);

}

QueueHandle_t get_serial_msg_queue(void)
{
	return serial_msg_queue;
}

QueueHandle_t get_serial_time_queue(void)
{
	return serial_time_queue;
}
