/*
 * menu.c
 *
 *  Created on: Mar 26, 2018
 *      Author: lrolo
 */
#include "terminal.h"
#include "menu.h"
#include "time_memory_func.h"
#include "lcd_func.h"

#define MAIN_MENU 0
#define MENU_SIZE (sizeof(terminal_menu) - 1)
#define MSG1MENU1 (sizeof(msg1_menu1) - 1)
#define MSG2MENU1 (sizeof(msg2_menu1) - 1)
#define MSG3MENU1 (sizeof(msg3_menu1) - 1)
#define MSG1MENU3 (sizeof(msg1_menu3) - 1)
#define MSG2MENU3 (sizeof(msg2_menu3) - 1)
#define MSG1MENU6 (sizeof(msg1_menu6) - 1)
#define MSGSIZEDISC (sizeof(disconnect_msg) - 1)
#define MSGSIZECON (sizeof(connection_msg) - 1)
#define ENTERSIZE (sizeof(enter) - 1)
#define YOUSIZE (sizeof(you) - 1)

typedef struct state {
	 /**pointer to function*/
	uint8_t (*ptr)(UART_Type *, uart_handle_t *, EventGroupHandle_t,
			QueueHandle_t, QueueHandle_t, QueueHandle_t);

} State;

uint8_t menu_one(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t menu_two(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t menu_three(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t menu_six(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t menu_eight(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t main_menu(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t * check_hour(uart_transfer_t hour);

static const State menu_state[10] = { 	{ &main_menu },
										{ &menu_one },
										{ &menu_two },
										{ &menu_three },
										{ &menu_one },
										{ &menu_one },
										{ &menu_six },
										{ &menu_one },
										{ &menu_eight },
										{ &menu_one } };

static uint8_t terminal_menu[] =
		"\r\n(1) Read EEPROM I2C\r\n(2) Write EEPROM I2C"
				"\r\n(3) Set Hour\r\n(4) Set Date"
				"\r\n(5) Hour Format\r\n(6) Show Current Hour"
				"\r\n(7) Show Date\r\n(8) Terminal Communication"
				"\r\n(9) LCD echo\r\n";

static uint8_t msg1_menu1[] =
		"\r\nOnly HEX values beetween 0 and 7FFF\r\nEEPROM memory address:\r\n0x";

static uint8_t msg2_menu1[] = "\r\nType text to Store:\r\n";

static uint8_t msg3_menu1[] = "\r\nBytes length (MAX 99):\r\n";

static uint8_t msg1_menu3[] = "\r\nType hour in hh/mm/ss: ";

static uint8_t msg2_menu3[] = "\r\nHour has been modified... ";

static uint8_t msg1_menu6[] = "\r\nCurrent Hour...\r\n";

static uint8_t disconnect_msg[] = "\r\nThe other person has left the conversation\r\n";

static uint8_t connection_msg[] = "\r\nYou have enter the room\r\n";

static uint8_t enter[] = "\r\nTerminal says: ";

static uint8_t you[] = "\r\n";

void print_time_lcd_task(void * pvParameters)
{
	uint8_t buffer[3] = {0};
	uint8_t time[10];
	static uint8_t welcome_msg1[] = "Bluetooth Chat";

	for(;;)
	{
		printline(Inverse_print, welcome_msg1, first_row);
		xQueueReceive(get_time_mailbox(), buffer, portMAX_DELAY);

		time[9] = 0;
		time[8] = ' ';
		time[7] = (buffer[0] & 0x0F) + '0';
		time[6] = ((buffer[0] & 0xF0) >> 4) + '0';
		time[5] = ':';
		time[4] = (buffer[1] & 0x0F) + '0';
		time[3] = ((buffer[1] & 0xF0) >> 4) + '0';
		time[2] = ':';
		time[1] = (buffer[2] & 0x0F) + '0';
		time[0] = ((buffer[2] & 0x30) >> 4) + '0';

		LCDNokia_clear();
		printline(Normal_print, time, sixth_row);

	}

}
void print_time_task(void * pvParameters)
{

	terminal_type * uart_param = (terminal_type *) pvParameters;
	static uint8_t msg = 0;
	static uint8_t buffer[3] = {0};
	static uint8_t time[9] = {0};

	xEventGroupWaitBits(uart_param->event_group, EXIT_TIME, pdTRUE, pdTRUE,
			portMAX_DELAY);

	for(;;)
	{
		xQueueReceive(uart_param->queue2, buffer, portMAX_DELAY);
		time[8] = '\r';
		time[7] = (buffer[0] & 0x0F) + '0';
		time[6] = ((buffer[0] & 0xF0) >> 4) + '0';
		time[5] = ':';
		time[4] = (buffer[1] & 0x0F) + '0';
		time[3] = ((buffer[1] & 0xF0) >> 4) + '0';
		time[2] = ':';
		time[1] = (buffer[2] & 0x0F) + '0';
		time[0] = ((buffer[2] & 0x30) >> 4) + '0';
		xQueueReceive(uart_param->queue, &msg, 0);
		if(msg)
		{
			msg = 0;
			xEventGroupWaitBits(uart_param->event_group, EXIT_TIME, pdTRUE,
					pdTRUE, portMAX_DELAY);
		}
		else
		{
			print(uart_param->xuart, &(uart_param->uart_handle),
					uart_param->event_group, time, 9 * sizeof(uint8_t));
		}
	}
}

void communication_task(void * pvParameters)
{
	uart_transfer_t received = { NULL, 0 };
	terminal_type * uart_param = (terminal_type *) pvParameters;

	for (;;)
	{
		xSemaphoreTake(chat_smaphore,portMAX_DELAY);
		xQueueReset(uart_param->actual_queue);
		do
		{

		xQueueReceive(uart_param->actual_queue, &received, portMAX_DELAY);
		print(uart_param->xuart, &(uart_param->uart_handle),
				uart_param->event_group, enter, ENTERSIZE);
		print(uart_param->xuart, &(uart_param->uart_handle),
				uart_param->event_group, received.data, received.dataSize);
		print(uart_param->xuart, &(uart_param->uart_handle),
				uart_param->event_group, you, YOUSIZE);
		}while(xSemaphoreTake(end_char_sempahore,0) == pdTRUE);
	}

}

void main_menu_task(void * pvParameters)
{

	terminal_type * uart_param = (terminal_type *) pvParameters;
	static uint8_t menu = 0;

	xTaskCreate(print_time_task, "print_time_task_terminal",
	configMINIMAL_STACK_SIZE, pvParameters,
	configMAX_PRIORITIES - 1, NULL);

	xTaskCreate(communication_task, "communication_terminal_print_task",
	configMINIMAL_STACK_SIZE, pvParameters,
	configMAX_PRIORITIES - 1, NULL);


	for(;;)
	{

		menu = menu_state[menu].ptr(uart_param->xuart,
		        &(uart_param->uart_handle), uart_param->event_group,
		        uart_param->queue, uart_param->foreign_queue,
		        uart_param->actual_queue);

	}

}
/*******************************************************************************
 * RUTINA MENU PRINCIPAL
 *
 *	Esta rutina va a imprimir el menu principal en pantalla
 *	y espera respuesta por parte del usuario
 *
 ******************************************************************************/
uint8_t main_menu(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{

	uint8_t menu = 0;

	/*******************************************************************************
	 * DEPLOY MENU
	 ******************************************************************************/
	print(xuart, (uart_handle), event_group, terminal_menu, MENU_SIZE);

	/*******************************************************************************
	 * READ MENU FROM KEYBOARD
	 ******************************************************************************/
	while (0 == menu)
	{
		menu = read_from_keyboard(xuart, (uart_handle), event_group);
	}

	return menu;
}

/*******************************************************************************
 * RUTINA MENU 1
 *
 *	Esta rutina desplegara un mensaje que pedira la direccion que queremos leer
 *	despues espera respuesta por parte del usuario, hasta que se presiona enter
 *	se valida lo que el usuario introdujo
 *	se despliega otro mensaje en el cual se pedira la longitud en bytes que queremos leer
 *
 ******************************************************************************/
uint8_t menu_one(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	int16_t subaddress = -1;
	uint16_t byte_length = 0;
	uint8_t * reserved;


	print(xuart, (uart_handle), event_group, msg1_menu1, MSG1MENU1);

	subaddress = read_from_keyboard1(xuart, (uart_handle), event_group);

	print(xuart, (uart_handle), event_group, msg3_menu1, MSG3MENU1);

	byte_length = read_from_keyboard3(xuart, (uart_handle), event_group);

	reserved = read_mem(subaddress, byte_length);

	print(xuart, uart_handle, event_group, reserved, byte_length);

	vPortFree(reserved);

	return MAIN_MENU;

}

/*******************************************************************************
 * RUTINA MENU 2
 *
 *	Esta rutina desplegara un mensaje que pedira la direccion que queremos escribir
 *	despues espera respuesta por parte del usuario, hasta que se presiona enter
 *	se valida lo que el usuario introdujo
 *	se despliega otro mensaje en el cual se pedira el mensaje que queremos escribir
 *
 ******************************************************************************/
uint8_t menu_two(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	int16_t subaddress = -1;
	uart_transfer_t text_to_send = {NULL, 0};


	print(xuart, (uart_handle), event_group, msg1_menu1, MSG1MENU1);

	subaddress = read_from_keyboard1(xuart, (uart_handle), event_group);

	print(xuart, (uart_handle), event_group, msg2_menu1, MSG2MENU1);

	text_to_send = read_from_keyboard2(xuart, (uart_handle), event_group);

	write_mem(subaddress, text_to_send.data,
			(text_to_send.dataSize * (sizeof(uint8_t))));

	vPortFree(text_to_send.data);

	return MAIN_MENU;

}

/*******************************************************************************
 * RUTINA MENU 3
 *
 *	Esta rutina desplegara un mensaje que pedira la hora en formato de 23:59:59
 *	despues espera respuesta por parte del usuario, hasta que se presiona enter
 *	se valida lo que el usuario introdujo y se regresa un mensaje de comprobacion
 *
 ******************************************************************************/
uint8_t menu_three(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	uart_transfer_t hour = {NULL, 0};
	uint8_t * time;

	print(xuart, (uart_handle), event_group, msg1_menu3, MSG1MENU3);

	do
	{
		hour = read_from_keyboard2(xuart, (uart_handle), event_group);

		time = check_hour(hour);

		if(NULL == time)
		{
			print(xuart, (uart_handle), event_group, msg1_menu3, MSG1MENU3);
		}

	} while (NULL == time);

	write_time(time);

	print(xuart, (uart_handle), event_group, msg2_menu3, MSG2MENU3);


	return MAIN_MENU;

}

/*******************************************************************************
 * RUTINA MENU 6
 *
 *	Esta rutina desplegara un mensaje indicando la hora actual
 *
 ******************************************************************************/
uint8_t menu_six(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	static uint8_t buffer = 1;

	print(xuart, (uart_handle), event_group, msg1_menu6, MSG1MENU6);

	xEventGroupSetBits(event_group, EXIT_TIME);

	wait_for_esc(xuart, uart_handle, event_group);

	xQueueSendToBack(queue, &buffer, portMAX_DELAY);

	return MAIN_MENU;

}

uint8_t menu_eight(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	static uart_transfer_t disconnect = { disconnect_msg, MSGSIZEDISC };
	uart_transfer_t sent = { NULL, 0 };
	print(xuart, (uart_handle), event_group, connection_msg, MSGSIZECON);
	xSemaphoreGive(chat_smaphore);
	do
	{

		sent = read_from_keyboard2(xuart, uart_handle, event_group);

		if (sent.data != NULL)
		{
			print(xuart, (uart_handle),
					event_group, you, YOUSIZE);
			xQueueSendToBack(foreign_queue, &sent, 0);
		}

	} while (NULL != sent.data);

	xQueueSendToBack(foreign_queue, &disconnect, 0);

	return MAIN_MENU;

}


uint8_t * check_hour(uart_transfer_t hour)
{
	int8_t time[6] = { 0 };
	static int8_t time_to_send[3] = { 0 };
	time_to_send[0] = -1;
	time_to_send[1] = -1;
	time_to_send[2] = -1;

	if (8 == hour.dataSize)
	{
		time[TENS_HOURS] =
				('0' <= hour.data[0] && '2' >= hour.data[0]) ?
						(hour.data[0] - '0') : -1;
		if (time[TENS_HOURS] == 2)
		{
			time[UNITS_HOURS] =
					('0' <= hour.data[1] && '3' >= hour.data[1]) ?
							(hour.data[1] - '0') : -1;
		}

		else
		{
			time[UNITS_HOURS] =
					('0' <= hour.data[1] && '9' >= hour.data[1]) ?
							(hour.data[1] - '0') : -1;
		}

		time[TENS_MINUTES] =
				('0' <= hour.data[3] && '5' >= hour.data[3]) ?
						(hour.data[3] - '0') : -1;
		time[UNITS_MINUTES] =
				('0' <= hour.data[4] && '9' >= hour.data[4]) ?
						(hour.data[4] - '0') : -1;

		time[TENS_SECONDS] =
				('0' <= hour.data[6] && '5' >= hour.data[6]) ?
						(hour.data[6] - '0') : -1;
		time[UNITS_SECONDS] =
				('0' <= hour.data[7] && '9' >= hour.data[7]) ?
						(hour.data[7] - '0') : -1;

		if (-1 == time[TENS_HOURS] || -1 == time[UNITS_HOURS]
				|| -1 == time[TENS_MINUTES] || -1 == time[UNITS_MINUTES]
				|| -1 == time[TENS_SECONDS] || -1 == time[UNITS_SECONDS])
		{
			return NULL;
		}

		else
		{
			time_to_send[2] = ((time[TENS_HOURS] << 4) & MASK_TENS)
					| (time[UNITS_HOURS] & MASK_UNITS);
			time_to_send[1] = ((time[TENS_MINUTES] << 4) & MASK_TENS)
					| (time[UNITS_MINUTES] & MASK_UNITS);
			time_to_send[0] = ((time[TENS_SECONDS] << 4) & MASK_TENS)
					| (time[UNITS_SECONDS] & MASK_UNITS);
			return (uint8_t *) time_to_send;
		}

	}

	return NULL;

}
