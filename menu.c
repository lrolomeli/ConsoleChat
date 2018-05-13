/*
 * menu.c
 *
 *  Created on: Mar 26, 2018
 *      Author: lrolo
 */
#include <terminal_func.h>
#include "menu.h"
#include "time_memory_func.h"
#include "lcd_func.h"
#include "FreeRTOS.h"
#include "task.h"

/******************************************************************************
* DEFINITIONS
******************************************************************************/
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

/*******************************************************************************
 * STRUCTURES
 ******************************************************************************/
typedef struct state {
	 /**pointer to function*/
	uint8_t (*ptr)(UART_Type *, uart_handle_t *, EventGroupHandle_t,
			QueueHandle_t, QueueHandle_t, QueueHandle_t);

} State;

/*******************************************************************************
* PROTOTYPES
******************************************************************************/
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
uint8_t menu_nine(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t main_menu(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue);
uint8_t * check_hour(uart_transfer_t hour);
uint8_t * check_date(uart_transfer_t date);

/******************************************************************************
* state machine of menu
******************************************************************************/
static const State menu_state[10] = { 	{ &main_menu },
										{ &menu_one },
										{ &menu_two },
										{ &menu_three },
										{ &menu_one },
										{ &menu_one },
										{ &menu_six },
										{ &menu_one },
										{ &menu_eight },
										{ &menu_nine } };

/******************************************************************************
* text of UART of (serial port / bluetooth)
******************************************************************************/
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

static uint8_t msg1_menu4[] = "\r\nType Date in dd/mm/yy: ";

static uint8_t msg2_menu4[] = "\r\nDate has been modified... ";

static uint8_t disconnect_msg[] = "\r\nThe other person has left the conversation\r\n";

static uint8_t connection_msg[] = "\r\nYou have enter the room\r\n";

static uint8_t enter[] = "\r\nTerminal says: ";

static uint8_t you[] = "\r\n";


/******************************************************************************
* It takes the hour from a buffer and then separate the values to
* send it to the LCD.
******************************************************************************/
void print_time_lcd_task(void * pvParameters)
{
	static uint8_t buffer[3] = {0};
	static uint8_t time[10];
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

/******************************************************************************
* It takes the hour from a buffer and then separate the values to
* send it to the terminal.
******************************************************************************/
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

/******************************************************************************
* It takes the date from a buffer and then separate the values to
* send it to the LCD.
******************************************************************************/
void print_date_lcd_task(void * pvParameters)
{
	static uint8_t buffer[2] = {0};
	static uint8_t date[10];
	static uint16_t year;
	static uint8_t bit1_year = 0;
	static uint8_t bit2_year = 0;


	for(;;)
	{

		xQueueReceive(get_date_mailbox(), buffer, portMAX_DELAY);

		year = buffer[2] * 1000 + buffer[3] * 100 + buffer[4] * 10 + buffer[5];
		bit1_year = buffer[0] & MASK_BIT1_YEAR;
		bit2_year = buffer[0] & MASK_BIT2_YEAR;

		year = ((year) + (bit1_year >> 7) + (bit2_year >> 7));

		date[9] =  ((int32_t)((year % 10000) / 1000)) + '0';;
		date[8] =  ((int32_t)((year % 1000) / 100)) + '0';
		date[7] =  ((int32_t)((year % 100) / 10)) + '0';
		date[6] =  ((int32_t)((year % 10) / 1)) + '0';
		date[5] = '/';
		date[4] = (buffer[0] & MASK_FORMAT_U_MONTH) + '0';
		date[3] = ((buffer[0] & MASK_FORMAT_D_MONTH) >> 4) + '0';
		date[2] = '/';
		date[1] = (buffer[1] & MASK_FORMAT_U_DAY) + '0';
		date[0] = ((buffer[1] & MASK_FORMAT_D_DAY) >> 4) + '0';

		printline(Normal_print, date, fifth_row);

	}

}

/******************************************************************************
* It takes the date from a buffer and then separate the values to
* send it to terminal.
******************************************************************************/
void print_date_task(void * pvParameters)
{

	terminal_type * uart_param = (terminal_type *) pvParameters;
	uint8_t msg = 0;
	static uint8_t buffer[6] = { 0 };
	static uint8_t date[11];
	static uint16_t year;
	static uint8_t bit1_year = 0;
	static uint8_t bit2_year = 0;

	xEventGroupWaitBits(uart_param->event_group, EXIT_DATE, pdTRUE, pdTRUE,
			portMAX_DELAY);

		for (;;)
		{

				xQueueReceive(uart_param->queue, &msg, 0);

				if (msg)
					{
						xEventGroupWaitBits(uart_param->event_group, EXIT_DATE, pdTRUE,
								pdTRUE, portMAX_DELAY);
						msg = 0;
					}

				xQueueReceive(get_date_mailbox(), buffer, portMAX_DELAY);
				year = buffer[2] * 1000 + buffer[3] * 100 + buffer[4] * 10 + buffer[5];
				bit1_year = buffer[0] & MASK_BIT1_YEAR;
				bit2_year = buffer[0] & MASK_BIT2_YEAR;


				year = ((year) + (bit1_year >> 7) + (bit2_year >> 7));


				date[10] = '\r';
				date[9] =  ((int32_t)((year % 10000) / 1000)) + '0';;
				date[8] =  ((int32_t)((year % 1000) / 100)) + '0';
				date[7] =  ((int32_t)((year % 100) / 10)) + '0';
				date[6] =  ((int32_t)((year % 10) / 1)) + '0';
				date[5] = '/';
				date[4] = (buffer[0] & MASK_FORMAT_U_MONTH) + '0';
				date[3] = ((buffer[0] & MASK_FORMAT_D_MONTH) >> 4) + '0';
				date[2] = '/';
				date[1] = (buffer[1] & MASK_FORMAT_U_DAY) + '0';
				date[0] = ((buffer[1] & MASK_FORMAT_D_DAY) >> 4) + '0';

				print(uart_param->xuart, &(uart_param->uart_handle),
				uart_param->event_group, date, 9 * sizeof(uint8_t));

	}

}

/******************************************************************************
* This function
******************************************************************************/
void communication_task(void * pvParameters)
{
	uart_transfer_t received = { NULL, 0 };
	terminal_type * uart_param = (terminal_type *) pvParameters;

	for (;;)
	{
		//xSemaphoreTake(chat_semaphore,portMAX_DELAY);
		xEventGroupWaitBits(uart_param->event_group, CHAT_EVENT, pdTRUE, pdTRUE, portMAX_DELAY);
		//xQueueReset(uart_param->actual_queue);
		do
		{

			xQueueReceive(uart_param->actual_queue, &received, portMAX_DELAY);
			print(uart_param->xuart, &(uart_param->uart_handle),
					uart_param->event_group, enter, ENTERSIZE);
			print(uart_param->xuart, &(uart_param->uart_handle),
					uart_param->event_group, received.data, received.dataSize);
			print(uart_param->xuart, &(uart_param->uart_handle),
					uart_param->event_group, you, YOUSIZE);

		//} while (xSemaphoreTake(end_chat_sempahore,0) == pdTRUE);
		} while (pdFALSE == xEventGroupWaitBits(uart_param->event_group, END_CHAT_EVENT, pdTRUE, pdTRUE, 0));
	}

}

/*******************************************************************************
 * 	MAIN MENU TASK
 *
 *	Main task that let us change the state between menus into terminals
 *
 ******************************************************************************/
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
 * 	ROUTINE MAIN MENU
 *
 *	Function that receives a number from the user in terminals
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
	print(xuart, uart_handle, event_group, terminal_menu, MENU_SIZE);

	/*******************************************************************************
	 * READ MENU FROM KEYBOARD
	 ******************************************************************************/
	while (0 == menu)
	{
		menu = select_menu(xuart, uart_handle, event_group);
	}

	return menu;
}

/*******************************************************************************
 * ROUTINE MENU 1 WRITE 12C MEMORY DESCRIPTION
 *
 *	This routine works as an interface for whoever wants to read I2C memory
 *	these function print messages on terminals and interact with user waiting
 *	him to put the address and buffer length to read
 *
 ******************************************************************************/
uint8_t menu_one(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	int16_t subaddress = -1;
	uint16_t byte_length = 0;
	uint8_t * reserved;


	print(xuart, uart_handle, event_group, msg1_menu1, MSG1MENU1);

	subaddress = read_eeprom_subaddress(xuart, uart_handle, event_group);

	print(xuart, uart_handle, event_group, msg3_menu1, MSG3MENU1);

	byte_length = bytes_to_read(xuart, uart_handle, event_group);

	reserved = read_mem(subaddress, byte_length);

	print(xuart, uart_handle, event_group, reserved, byte_length);

	vPortFree(reserved);

	return MAIN_MENU;

}

/*******************************************************************************
 * ROUTINE MENU 2 READ I2C MEMORY DESCRIPTION
 *
 *	This function uses UART print messages function to interact with the user
 *	furthermore this is going to wait for the user to capture a valid memory
 *	address.
 *	The same way it waits for the user to capture what he want to store in I2C
 *	EEPROM memory.
 *	That buffer written by the user is sent via write_mem function and then the
 *	buffer is cleared.
 *	this function returns to Main Menu when finished.
 *
 ******************************************************************************/
uint8_t menu_two(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	int16_t subaddress = -1;
	uart_transfer_t text_to_send = {NULL, 0};


	print(xuart, uart_handle, event_group, msg1_menu1, MSG1MENU1);

	subaddress = read_eeprom_subaddress(xuart, uart_handle, event_group);

	print(xuart, uart_handle, event_group, msg2_menu1, MSG2MENU1);

	text_to_send = read_from_keyboard(xuart, uart_handle, event_group);

	write_mem(subaddress, text_to_send.data,
			(text_to_send.dataSize * (sizeof(uint8_t))));

	vPortFree(text_to_send.data);

	return MAIN_MENU;

}

/*******************************************************************************
 * ROUTINE MENU 3 MODIFY HOUR DESCRIPTION
 *
 *	This routine ask for the user to enter a valid time and hour format 23:59:59
 *	when the user hits enter all the buffer which was introduced by the user is
 *	checked. Once it has a valid time it's stored respectively to RTC clock
 *
 ******************************************************************************/
uint8_t menu_three(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	uart_transfer_t hour = {NULL, 0};
	uint8_t * time;

	print(xuart, uart_handle, event_group, msg1_menu3, MSG1MENU3);

	do
	{
		hour = read_from_keyboard(xuart, (uart_handle), event_group);

		time = check_hour(hour);

		if(NULL == time)
		{
			print(xuart, uart_handle, event_group, msg1_menu3, MSG1MENU3);
		}

	} while (NULL == time);

	write_time(time);

	print(xuart, uart_handle, event_group, msg2_menu3, MSG2MENU3);


	return MAIN_MENU;

}

/*******************************************************************************
 * ROUTINE MENU 4 MODIFY DATE DESCRIPTION
 *
 *	This routine do the same as the modify hour function but uses another
 *	validate routine to check date.
 *
 ******************************************************************************/
uint8_t menu_four(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{

	return MAIN_MENU;

}

/*******************************************************************************
 * ROUTINE MENU 6
 *
 *	This function is constantly waiting while the print time task is checking
 *	and printing the hour. Once the user press enter this function wakes up for
 *	taking control of the core
 *
 ******************************************************************************/
uint8_t menu_six(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	static uint8_t buffer = 1;

	print(xuart, uart_handle, event_group, msg1_menu6, MSG1MENU6);

	xEventGroupSetBits(event_group, EXIT_TIME);

	stay_til_enter(xuart, uart_handle, event_group);

	xQueueSendToBack(queue, &buffer, portMAX_DELAY);

	return MAIN_MENU;

}

uint8_t menu_eight(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{
	static uart_transfer_t disconnect = { disconnect_msg, MSGSIZEDISC };
	uart_transfer_t sent = { NULL, 0 };
	print(xuart, uart_handle, event_group, connection_msg, MSGSIZECON);
	//xSemaphoreGive(chat_semaphore);
	xEventGroupSetBits(event_group, CHAT_EVENT);
	do
	{

		sent = read_from_keyboard(xuart, uart_handle, event_group);

		if (sent.data != NULL)
		{
			print(xuart, uart_handle,
					event_group, you, YOUSIZE);
			xQueueSendToBack(foreign_queue, &sent, 0);
		}

	} while (NULL != sent.data);

	xQueueSendToBack(foreign_queue, &disconnect, 0);

	//xSemaphoreGive(end_chat_semaphore);
	xEventGroupSetBits(event_group, END_CHAT_EVENT);

	return MAIN_MENU;

}

uint8_t menu_nine(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, QueueHandle_t queue,
		QueueHandle_t foreign_queue, QueueHandle_t actual_queue)
{

	LCD_echo(xuart, uart_handle, event_group);

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

uint8_t * check_date(uart_transfer_t year)
{
	int8_t date[8] = {0};
	static uint8_t date_to_send[6] = {0};
	date_to_send[0] = -1;
	date_to_send[1] = -1;
	date_to_send[2] = -1;
	date_to_send[3] = -1;
	date_to_send[4] = -1;
	date_to_send[5] = -1;


	if(10 == year.dataSize)
	{
		//////////////////////////////////////////////valdar datos de año ///////////////////////////////////////////////////////////////////////////
		date[UNITS_T_YEAR] = ( '0' <= year.data[0] && '9' >= year.data[0] ) ? (year.data[0] - '0') : -1;
		date[HUNDRED_YEAR] = ( '0' <= year.data[1] && '9' >= year.data[1] ) ? (year.data[1] - '0') : -1;
		date[TENS_YEAR] =    ( '0' <= year.data[2] && '9' >= year.data[2] ) ? (year.data[2] - '0') : -1;
		date[UNITS_YEAR] =   ( '0' <= year.data[3] && '9' >= year.data[3] ) ? (year.data[3] - '0') : -1;

		//////////////////////////////////////////////valdar decenas de mes ///////////////////////////////////////////////////////////////////////////
		date[TENS_MONTH] =   ( '0' <= year.data[5] && '1' >= year.data[5] ) ? (year.data[5] - '0') : -1;

		//////////////////////////////////////////////vaidar unidaades de mes ///////////////////////////////////////////////////////////////////////////
		if (date[TENS_MONTH] == 0)
		{
			date[UNITS_MONTH] = ( '0' <= year.data[6] && '9' >= year.data[6] ) ? (year.data[6] - '0') : -1;
		}
		else if (date[TENS_MONTH] == 1)
		{
			date[UNITS_MONTH] = ( '0' <= year.data[6] && '2' >= year.data[6] ) ? (year.data[6] - '0') : -1;
		}

		///////////////////////////////////////////////validar decena maxima de dias /////////////////////////////////////////////////////////////////////////////////////////////////
		date[TENS_DAY] =   ( '0' <= year.data[8] && '3' >= year.data[8] ) ? (year.data[8] - '0') : -1;


		///////////////////////////////////////////////validar dias en febrero /////////////////////////////////////////////////////////////////////////////////////////////////
		if (date[TENS_MONTH] == 0 && date[UNITS_MONTH] == 2 && (date[TENS_DAY] == 0 || date[TENS_DAY] == 1))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '9' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}
		if (date[TENS_MONTH] == 0 && date[UNITS_MONTH] == 2 && (date[TENS_DAY] == 2))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '8' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}

		///////////////////////////////////////////////validar los meses que tienen 30 dias ////////////////////////////////////////////////////////////////////////////////////////
		if (date[TENS_MONTH] == 0 && date[UNITS_MONTH] == 4  && date[UNITS_MONTH] == 6 && date[UNITS_MONTH] == 9 && (date[TENS_DAY] == 0 || date[TENS_DAY] == 1 || date[TENS_DAY] == 2))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '9' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}
		if (date[TENS_MONTH] == 1 && date[UNITS_MONTH] == 1  && (date[TENS_DAY] == 0 || date[TENS_DAY] == 1 || date[TENS_DAY] == 2))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '9' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}

		if (date[TENS_MONTH] == 0 && date[UNITS_MONTH] == 4  && date[UNITS_MONTH] == 6 && date[UNITS_MONTH] == 9 && (date[TENS_DAY] == 3) )
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '0' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}
		if (date[TENS_MONTH] == 1 && date[UNITS_MONTH] == 1  && (date[TENS_DAY] == 3))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '0' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}


		///////////////////////////////////////////////validar los meses que tienen 31 dias ///////////////////////////////////////////////////////////////////////////////////////
		if (date[TENS_MONTH] == 0 && date[UNITS_MONTH] == 1 && date[UNITS_MONTH] == 3 && date[UNITS_MONTH] == 5 && date[UNITS_MONTH] == 7 && date[UNITS_MONTH] == 8 && (date[TENS_DAY] == 0 || date[TENS_DAY] == 1 || date[TENS_DAY] == 2))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '9' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}
		if (date[TENS_MONTH] == 1 && date[UNITS_MONTH] == 0 && date[UNITS_MONTH] == 2  && (date[TENS_DAY] == 0 || date[TENS_DAY] == 1 || date[TENS_DAY] == 2))
		{
			date[UNITS_DAY] = ( '0' <= year.data[5] && '9' >= year.data[5] ) ? (year.data[9] - '0') : -1;
		}

		if (date[TENS_MONTH] == 0 && date[UNITS_MONTH] == 1 && date[UNITS_MONTH] == 3 && date[UNITS_MONTH] == 5 && date[UNITS_MONTH] == 7 && date[UNITS_MONTH] == 8 && (date[TENS_DAY] == 3 ))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '1' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}
		if (date[TENS_MONTH] == 1 && date[UNITS_MONTH] == 0 && date[UNITS_MONTH] == 2  && (date[TENS_DAY] == 3 ))
		{
			date[UNITS_DAY] = ( '0' <= year.data[9] && '1' >= year.data[9] ) ? (year.data[9] - '0') : -1;
		}


		if (-1 == date[UNITS_T_YEAR] || -1 == date[HUNDRED_YEAR] || -1 == date[TENS_YEAR] || -1 == date[UNITS_YEAR]
		        || -1 == date[TENS_MONTH] || -1 == date[UNITS_MONTH] || -1 == date[TENS_DAY] || -1 == date[UNITS_DAY])
		{
			return NULL;
		}


		else
		{
			date_to_send[0] = ((0x3F) & (((date[TENS_DAY] << 4) & (MASK_FORMAT_D_DAY)) | ((date[UNITS_DAY]) & (MASK_FORMAT_U_DAY)))); //se le envia 0 a años y solo se envia bien los dias
			date_to_send[1] = (((date[TENS_MONTH] << 4) & (MASK_FORMAT_D_MONTH))) | ((date[UNITS_MONTH]) & (MASK_FORMAT_U_MONTH));
			date_to_send[2] = date[UNITS_T_YEAR];
			date_to_send[3] = date[HUNDRED_YEAR];
			date_to_send[4] = date[TENS_YEAR];
			date_to_send[5] = date[UNITS_YEAR];
			return (uint8_t *)date_to_send;
		}

	}

	return NULL;

}
