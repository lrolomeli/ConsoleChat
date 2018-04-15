#ifndef TERMINAL_FUNC_H_
#define TERMINAL_FUNC_H_
#include "MK64F12.h"
#include "fsl_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"

/*******************************************************************************
 * ENUMERATIONS
 ******************************************************************************/
typedef enum {

	FALSE = 0, TRUE

} boolean_type;

/*******************************************************************************
 * STRUCTURES
 ******************************************************************************/
typedef struct {

	UART_Type * xuart;
	uart_handle_t uart_handle;
	EventGroupHandle_t event_group;
	QueueHandle_t queue;
	QueueHandle_t queue2;
	QueueHandle_t foreign_queue;
	QueueHandle_t actual_queue;

} terminal_type;

/*******************************************************************************
 * DEFINITIONS
 ******************************************************************************/
#define EVENT_RX 				(1 << 0)
#define EVENT_TX 				(1 << 1)
#define PRINT_TIME				(1 << 2)
#define EXIT_TIME				(1 << 3)
#define CHAT_EVENT				(1 << 4)
#define END_CHAT_EVENT			(1 << 5)
/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/


/********************************************************************************************/
/********************************************************************************************!
 	 \brief:	print a message in the selected terminal
 	 \param[in] 	UART BASE
 	 \param[in]  	Callback handle BT/SERIAL
 	 \param[in]  	EventGroup BT/SERIAL
 	 \param[in]   	Array with message
  	 \param[in]   	size message

 	 \return void
 ********************************************************************************************/
/********************************************************************************************/
void print(UART_Type * xuart, uart_handle_t* uart_handle,
		EventGroupHandle_t event_group, uint8_t printmsg[], uint16_t size);

/********************************************************************************************/
/********************************************************************************************!
 	 \brief:	Read numbers between 0 and 9 and returns the selected number
 	 \param[in] 	UART BASE
 	 \param[in]  	Callback handle BT/SERIAL
 	 \param[in]  	EventGroup BT/SERIAL

 	 \return uint8_t
 ********************************************************************************************/
/********************************************************************************************/
uint8_t select_menu(UART_Type * xuart, uart_handle_t * uart_handle,
		EventGroupHandle_t event_group);

/********************************************************************************************/
/********************************************************************************************!
 	 \brief:	Read only HEX numbers and store them into a variable of 2 bytes length
 	 \param[in] 	UART BASE
 	 \param[in]  	Callback handle BT/SERIAL
 	 \param[in]  	EventGroup BT/SERIAL

 	 \return uint16_t
 ********************************************************************************************/
/********************************************************************************************/
int16_t read_eeprom_subaddress(UART_Type * xuart, uart_handle_t * uart_handle,
		EventGroupHandle_t event_group);

/********************************************************************************************/
/********************************************************************************************!
 	 \brief:	Reads and saves a message from keyboard into a buffer
 	 \param[in] 	UART BASE
 	 \param[in]  	Callback handle BT/SERIAL
 	 \param[in]  	EventGroup BT/SERIAL

 	 \return uart_transfer_t
 ********************************************************************************************/
/********************************************************************************************/
uart_transfer_t read_from_keyboard(UART_Type * xuart,
		uart_handle_t * uart_handle, EventGroupHandle_t event_group);

/********************************************************************************************/
/********************************************************************************************!
 	 \brief:	Read from keyboard how many bytes you need to read
 	 \param[in] 	UART BASE
 	 \param[in]  	Callback handle BT/SERIAL
 	 \param[in]  	EventGroup BT/SERIAL

 	 \return uint16_t
 ********************************************************************************************/
/********************************************************************************************/
uint16_t bytes_to_read(UART_Type * xuart, uart_handle_t * uart_handle,
		EventGroupHandle_t event_group);

/********************************************************************************************/
/********************************************************************************************!
 	 \brief:	read keyboard until enter is pressed
 	 \param[in] 	UART BASE
 	 \param[in]  	Callback handle BT/SERIAL
 	 \param[in]  	EventGroup BT/SERIAL
 	 \param[in]   	Array with message
  	 \param[in]   	size message

 	 \return void
 ********************************************************************************************/
/********************************************************************************************/
void stay_til_enter(UART_Type * xuart, uart_handle_t * uart_handle,
		EventGroupHandle_t event_group);

/********************************************************************************************/
/********************************************************************************************!
 	 \brief:	Sends uart received buffer to print it on lcd
 	 \param[in] 	UART BASE
 	 \param[in]  	Callback handle BT/SERIAL
 	 \param[in]  	EventGroup BT/SERIAL

 	 \return void
 ********************************************************************************************/
/********************************************************************************************/
void LCD_echo(UART_Type * xuart, uart_handle_t * uart_handle,
		EventGroupHandle_t event_group);

#endif /* TERMINAL_FUNC_H_ */
