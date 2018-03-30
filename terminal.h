#ifndef TERMINAL_H_
#define TERMINAL_H_
#include "MK64F12.h"
#include "fsl_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"

/*******************************************************************************
 * Structures
 ******************************************************************************/
typedef enum {

	FALSE = 0, TRUE

} boolean_type;

typedef struct {

	UART_Type * xuart;
	uart_handle_t uart_handle;
	EventGroupHandle_t event_group;
	QueueHandle_t queue;

} terminal_type;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EVENT_RX 				(1 << 0)
#define EVENT_TX 				(1 << 1)
#define DEPLOY_MENU 			(1 << 2)
#define DEPLOY_MENU_SET			(1 << 3)
#define READ_FROM_KEYBOARD		(1 << 4)
#define KEEP_READING			(1 << 5)

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void print(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group, uint8_t printmsg[], uint16_t size);
uint8_t read_from_keyboard(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group);
int16_t read_from_keyboard1(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group);
uart_transfer_t read_from_keyboard2(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group);
uint16_t read_from_keyboard3(UART_Type * xuart, uart_handle_t * uart_handle, EventGroupHandle_t event_group);
uint8_t validate_number(uint8_t keyboard_data);
int8_t validatation_number(uint8_t keyboard_data);
int8_t validation_address(uint8_t keyboard_data);
uint8_t* storing_phrase(uint8_t phrase_length, uint8_t * phrase);
#endif /* TERMINAL_H_ */
