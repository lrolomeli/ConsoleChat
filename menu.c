/*
 * menu.c
 *
 *  Created on: Mar 26, 2018
 *      Author: lrolo
 */
#include "terminal.h"
#include "menu.h"
#include "time_memory_func.h"

#define MAIN_MENU 0
#define MAIN_MENU_EVENT (1 << 0)
#define MENU_1_EVENT (1 << 1)
#define MENU_2_EVENT (1 << 2)
#define MENU_SIZE (sizeof(terminal_menu) - 1)
#define MSG1MENU1 (sizeof(msg1_menu1) - 1)
#define MSG2MENU1 (sizeof(msg2_menu1) - 1)
#define MSG3MENU1 (sizeof(msg3_menu1) - 1)

typedef struct state {

	uint8_t (*ptr)(UART_Type *, uart_handle_t *, EventGroupHandle_t); /**pointer to function*/

} State;

//uint8_t validation_menu(uint8_t * variable, uint8_t length);
//uint8_t validation_address(uint8_t * variable, uint8_t length);
uint8_t menu_one(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group);
uint8_t menu_two(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group);
uint8_t main_menu(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group);

static const State menu_state[10] = { {&main_menu},
		{&menu_one},
		{&menu_two},
		{&menu_one},
		{&menu_one},
		{&menu_one},
		{&menu_one},
		{&menu_one},
		{&menu_one},
		{&menu_one} };

static uint8_t terminal_menu[] =
				"\r\n(1) Leer Memoria I2C\r\n(2) Escribir Memoria I2C"
		        "\r\n(3) Establecer Hora\r\n(4) Establecer Fecha"
		        "\r\n(5) Formato de Hora\r\n(6) Leer Hora"
		        "\r\n(7) Leer Fecha\r\n(8) Comunicacion con terminal 2"
		        "\r\n(9) Eco en LCD\r\n";

static uint8_t msg1_menu1[] =
				"\r\nSolo valores en hexadecimal entre 0 y 7FFF\r\nDireccion de Lectura:\r\n0x";

static uint8_t msg2_menu1[] =
				"\r\nTexto a guardar:\r\n";

static uint8_t msg3_menu1[] =
				"\r\nLongitud en bytes:\r\n";

/*******************************************************************************
 * RUTINA MENU PRINCIPAL
 *
 *	Esta rutina va a imprimir el menu principal en pantalla
 *	y espera respuesta por parte del usuario
 *
 ******************************************************************************/
void main_menu_task(void * pvParameters)
{

	terminal_type * uart_param = (terminal_type *) pvParameters;
	static uint8_t menu = 0;

	for(;;)
	{

		menu = menu_state[menu].ptr(uart_param->xuart, &(uart_param->uart_handle),uart_param->event_group);

	}

}

uint8_t main_menu(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group)
{

	uint8_t menu = 0;

	/*******************************************************************************
	 * DEPLOY MENU
	 ******************************************************************************/
	print(xuart, (uart_handle),event_group, terminal_menu, MENU_SIZE);

	/*******************************************************************************
	 * READ MENU FROM KEYBOARD
	 ******************************************************************************/
	while (0 == menu)
	{
		menu = read_from_keyboard(xuart,
						(uart_handle), event_group);
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
uint8_t menu_one(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group)
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
uint8_t menu_two(UART_Type * xuart, uart_handle_t* uart_handle, EventGroupHandle_t event_group)
{
	int16_t subaddress = -1;
	uart_transfer_t text_to_send = {NULL, 0};


	print(xuart, (uart_handle), event_group, msg1_menu1, MSG1MENU1);

	subaddress = read_from_keyboard1(xuart, (uart_handle), event_group);

	print(xuart, (uart_handle), event_group, msg2_menu1, MSG2MENU1);

	text_to_send = read_from_keyboard2(xuart, (uart_handle), event_group);

	write_mem(subaddress, text_to_send.data, (text_to_send.dataSize*(sizeof(uint8_t))));

	vPortFree(text_to_send.data);

	return MAIN_MENU;

}
