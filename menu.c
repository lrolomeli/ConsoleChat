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

//uint8_t validation_menu(uint8_t * variable, uint8_t length);
//uint8_t validation_address(uint8_t * variable, uint8_t length);
void in_charge(uint8_t menu);

static EventGroupHandle_t menu_events_g;



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
	static uint8_t menu;

	for(;;)
	{
		/*******************************************************************************
		 * SUSPEND UNTIL ITS CALLED WHILE WE'RE ON IN CHARGE FUNCTION
		 ******************************************************************************/
		//xEventGroupWaitBits(menu_events_g, MAIN_MENU_EVENT, pdTRUE, pdTRUE, portMAX_DELAY);

		/*******************************************************************************
		 * DEPLOY MENU
		 ******************************************************************************/
		send_menu_task(uart_param->xuart, &(uart_param->uart_handle),uart_param->event_group);
		/*******************************************************************************
		 * READ MENU FROM KEYBOARD
		 ******************************************************************************/
		while (0 == menu)
		{
			menu = read_menu_from_keyboard(uart_param->xuart,
					&(uart_param->uart_handle), uart_param->event_group);
		}
		/*******************************************************************************
		 * CHECK AND VALID THE RECEIVED VALUE && SEND MENU TO IN CHARGE FUNCTION
		 ******************************************************************************/
		in_charge(menu);
		menu=0;
		vTaskDelay(portMAX_DELAY);

	}

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
void menu_task1(void * pvParameters)
{

//	uint8_t * address;

//	xEventGroupWaitBits(get_menu_event(), 1 << 1, pdTRUE, pdTRUE,
//			portMAX_DELAY);
	//printf ("dame la direccion en la que quieres escribir")

	//queue receive blocking
//	 xQueueReceive( xQueue, &address, portMAX_DELAY );
	//guarda la direccion en un buffer
	//printf ("escribe lo que quieras en memoria")
	//guarda el mensaje en el buffer
	//envialo a i2c write memory function

	//vamos a mandarle un buffer para que lo imprima la terminal que estamos usando
	//el que mando llamar esta tarea nos debe proveer informacion como
	//quien fue el que la mando ejecutar, cpu o bluetooth
	//para saber en donde vamos a imprimir lo que se requiere
	//para saber que tipo de validacion vamos a hacer es necesario conocer en que menu nos encontramos
	//por lo que debemos encender una bandera del menu o de alguna forma informarle a la tarea validacion
	//en donde estamos

	//como le voy a decir en que menu estoy de modo que sea mas facil comparar

	 for (;;)
	{
		/*******************************************************************************
		 * SUSPEND UNTIL ITS CALLED WHILE WE'RE ON IN CHARGE FUNCTION
		 ******************************************************************************/
		xEventGroupWaitBits(menu_events_g, MENU_1_EVENT, pdTRUE, pdTRUE,
		        portMAX_DELAY);

		xEventGroupSetBits(get_i2c_event(), MEMORY_READ_EVENT);

		xEventGroupWaitBits(get_i2c_event(), MEMORY_READ_DONE, pdTRUE, pdTRUE,
				        portMAX_DELAY);

	}

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
void menu_task2(void * pvParameters)
{


	 for (;;)
	{
		/*******************************************************************************
		 * SUSPEND UNTIL ITS CALLED WHILE WE'RE ON IN CHARGE FUNCTION
		 ******************************************************************************/
		xEventGroupWaitBits(menu_events_g, MENU_2_EVENT, pdTRUE, pdTRUE,
		portMAX_DELAY);
	}


}


void start_task(void * pvParameters)
{

//	xTaskCreate(main_menu_task, "terminal_select_menu",
//			configMINIMAL_STACK_SIZE, (void *) 0,
//			configMAX_PRIORITIES-4, NULL);
//
//	xTaskCreate(main_menu_task, "bluetooth_select_menu",
//			configMINIMAL_STACK_SIZE, (void *) 0,
//			configMAX_PRIORITIES-4, NULL);

//	xTaskCreate(menu_task1, "read_eeprom",
//			configMINIMAL_STACK_SIZE, NULL,
//			configMAX_PRIORITIES-4, NULL);
//
//	xTaskCreate(menu_task2, "write_eeprom",
//			configMINIMAL_STACK_SIZE, NULL,
//			configMAX_PRIORITIES-4, NULL);

	/*******************************************************************************
	 * EVENTS CREATION
	 ******************************************************************************/
	menu_events_g = xEventGroupCreate();
	/*******************************************************************************
	 * GO TO MAIN MENU TASK
	 ******************************************************************************/
	in_charge(MAIN_MENU);

	vTaskDelete(NULL);

}

void init_menu(void)
{

	xTaskCreate(start_task, "create_menus_task",
			configMINIMAL_STACK_SIZE, NULL,
			configMAX_PRIORITIES, NULL);

}

/*******************************************************************************
 * IN CHARGE
 *
 * Cual menu sigue despues
 ******************************************************************************/
void in_charge(uint8_t menu)
{

	xEventGroupSetBits(menu_events_g, 1 << menu);

}

//uint8_t validation_menu(uint8_t * variable, uint8_t length)
//{
//
//	if (1 == length)
//	{
//		if ('0' < variable[0] && '9' >= variable[0])
//		{
//			return variable[0];
//		}
//	}
//	return FALSE;
//
//}
//
//uint8_t validation_address(uint8_t * variable, uint8_t length)
//{
//	static uint8_t index;
//
//	if (5 > length)
//	{
//		for (index = 0; index < length; index++)
//		{
//			variable[index] = variable[index] - '0';
//		}
//		return TRUE;
//	}
//	return FALSE;
//
//}

