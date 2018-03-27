/*
 * menu.c
 *
 *  Created on: Mar 26, 2018
 *      Author: lrolo
 */

void init_menu_task(void)
{


	xTaskCreate(menu1, "read_EEPROM", configMINIMAL_STACK_SIZE,
			NULL, configMAX_PRIORITIES - 3, NULL);

	xTaskCreate(menu2, "write_EEPROM", configMINIMAL_STACK_SIZE,
			NULL, configMAX_PRIORITIES - 3, NULL);



}


void menu1(void * pvParameters)
{

	uint8_t * address;

	xEventGroupWaitBits(get_menu_event(), 1 << 1, pdTRUE, pdTRUE,
			portMAX_DELAY);
	//printf ("dame la direccion en la que quieres escribir")

	//queue receive blocking
	 xQueueReceive( xQueue, &address, portMAX_DELAY );
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



}
