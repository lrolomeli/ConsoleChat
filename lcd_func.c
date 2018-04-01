/**
	\file 
	\brief 
		This is a starter file to use the Nokia 5510 LCD. 
		The LCD is connected as follows:
		reset-PDT0
		CE-GND
		CD-PTD3
		DIN-PTD2
		CLK-PTD1
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	1/08/2015
	\todo
	    The SPI device driver needs to be completed.
 */

#include "lcd_func.h"
#include "menu.h"

EventGroupHandle_t lcd_term_event;

EventGroupHandle_t get_lcd_term_event(void)
{
	return lcd_term_event;
}

void nokia_lcd_init_task(void * pvParameters)
{
    static uint8_t first_line[] = "Welcome to";
    static uint8_t second_line[] = "Practica 1";

    GPIO_ClearPinsOutput(GPIOD, 1 << LCD_RESET_PIN);
    vTaskDelay(100);/**delay of 100ms for properly reset*/
    GPIO_SetPinsOutput(GPIOD, 1 << LCD_RESET_PIN);

	LCDNokia_init();
//	xTaskCreate(print_time_lcd_task, "lcd_nokia_print_time", 200, NULL,
//	configMAX_PRIORITIES - 1, NULL);
	printline(Normal_print, first_line, first_row);
	printline(Inverse_print, second_line, second_row);
	vTaskDelete(NULL);

}



void lcd_spi_pins_init(void)
{

	/**Configure control pins*/
	config_lcd_spi_pins();

    xTaskCreate(nokia_lcd_init_task, "lcd_nokia_init", 200, NULL,
            configMAX_PRIORITIES, NULL);

    lcd_term_event = xEventGroupCreate();
    NVIC_EnableIRQ(SPI0_IRQn);
    NVIC_SetPriority(SPI0_IRQn, 5);
}


