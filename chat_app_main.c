#include "board.h"
#include "fsl_port.h"
#include "clock_config.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "terminal.h"
#include "lcd_func.h"
#include "time_memory_func.h"
#include "chat_app_main.h"

EventGroupHandle_t menu_events_g;



void terminal_menu_task(void * pvParameters)
{


	for(;;)
	{

	}
}


int main(void)
{

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

    terminal_init();
    //lcd_spi_pins_init();
    i2c_init_peripherals();
    //create task
//	xTaskCreate(terminal_menu_task, "menu_terminal", 110, (void *) 0,
//	configMAX_PRIORITIES, NULL);


    menu_events_g = xEventGroupCreate();
    vTaskStartScheduler();

    for(;;)
    {

    }

}

EventGroupHandle_t get_menu_event(void)
{

	return menu_events_g;

}
