#include "board.h"
#include "fsl_port.h"
#include "clock_config.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "serialterminal.h"
#include "bluetoothterminal.h"
#include "lcd_func.h"
#include "time_memory_func.h"
#include "chat_app_main.h"
#include "menu.h"

int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	lcd_spi_pins_init();
	i2c_init_peripherals();
	bt_terminal_init();
	serial_terminal_init();

	vTaskStartScheduler();

	for (;;)
	{

	}

}


