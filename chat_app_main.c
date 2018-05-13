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
	//config_buttons();

	/**start the parameters of uart bluetooth */
	bluetooth_init();

	/**start the parameters of uart serial terminal */
	serial_init();

	/**start the parameters of i2c to use rtc and eprom */
	i2c_init_peripherals();

	/**start the parameters of SPI to lcd */
	lcd_spi_pins_init();

	vTaskStartScheduler();

	for (;;)
	{

	}

}


