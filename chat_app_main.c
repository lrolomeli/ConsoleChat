#if 0
#include "terminal.h"
#include "lcd_func.h"

int main(void)
{

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

    terminal_init();
    lcd_spi_pins_init();
    vTaskStartScheduler();

    for(;;)
    {

    }

}
#endif
