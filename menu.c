#ifdef 0
#include "LCDNokia5110.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BUTTON1_HANDLER PORTA_IRQHandler
#define BUTTON2_HANDLER PORTC_IRQHandler
/*******************************************************************************
 * Structures
 ******************************************************************************/
typedef struct state {

	void (*ptr)(void); /**pointer to function*/
	uint8_t next;
	uint8_t back;

} State;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void config_buttons(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool sw2 = false;
volatile bool sw3 = false;
volatile uint8_t current = 0;
static const State led_state[4] = {
		{ &first_line_slct, 0, 1, 0, 2},	/**will remark the first line*/
		{ &second_line_slct, 1, 0, 1, 3},	/**will remark second line*/
		{ &lcd_echo, 2, 2, 0, 2},			/**will be showing echo*/
		{ &lcd_time_config, 3, 3, 0, 4},	/**will be modifying hour*/
		{ &set_time, 0, 0, 0, 0}			/**time will be set*/
};
/*******************************************************************************
 * Code
 ******************************************************************************/

void BUTTON1_HANDLER()
{

	/*Limpiamos la bandera del pin que causo la interrupcion*/
	PORT_ClearPinsInterruptFlags(PORTA, 1 << 4);

	/*Si state es igual a 0 entonces se hace 1 y al revÃ©s*/
	sw2 = (0 == sw2) ? 1 : 0;
}

void BUTTON2_HANDLER()
{

	/*Limpiamos la bandera del pin que causo la interrupcion*/
	PORT_ClearPinsInterruptFlags(PORTC, 1 << 6);

	/*Si state es igual a 0 entonces se hace 1 y al revÃ©s*/
	sw3 = (0 == sw3) ? 1 : 0;
}

/*!
 * @brief Main function
 */
int main(void)
{

	/* Board pin, clock, debug console init */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	config_buttons();


	while (true)
	{

			if(sw3)
			{

			}
			else if (sw2)
			{
				current = led_state[current].next;
				led_state[current].ptr();
			}
			else
			{
				//calling function in current state
				current = led_state[current].back;
				led_state[current].ptr();
			}

	}
}

void config_buttons(void)
{

	/* Input pin PORT configuration */
	port_pin_config_t config_switch = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterEnable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };

	gpio_pin_config_t switch_config = { kGPIO_DigitalInput, 0 };

	/*Habilitar el reloj SCG*/
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortC);

	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(PORTA, 4, &config_switch);
	PORT_SetPinConfig(PORTC, 6, &config_switch);

	/* Sets the configuration */
	GPIO_PinInit(GPIOA, 4, &switch_config);
	GPIO_PinInit(GPIOC, 6, &switch_config);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);

}
#endif
