#if 0
#include "LCDNokia5110.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BUTTON_UP_DOWN_HANDLER PORTB_IRQHandler
#define BUTTON_LEFT_RIGHT_HANDLER PORTC_IRQHandler

/*******************************************************************************
 * Structures
 ******************************************************************************/
typedef struct state {

	void (*ptr)(void); /**pointer to function*/
	uint8_t arrow_down;
	uint8_t arrow_up;
	uint8_t cancel_return;
	uint8_t done_ok;


} State;

typedef enum{

	DOWN_BUTTON = 0,
	UP_BUTTON,
	CANCEL_BUTTON,
	OK_BUTTON

} menu_msg_enum;

typedef uint8_t menu_msg_t;

QueueHandle_t g_menu_queue;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PIN_MASK_16	(1<<16)	/*BUTTON DOWN*/
#define PIN_MASK_17	(1<<17)	/*BUTTON UP*/
#define PIN_MASK_18	(1<<18)	/*BUTTON OK*/
#define PIN_MASK_19	(1<<19)	/*BUTTON CANCEL*/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void config_buttons(void);
void first_line_slct(void);
void second_line_slct(void);
void lcd_echo(void);
void lcd_time_config(void);
void set_time(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/

static const State menu_state[5] = {
		{ &first_line_slct, 1, 0, 0, 2},	/**will remark the first line*/
		{ &second_line_slct, 0, 1, 1, 3},	/**will remark second line*/
		{ &lcd_echo, 2, 2, 0, 2},			/**will be showing echo*/
		{ &lcd_time_config, 3, 3, 0, 4},	/**will be modifying hour*/
		{ &set_time, 0, 0, 0, 0}			/**time will be set*/
};

/*******************************************************************************
 * IRQ HANDLERS
 ******************************************************************************/
void BUTTON_UP_DOWN_HANDLER(void)
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/*this pointer stores the first address of the memory reserve for a structure*/
	static menu_msg_t msg;

	if (PIN_MASK_18 & PORT_GetPinsInterruptFlags(PORTB))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTB, PIN_MASK_18);

		msg = OK_BUTTON;

		/*once your done you can send the message waiting
		somebody to respond.*/
		xQueueSendToBackFromISR(g_menu_queue, &msg, &xHigherPriorityTaskWoken);

	}

	else if (PIN_MASK_19 & PORT_GetPinsInterruptFlags(PORTB))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTB, PIN_MASK_19);

		msg = CANCEL_BUTTON;

		/*once your done you can send the message waiting
		somebody to respond.*/
		xQueueSendToBackFromISR(g_menu_queue, &msg, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BUTTON_LEFT_RIGHT_HANDLER(void)
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/*this pointer stores the first address of the memory reserve for a structure*/
	static menu_msg_t msg;

	if (PIN_MASK_16 & PORT_GetPinsInterruptFlags(PORTC))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTC, PIN_MASK_16);

		msg = DOWN_BUTTON;

		/*once your done you can send the message waiting
		somebody to respond.*/
		xQueueSendToBackFromISR(g_menu_queue, &msg, &xHigherPriorityTaskWoken);
	}

	else if (PIN_MASK_17 & PORT_GetPinsInterruptFlags(PORTC))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTC, PIN_MASK_17);

		msg = UP_BUTTON;

		/*once your done you can send the message waiting
		somebody to respond.*/
		xQueueSendToBackFromISR(g_menu_queue, &msg, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
void first_line_slct(void)
{

}

void second_line_slct(void)
{

}

void lcd_echo(void)
{

}

void lcd_time_config(void)
{

}

void set_time(void)
{

}


/*******************************************************************************
 * TASKS
 ******************************************************************************/
void change_menu_task(void * pvParameters)
{
	static uint8_t current = 0;
	/*this pointer stores the first address of the memory reserved for a structure*/
	menu_msg_t msg;


	for(;;)
	{

		/**while the other task is going to receive the
		message and clean the queue after using those fields*/
		xQueueReceive(g_menu_queue,&msg,portMAX_DELAY);

		switch (msg)
		{

		case DOWN_BUTTON:
			current = menu_state[current].arrow_down;
			menu_state[current].ptr();
			break;

		case UP_BUTTON:
			current = menu_state[current].arrow_up;
			menu_state[current].ptr();
			break;

		case CANCEL_BUTTON:
			current = menu_state[current].cancel_return;
			menu_state[current].ptr();
			break;

		case OK_BUTTON:
			current = menu_state[current].done_ok;
			menu_state[current].ptr();
			break;

		}

	}

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


	vTaskStartScheduler();

	for(;;)
	{



	}
}

void config_buttons(void)
{

	/* Input pin PORT configuration */
	port_pin_config_t config_switch = { kPORT_PullUp, kPORT_SlowSlewRate,
			kPORT_PassiveFilterEnable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };

	gpio_pin_config_t switch_config = { kGPIO_DigitalInput, 0 };

	/*Habilitar el reloj SCG*/
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);

	PORT_SetPinInterruptConfig(PORTB, 19, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTB, 18, kPORT_InterruptFallingEdge);

	PORT_SetPinInterruptConfig(PORTC, 17, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(PORTC, 16, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(PORTB, 19, &config_switch);
	PORT_SetPinConfig(PORTB, 18, &config_switch);

	PORT_SetPinConfig(PORTC, 17, &config_switch);
	PORT_SetPinConfig(PORTC, 16, &config_switch);

	/* Sets the configuration */
	GPIO_PinInit(GPIOB, 19, &switch_config);
	GPIO_PinInit(GPIOB, 18, &switch_config);

	GPIO_PinInit(GPIOC, 17, &switch_config);
	GPIO_PinInit(GPIOC, 16, &switch_config);

	NVIC_EnableIRQ(PORTB_IRQn);
	NVIC_SetPriority(PORTB_IRQn, 7);

	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_SetPriority(PORTC_IRQn, 7);

	xTaskCreate(change_menu_task, "change_menu", 90, NULL,
	configMAX_PRIORITIES, NULL);

	g_menu_queue = xQueueCreate(1, sizeof(menu_msg_t));

}
#endif
