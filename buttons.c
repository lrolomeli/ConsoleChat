
#include "LCDNokia5110.h"
#include "buttons.h"
#include "menu.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BUTTON_ENTER_ESC_HANDLER PORTB_IRQHandler
#define BUTTON_LEFT_RIGHT_HANDLER PORTC_IRQHandler
#define SHOW_DATE_TIME 0
#define MODIFY_TIME 1
#define MODIFY_DATE 2

/*******************************************************************************
 * Structures
 ******************************************************************************/
typedef struct state {

	void (*ptr)(void); /**pointer to function*/
	uint8_t next[2];

} State;

typedef enum {

	ENTER_BUTTON = 0,
	ESC_BUTTON

} menu_msg_enum;

typedef unsigned char menu_msg_t;

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
void date_time_lcd(void);
void modify_time(void);
void modify_date(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static const State menu_state[3] = {
		{ date_time_lcd, { MODIFY_TIME, MODIFY_DATE } },	/**will remark the first line*/
		{ modify_time, { SHOW_DATE_TIME, SHOW_DATE_TIME } },	/**will remark the first line*/
		{ modify_date, { SHOW_DATE_TIME, SHOW_DATE_TIME } }	/**will remark the first line*/

};

/*******************************************************************************
 * IRQ HANDLERS
 ******************************************************************************/
void BUTTON_ENTER_ESC_HANDLER(void)
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/*this pointer stores the first address of the memory reserve for a structure*/
	static menu_msg_t msg;

	if (PIN_MASK_18 & PORT_GetPinsInterruptFlags(PORTB))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTB, PIN_MASK_18);

		msg = ENTER_BUTTON;

		/*once your done you can send the message waiting
		somebody to respond.*/
		xQueueSendToBackFromISR(g_menu_queue, &msg, &xHigherPriorityTaskWoken);

	}

	else if (PIN_MASK_19 & PORT_GetPinsInterruptFlags(PORTB))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTB, PIN_MASK_19);

		msg = 	ESC_BUTTON;

		/*once your done you can send the message waiting
		somebody to respond.*/
		xQueueSendToBackFromISR(g_menu_queue, &msg, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void BUTTON_LEFT_RIGHT_HANDLER(void)
{


	if (PIN_MASK_16 & PORT_GetPinsInterruptFlags(PORTC))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTC, PIN_MASK_16);

	}

	else if (PIN_MASK_17 & PORT_GetPinsInterruptFlags(PORTC))
	{
		/*Clean flag which cause the interrupt*/
		PORT_ClearPinsInterruptFlags(PORTC, PIN_MASK_17);

	}

}

/*******************************************************************************
 * TASKS
 ******************************************************************************/
void change_menu_task(void * pvParameters)
{
	static uint8_t current = 0;
	menu_msg_t button_from_IRQ = 0;

	for(;;)
	{
		if(pdPASS == xQueueReceive(g_menu_queue, &button_from_IRQ, 0))
		{
			current = menu_state[current].next[button_from_IRQ];

		}
		menu_state[current].ptr();

	}

}

/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
void date_time_lcd(void)
{
	PRINTF("Hello World\r\n");
}

void modify_time(void)
{
	//print_time_lcd();
}

void modify_date(void)
{
	PRINTF("Hello Bug\r\n");
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

	xTaskCreate(change_menu_task, "change_menu", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES, NULL);

	g_menu_queue = xQueueCreate(1, sizeof(menu_msg_t));

}

