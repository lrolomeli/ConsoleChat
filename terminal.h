#ifndef TERMINAL_H_
#define TERMINAL_H_
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

typedef struct {

	uint8_t message[20];
	uint8_t message_size;

} echo_data_type;


#define READ_FROM_KEYBOARD (1 << 9)
#define READ_MENU_FROM_KEYBOARD (1 << 11)
#define READ_MENU_EVENT (1 << 12)
#define DEPLOY_MENU (1 << 13)
#define DEPLOY_MENU_DONE (1 << 14)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void terminal_init(void);
EventGroupHandle_t get_uart_event(void);
QueueHandle_t get_uart_mailbox(void);
void storing_phrase(uint8_t phrase_length, uint8_t * phrase);
#endif /* TERMINAL_H_ */
