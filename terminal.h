#ifndef TERMINAL_H_
#define TERMINAL_H_
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void terminal_init(void);
EventGroupHandle_t get_uart_event(void);
void storing_phrase(uint8_t phrase_length, uint8_t * phrase);
#endif /* TERMINAL_H_ */
