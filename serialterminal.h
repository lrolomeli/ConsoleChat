/*
 * serialterminal.h
 *
 *  Created on: Mar 28, 2018
 *      Author: lrolo
 */

#ifndef SERIALTERMINAL_H_
#define SERIALTERMINAL_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

EventGroupHandle_t get_serialterm_event(void);

void serial_terminal_init(void);


#endif /* SERIALTERMINAL_H_ */
