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

QueueHandle_t get_serial_msg_queue(void);
QueueHandle_t get_serial_time_queue(void);
void serial_terminal_init(void);


#endif /* SERIALTERMINAL_H_ */
