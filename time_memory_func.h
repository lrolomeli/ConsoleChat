/*
 * time_memory_func.h
 *
 *  Created on: Mar 23, 2018
 *      Author: Usuario
 */

#ifndef TIME_MEMORY_FUNC_H_
#define TIME_MEMORY_FUNC_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#define MEMORY_READ_EVENT (1 << 0)
#define MEMORY_READ_DONE (1 << 1)

void i2c_init_peripherals(void);
EventGroupHandle_t get_i2c_event(void);


#endif /* TIME_MEMORY_FUNC_H_ */
