/*
 * time_memory_func.h
 *
 *  Created on: Mar 23, 2018
 *      Author: Usuario
 */

#ifndef TIME_MEMORY_FUNC_H_
#define TIME_MEMORY_FUNC_H_
#include "fsl_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#define MEMORY_READ_EVENT (1 << 0)
#define MEMORY_READ_DONE (1 << 1)

#define MASK_UNITS		(0X0FU)
#define MASK_TENS		(0XF0U)


#define UNITS_SECONDS		(5)
#define TENS_SECONDS		(4)
#define UNITS_MINUTES		(3)
#define TENS_MINUTES		(2)
#define UNITS_HOURS			(1)
#define TENS_HOURS			(0)


void i2c_init_peripherals(void);
QueueHandle_t get_time_mailbox(void);

void write_mem(int16_t subaddress, uint8_t buffer[], uint8_t dataSize);
uint8_t * read_mem(int16_t subaddress, uint16_t dataSize);
void write_time(uint8_t buffer[]);

#endif /* TIME_MEMORY_FUNC_H_ */
