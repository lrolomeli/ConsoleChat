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

void i2c_init_peripherals(void);
EventGroupHandle_t get_i2c_event(void);

void write_mem(int16_t subaddress, uint8_t buffer[], uint8_t dataSize);
uint8_t * read_mem(int16_t subaddress, uint16_t dataSize);
#endif /* TIME_MEMORY_FUNC_H_ */
