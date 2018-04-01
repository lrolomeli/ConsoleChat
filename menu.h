/*
 * menu.h
 *
 *  Created on: Mar 26, 2018
 *      Author: lrolo
 */

#ifndef MENU_H_
#define MENU_H_
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void init_menu(void);
void main_menu_task(void * pvParameters);
void print_time_task(void * pvParameters);
void print_time_lcd_task(void * pvParameters);

#endif /* MENU_H_ */
