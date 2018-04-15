/*
 * buttons.h
 *
 *  Created on: Apr 6, 2018
 *      Author: pelon
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

void config_buttons(void);

#endif /* BUTTONS_H_ */
