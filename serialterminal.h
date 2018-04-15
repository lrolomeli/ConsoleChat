/*
 * serialterminal.h
 *
 *  Created on: Mar 28, 2018
 *      Author: lrolo
 */

#ifndef SERIALTERMINAL_H_
#define SERIALTERMINAL_H_

#include "terminal_func.h"

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief Share this queue when a module needs it
 	\param[in] none.
 	\return QueueHandle_t
 */
QueueHandle_t get_serial_msg_queue(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief Share this queue when a module needs it
 	\param[in] none.
 	\return QueueHandle_t
 */
QueueHandle_t get_serial_time_queue(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief Configure all peripherals needed to use UART SERIAL
 	\param[in] none.
 	\return void
 */
void serial_init(void);


#endif /* SERIALTERMINAL_H_ */
