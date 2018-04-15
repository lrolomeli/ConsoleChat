/*
 * bluetooth.h
 *
 *  Created on: Mar 28, 2018
 *      Author: lrolo
 */

#ifndef BLUETOOTHTERMINAL_H_
#define BLUETOOTHTERMINAL_H_

#include "terminal_func.h"

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief Share this queue when a module needs it
 	\param[in] none.
 	\return QueueHandle_t
 */
QueueHandle_t get_bt_msg_queue(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief Share this queue when a module needs it
 	\param[in] none.
 	\return QueueHandle_t
 */
QueueHandle_t get_bt_time_queue(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief Configure all peripherals needed to use UART BLUETOOTH
 	\param[in] none.
 	\return void
 */
void bluetooth_init(void);

#endif /* BLUETOOTHTERMINAL_H_ */
