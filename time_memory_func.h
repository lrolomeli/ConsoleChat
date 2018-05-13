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
 /******************************************************************************
 * DEFINITIONS
 ******************************************************************************/
#define MEMORY_READ_EVENT (1 << 0)
#define MEMORY_READ_DONE (1 << 1)

#define MASK_UNITS		(0X0FU)
#define MASK_TENS		(0XF0U)

#define MASK_FORMAT_AM_OR_PM_HOURS (0X40U)
#define MASK_FORMAT_24_OR_12_HOURS (0X80U)
#define MASK_FORMAT_Y_YEAR         (0XC0U)
#define MASK_FORMAT_U_MONTH        (0X0FU)
#define MASK_FORMAT_D_MONTH        (0X10U)
#define MASK_FORMAT_U_DAY          (0X0FU)
#define MASK_FORMAT_D_DAY     	   (0X30U)
#define SLADE4 					   (A<<4)
#define SLADE6 					   (A<<6)
#define SLADE7 					   (A<<7))
#define MASK_BIT1_YEAR			   (0X80U)
#define MASK_BIT2_YEAR	           (0X40U)

#define UNITS_SECONDS		(5)
#define TENS_SECONDS		(4)
#define UNITS_MINUTES		(3)
#define TENS_MINUTES		(2)
#define UNITS_HOURS			(1)
#define TENS_HOURS			(0)

#define UNITS_DAY			(0)
#define TENS_DAY			(1)
#define UNITS_MONTH			(2)
#define TENS_MONTH			(3)
#define UNITS_YEAR			(4)
#define TENS_YEAR			(5)
#define	HUNDRED_YEAR		(6)
#define UNITS_T_YEAR		(7)

 /********************************************************************************************/
 /********************************************************************************************!
 \brief:	initialize the peripherals in the board
 \param[in] 	UART BASE

 \return void
 ********************************************************************************************/
 /********************************************************************************************/
void i2c_init_peripherals(void);

/********************************************************************************************/
/********************************************************************************************!
\brief:	get the queue in the project to time variable

\return time_mailbox
********************************************************************************************/
/********************************************************************************************/
QueueHandle_t get_time_mailbox(void);

/********************************************************************************************/
/********************************************************************************************!
\brief:	write strings of characters in the external eprom
\param[in] 	subaddress
\param[in]  buffer []
\param[in]  dateSize

\return void
********************************************************************************************/
/********************************************************************************************/
void write_mem(int16_t subaddress, uint8_t buffer[], uint8_t dataSize);

/********************************************************************************************/
/********************************************************************************************!
\brief:	read the buffer of external eeprom
\param[in] 	subaddress
\param[in]  dateSize

\return uint8_t
********************************************************************************************/
/********************************************************************************************/
uint8_t * read_mem(int16_t subaddress, uint16_t dataSize);

/********************************************************************************************/
/********************************************************************************************!
\brief:	write in the specific space of rtc for configure the date
\param[in]  buffer []

\return void
********************************************************************************************/
/********************************************************************************************/
void write_time(uint8_t buffer[]);

/********************************************************************************************/
/********************************************************************************************!
\brief:	read the buffer of external eeprom
\param[in] 	subaddress
\param[in]  dateSize

\return uint8_t
********************************************************************************************/
/********************************************************************************************/
uint8_t * read_date(int16_t subaddress, uint16_t dataSize);

/********************************************************************************************/
/********************************************************************************************!
\brief:	write in the specific space of rtc for configure the date
\param[in]  buffer []

\return void
********************************************************************************************/
/********************************************************************************************/
void write_date(uint8_t buffer[]);

#endif /* TIME_MEMORY_FUNC_H_ */
