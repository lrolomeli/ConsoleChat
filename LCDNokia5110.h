/*
 * LCDNokia5110.h
 *
 *  Created on: Jun 11, 2014
 *      Author: Luis
 */

#ifndef LCDNOKIA5110_H_
#define LCDNOKIA5110_H_

#include "DataTypeDefinitions.h"
#include "fsl_gpio.h"
#include "fsl_dspi.h"
#include "fsl_port.h"

#define SCREENW 84
#define SCREENH 48

#define LCD_DSPI_MASTER_BASEADDR SPI0
#define LCD_DSPI_SLAVE_BASEADDR SPI1
#define LCD_DSPI_MASTER_IRQ SPI0_IRQn
#define LCD_DSPI_MASTER_IRQHandler SPI0_IRQHandler
#define LCD_DSPI_SLAVE_IRQ SPI1_IRQn
#define LCD_DSPI_SLAVE_IRQHandler SPI1_IRQHandler
#define TRANSFER_BAUDRATE 500000U /*! Transfer baudrate - 500k */
#define TRANSFER_SIZE 1U         /*! Transfer dataSize */

#define LCD_X 84
#define LCD_Y 48
#define LCD_DATA 1
#define LCD_CMD 0
#define LCD_DC_PIN 3
#define LCD_DIN_PIN 2
#define LCD_CLK_PIN 1
#define LCD_RESET_PIN 0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* DSPI user callback */
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);

/*It configures the LCD*/
void LCDNokia_init(void);
/*It writes a byte in the LCD memory. The place of writting is the last place that was indicated by LCDNokia_gotoXY. In the reset state
 * the initial place is x=0 y=0*/
void LCDNokia_writeByte(uint8, uint8);
/*it clears all the figures in the LCD*/
void LCDNokia_clear(void);
/*It is used to indicate the place for writing a new character in the LCD. The values that x can take are 0 to 84 and y can take values
 * from 0 to 5*/
void LCDNokia_gotoXY(uint8 x, uint8 y);
/*It allows to write a figure represented by constant array*/
void LCDNokia_bitmap(const uint8*);
/*It write a character in the LCD*/
void LCDNokia_sendChar(uint8);
/*It write a string into the LCD*/
void LCDNokia_sendString(uint8*);
/*It used in the initialisation routine*/
void LCD_delay(void);



#endif /* LCDNOKIA5110_H_ */
