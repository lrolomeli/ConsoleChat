#ifndef LCDFUNC_H_
#define LCDFUNC_H_
#include "LCDNokia5110.h"
#include "LCDNokia5110Images.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"

void lcd_spi_pins_init(void);


#endif /* LCDFUNC_H_ */
