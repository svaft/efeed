/*
 * screen.h
 *
 */

#ifndef SCREEN_H_
#define SCREEN_H_

//#include "stm32f1xx_hal.h"
#include "main.h"

#define auto_symbol 0
#define left_arrow	1
#define right_arrow 2


void init_screen(I2C_TypeDef *hi2c);
int update_screen(void);



#endif /* SCREEN_H_ */
