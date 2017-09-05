/*
 * error_code.h
 *
 *  Created on: 8Aug.,2017
 *      Author: yuri
 */

#ifndef ERROR_CODE_H_
#define ERROR_CODE_H_

typedef enum{
	ERROR_OK,
	ERROR_INIT_I2C,
	ERROR_INIT_UART,
	ERROR_FLASH_ERASE,
	ERROR_FLASH_WRITE,
	ERROR_I2C_SAMPLE,
	ERROR_UART_SEND,
}error_code_t;

#endif /* ERROR_CODE_H_ */
