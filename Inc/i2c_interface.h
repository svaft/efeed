/*
 * i2c_interface.h
 *
 *  Created on: 8Aug.,2017
 *      Author: yuri
 */

#ifndef I2C_INTERFACE_H_
#define I2C_INTERFACE_H_

#define User_button_Pin GPIO_PIN_0
#define User_button_GPIO_Port GPIOA

#define i2c_device_id 	0x52<<1

#define SAMPLE_SIZE		10

#include "stm32f1xx_hal.h"

typedef enum{
	BUTTON_RELEASED,
	BUTTON_PRESSED
}button_status_t;


typedef struct{
	uint16_t joy_x;
	uint16_t joy_y;
	uint16_t accel_x;
	uint16_t accel_y;
	uint16_t accel_z;
	uint8_t button_c;
	uint8_t button_z;
}i2c_sample_t;

typedef struct{
	uint8_t index;
	i2c_sample_t sample[SAMPLE_SIZE];
}sample_log_t;

typedef enum{
	NOT_LOGGING,
	LOGGING
}logging_status_t;

void sampling_task(void const * argument);

#endif /* I2C_INTERFACE_H_ */
