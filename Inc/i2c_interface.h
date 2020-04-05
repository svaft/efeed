/*
 * i2c_interface.h
 *
 *  Created on: 8Aug.,2017
 *      Author: yuri
 */

#ifndef I2C_INTERFACE_H_
#define I2C_INTERFACE_H_

//#include "stm32f1xx_hal.h"
#include "main.h"
#include "error_code.h"

#define User_button_Pin GPIO_PIN_0
#define User_button_GPIO_Port GPIOA

#define i2c_device_id 	0x52<<1 // joystick

#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01

#define SAMPLE_SIZE		10

/* Define used to enable time-out management*/
#define USE_TIMEOUT       1


extern __IO uint8_t  ubMasterRequestDirection;
extern __IO uint8_t  ubMasterXferDirection;
extern __IO uint8_t  ubMasterNbDataToReceive;
extern __IO uint8_t  ubTransferComplete;


void Configure_I2C_Master(void);
void Activate_I2C_Master(void);
//void Handle_I2C_Master(I2C_TypeDef *, uint8_t , uint8_t *, uint16_t, uint8_t );
int Handle_I2C_MasterDMA_IT(I2C_TypeDef *, uint8_t , uint8_t *, uint16_t, uint8_t );
int Handle_I2C_MasterDMA_IT_async(uint8_t , uint8_t *, uint16_t);

void Transfer_Complete_Callback(void);
void Transfer_Error_Callback(void);

void Error_Callback(void);
void FlushBuffer8(uint8_t* );


void     LED_On(void);
void     LED_Off(void);
void     LED_Blinking(uint32_t Period);

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000


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

extern sample_log_t i2c_device_logging;
extern uint8_t  device_ready;
extern volatile uint32_t dma_delay;
typedef enum{
	NOT_LOGGING,
	LOGGING
}logging_status_t;

extern uint8_t dma_data[];

void sampling_task(void const * argument);
error_code_t i2c_device_init(I2C_TypeDef *hi2c);
error_code_t read_sample_i2c(i2c_sample_t *sample);
error_code_t reqest_sample_i2c_dma(I2C_TypeDef *hi2c);

#endif /* I2C_INTERFACE_H_ */
