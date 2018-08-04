/*
 * i2c_interface.c
 *
 *  Created on: 8Aug.,2017
 *      Author: yuri
 */

#include "stm32f1xx_hal.h"

#include "i2c_interface.h"
#include "string.h"
#include "main.h"
//#include "buttons.h"

#define USER_BUTTON_DEBOUNCE 5
sample_log_t i2c_device_logging;
I2C_HandleTypeDef *hi2c_ref;

volatile uint32_t dma_delay = 0;
uint32_t dma_delay2 = 0;
uint8_t  device_ready = 0;
uint8_t dma_data[6];
/**
 * Sent the first command to init the device
 * @param hi2c
 * @return
 */
error_code_t i2c_device_init(I2C_HandleTypeDef *hi2c){
  hi2c_ref = hi2c;
	uint8_t  handShake[2];
	handShake[0]=0xf0;
	handShake[1]=0x55;
	HAL_Delay(100);

	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		HAL_Delay(1);
	}
	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,handShake, 2) != HAL_OK){
		return ERROR_INIT_I2C;
	}
	HAL_Delay(20);
	dma_delay = 0;
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		dma_delay++;
		HAL_Delay(1);
	}	
//	HAL_Delay(100);
	handShake[0]=0xfb;
	handShake[1]=0x00;

	dma_delay = 0;
	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,handShake, 2)!= HAL_OK){
		dma_delay++;
		return ERROR_INIT_I2C;
	}
	HAL_Delay(20);
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		HAL_Delay(1);
	}	
//	HAL_Delay(100);
	device_ready = 1;
	return ERROR_OK;
}

error_code_t reqest_sample_i2c_dma(){
	if (device_ready == 0)
		return ERROR_OK;
	uint8_t cmd[1];
	cmd[0]=0x00;
//	dma_delay = 0;
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
		HAL_Delay(1);
	}	

	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,cmd, 1) != HAL_OK){
		return ERROR_I2C_SAMPLE;
	}
	
//	dma_delay = 0;
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
		HAL_Delay(1);
	}	
//	HAL_Delay(1);

//	dma_delay = 0;
	while(hi2c_ref->hdmarx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
		HAL_Delay(1);
	}	
	if(hi2c_ref->hdmarx->State == HAL_DMA_STATE_READY)
		HAL_I2C_Master_Receive_DMA(hi2c_ref, i2c_device_id, dma_data,6);

//	dma_delay = 0;
	HAL_Delay(1);
	while(hi2c_ref->hdmarx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
		HAL_Delay(1);
	}
//	dma_delay2 = dma_delay;
	return ERROR_OK;
}

/**
 * Reads the sample from the i2c devices. It reads tree accels and two buttons.
 * @param hi2c - i2c object
 * @param sample - out put sample pointer
 * @return error code
 */
error_code_t read_sample_i2c(i2c_sample_t *sample){
	if (device_ready == 0) //|| 	hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY)
		return ERROR_OK;
	uint8_t cmd[1];
	uint8_t data[6];
///	memset(&dma_data,);
	cmd[0]=0x00;

	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		HAL_Delay(1);
	}

	if (HAL_I2C_Master_Transmit_DMA(hi2c_ref,i2c_device_id,cmd, 1) != HAL_OK){
		return ERROR_I2C_SAMPLE;
	}
	while(hi2c_ref->hdmatx->State != HAL_DMA_STATE_READY){
		HAL_Delay(1);
	}

	if (HAL_I2C_Master_Receive(hi2c_ref, i2c_device_id, data,6,0x1000) != HAL_OK){
		return ERROR_I2C_SAMPLE;
	}
	sample->joy_x = data[0];
	sample->joy_y = data[1];
	sample->accel_x = (data[2] << 2)|((data[5] >> 2) & 0x03) ;
	sample->accel_y = (data[3] << 2)|((data[5] >> 4) & 0x03) ;
	sample->accel_z = (data[4] << 2)|((data[5] >> 6) & 0x03) ;
	sample->button_c=(data[5]&0x02)>>1;
	sample->button_z=data[5]&0x01;

	dma_data[5] = data[5];
	
	return ERROR_OK;
}
/**
 * touggle the value value of the temp pointer
 * @param temp
 */
void toggle(uint8_t* temp){
	if (*temp > 0){
		*temp = 0;
	}else {
		*temp = 1;
	}
}

/*
void sampling_task(void const * argument)
{

	button_status_t last_button_status = BUTTON_RELEASED;
	logging_status_t sampling_status = 0;
	uint8_t sampling_time_counter = 0;

	I2C_HandleTypeDef *hi2c2 = (I2C_HandleTypeDef*)argument;
//	read_flash((uint8_t*)FLASH_DATA_ADDR_START, (uint8_t*)&i2c_device_logging, sizeof(i2c_device_logging));

	i2c_device_init(hi2c2);

	for(;;)
	  {

//		if (check_user_button() == BUTTON_PRESSED && last_button_status == BUTTON_RELEASED){
//			toggle(&sampling_status);
//		}
		//update the last button status
//		last_button_status = check_user_button();


		if(sampling_status == LOGGING){
			if (sampling_time_counter > 10){ //10 times 50 miliseconds = 0.5 second
				sampling_time_counter = 0;
				if (i2c_device_logging.index >= SAMPLE_SIZE){
					i2c_device_logging.index = 0;
				}
				else i2c_device_logging.index++;

				read_sample_i2c(hi2c2,&i2c_device_logging.sample[i2c_device_logging.index]);
//				write_flash((uint8_t*)&i2c_device_logging, sizeof(i2c_device_logging));

//				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // just to see if is logging
			}
		else sampling_time_counter++;
		}
//		vTaskDelay(50);
	  }
}

*/
