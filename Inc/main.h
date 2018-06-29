/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#define TRUE true
#define FALSE false
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define min_pulse 145

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define MOTOR_X_STEP_Pin GPIO_PIN_6
#define MOTOR_X_STEP_GPIO_Port GPIOA
#define MOTOR_X_DIR_Pin GPIO_PIN_7
#define MOTOR_X_DIR_GPIO_Port GPIOA
#define MOTOR_X_ENABLE_Pin GPIO_PIN_1
#define MOTOR_X_ENABLE_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOB
#define ENC_ZERO_Pin GPIO_PIN_8
#define ENC_ZERO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


#ifndef _SIMU
//#define _SIMU
#endif /* __MAIN_H */

#define t4cr1       ((uint32_t *)((0x42000000  + ((0x40000800)-0x40000000)*32)))
#define t4sr        ((uint32_t *)((0x42000000  + ((0x40000810)-0x40000000)*32)))
#define t4dier  ((uint32_t *)((0x42000000  + ((0x4000080C)-0x40000000)*32)))
#define disable_encoder_ticks() t4dier[TIM_DIER_UIE_Pos] = 0    
#define enable_encoder_ticks()  t4dier[TIM_DIER_UIE_Pos] = 1    

#define auto_mode_delay_ms 4000

#define step_divider 2 //stepper driver divider microstep




#define Motor_X_SetPulse()           __HAL_TIM_ENABLE(&htim3)
#define Motor_X_RemovePulse()        // dummy macro, pulse disabled by hardware
#define Motor_X_Forward()            MOTOR_X_DIR_GPIO_Port->BSRR = MOTOR_X_DIR_Pin
#define Motor_X_Reverse()            MOTOR_X_DIR_GPIO_Port->BRR = MOTOR_X_DIR_Pin
#define Motor_X_Enable()             MOTOR_X_ENABLE_GPIO_Port->BSRR = MOTOR_X_ENABLE_Pin
#define Motor_X_Disable()            MOTOR_X_ENABLE_GPIO_Port->BRR = MOTOR_X_ENABLE_Pin


// ***** Taho *****
#define TahoSetPulse()               
#define TahoRemovePulse()            


// ***** Encoder *****
#define Enc_Line_per_Revolution      1800                         // Кол-во линий энкодера
#define Enc_Line                     Enc_Line_per_Revolution*2    // Рабочее кол-во тиков

#define Enc_Line_Q1648                (uint64_t)((uint64_t)Enc_Line << 48)

#define Enc_Read()                   __HAL_TIM_GetCounter(&htim4)

#define Spindle_Direction_CW         0                             // прямое вращение
#define Spindle_Direction_CCW        1                             // обратное вращение
#define feed_direction_left         0 // from right to left
#define feed_direction_right        1 // from left to right

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
