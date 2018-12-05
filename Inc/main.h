/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include "fixedptc.h"
#define TRUE true
#define FALSE false
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define min_pulse 145

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define MOTOR_X_DIR_Pin GPIO_PIN_15
#define MOTOR_X_DIR_GPIO_Port GPIOC
#define MOTOR_X_ENABLE_Pin GPIO_PIN_1
#define MOTOR_X_ENABLE_GPIO_Port GPIOA
#define MOTOR_X_STEP_Pin GPIO_PIN_6
#define MOTOR_X_STEP_GPIO_Port GPIOA
#define MOTOR_Z_DIR_Pin GPIO_PIN_7
#define MOTOR_Z_DIR_GPIO_Port GPIOA
#define MOTOR_Z_STEP_Pin GPIO_PIN_0
#define MOTOR_Z_STEP_GPIO_Port GPIOB
#define MOTOR_Z_ENABLE_Pin GPIO_PIN_1
#define MOTOR_Z_ENABLE_GPIO_Port GPIOB
#define BUTTON_1_Pin GPIO_PIN_8
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_2_Pin GPIO_PIN_9
#define BUTTON_2_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOB
#define ENC_ZERO_Pin GPIO_PIN_8
#define ENC_ZERO_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */


#ifndef _SIMU
#define _SIMU
#endif /* __MAIN_H */

#define t4cr1       ((uint32_t *)((0x42000000  + ((0x40000800)-0x40000000)*32)))
#define t4sr        ((uint32_t *)((0x42000000  + ((0x40000810)-0x40000000)*32)))
#define t4dier  ((uint32_t *)((0x42000000  + ((0x4000080C)-0x40000000)*32)))
#define disable_encoder_ticks() t4dier[TIM_DIER_UIE_Pos] = 0    
#define enable_encoder_ticks()  t4dier[TIM_DIER_UIE_Pos] = 1    

#define auto_mode_delay_ms 4000

#define step_divider 2 //stepper driver divider microstep

// main carriage
#define MOTOR_Z_SetPulse()           __HAL_TIM_ENABLE(&htim3)
#define MOTOR_Z_RemovePulse()        // dummy macro, pulse disabled by hardware
#define MOTOR_Z_Forward()            MOTOR_Z_DIR_GPIO_Port->BSRR        = MOTOR_Z_DIR_Pin
#define MOTOR_Z_Reverse()            MOTOR_Z_DIR_GPIO_Port->BRR         = MOTOR_Z_DIR_Pin
#define MOTOR_Z_Enable()             MOTOR_Z_ENABLE_GPIO_Port->BSRR = MOTOR_Z_ENABLE_Pin
#define MOTOR_Z_Disable()            MOTOR_Z_ENABLE_GPIO_Port->BRR  = MOTOR_Z_ENABLE_Pin


#define MOTOR_Z_BlockPulse()         TIM3->CCR3 = 0
#define MOTOR_X_BlockPulse()         TIM3->CCR1 = 0
#define MOTOR_Z_AllowPulse()         TIM3->CCR3 = min_pulse
#define MOTOR_X_AllowPulse()         TIM3->CCR1 = min_pulse


typedef enum {
	fsm_menu,										//0 . menu mode, if long_press_start event: go to sub-menu or up-menu, DOUBLE_CLICK: initial direction change
	fsm_menu_lps, 							//10. long_press_start: end_pos = current_pos = 0, идем в п. fsm_first_cut_lps
	fsm_first_cut_lps, 					//20. init selected mode, init direction, motor on, goto fsm_wait_tacho
	fsm_wait_tacho, 						//24. wait tacho pulse, go to 25
	fsm_first_cut_ramp_up, 			//25. tacho pulse interrupt: включаем прерывание по тикам энкодера, начинаем разгоняться(ramp up) по таблице пока не выйдем на расчетную скорость,далее в режим 26
	fsm_first_cut_main_part, 		//26. step until long_press_end event, then go to 27
	fsm_first_cut_lpe,					//27. long_press_end event: проверяем общее расчетное количество шагов(разгон+infeed+основной путь+торможение), при необходимости делаем дошагиваем до кратного целого в зависимости от микрошага, далее в режим торможения, п. 30
	fsm_first_cut_ramp_down,		//30. режим торможения(ramp down) пока по таблице разгона не дойдем обратно до нуля, останавливаем мотор, end_pos = current_pos, меняем направление, обновляем экран, идем в п.35
	fsm_wait_sclick,          	//35. ждем SINGLE_CLICK: если current_pos > 0 ? идем в mode = 40 иначе в	mode = 50
	fsm_sclick_event,         	//40. клик: включаем моторы обратно, идем в п.45
	fsm_main_cut_back_ramp_up,	//45. если счетчик current_pos > 0 то едем обратно до нуля: разгон
	fsm_main_cut_back,					//46. main path back to initial position. If long_press_start detected during process, activate prolonged mode ( 48).
	fsm_main_cut_back_ramp_down,//47. аналогично 27, торможение, останавливаем мотор, меняем направление, обновляем экран, идем в п.35
	fsm_main_cut_back_prolong,	//48. prolonged mode used to extend cutting path until long_press released. step back until current_pos reach start position add full revolution steps of servo. when released go back to 46
	fsm_main_cut_wait_tacho,		//50. клик: включаем моторы вперед, ждем тахо, идем в п.52
	fsm_main_cut_ramp_up,				//54. тахо пульс обнаружен, включаем прерывание по тикам энкодера, можно шагать, идем в п.55
	fsm_main_cut,								//55. если счетчик current_pos = 0 то в зависимости от выбранной стратегии вычисляем infeed и идем в режим резьбы до end_pos: разгон, далее идем в п.56
//стратегии врезания(infeed): 0: radial infeed, 1: incremental infeed, 2: modifyed flank infeed
	fsm_main_cut_infeed,				//56. infeed для резьбы: в зависимости от номера прохода сдвигаем каретку на определенное количество шагов для облегчения резания+основной путь, далее в п. 30
} fsm_t;


typedef struct
{
    uint32_t current_pos;
    uint32_t end_pos;
//    fsm_t mode;
//    fsm_t mode_prev;
    
    uint32_t Q824set;
    uint32_t fract_part; // Q8.24 format fract part
    uint64_t prolong_addSteps;
    uint64_t prolong_fract;
    uint8_t ramp_step;
} axis;


typedef struct {
	fixedptu Q824; //Q8.24 fix math format
	uint8_t submenu;
	char Text[6];
	char Unit[6];
	uint8_t level;
	char infeed_mm[6];
	char infeed_inch[6];
	uint8_t infeed_strategy;
} THREAD_INFO;

extern THREAD_INFO Thread_Info[];
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)

typedef struct {
	fixedptu Q824; //Q8.24 fix math format
	fixedptu pitch;
	uint8_t total_pass;
	uint8_t pass;

	fixedptu thread_depth;
//	fixedptu thread_angle; // tan(60 / 2 ) = 0,5774 >>>> fixedtp

	fixedptu infeed_mod; // =TAN(radians(B4/2-B5))
	fixedptu init_delta;
	fixedptu deltap_mm[20];
	fixedptu deltap_inch[20];
	uint8_t submenu;
	char Text[6];
	char Unit[6];
	uint8_t level;
	char infeed_mm[6];
	char infeed_inch[6];
	uint8_t infeed_strategy;
} S_WORK_SETUP;

extern axis z_axis;

extern uint32_t menu_changed;



// crosslide feed:
/*
#define MOTOR_X_STEP_Pin GPIO_PIN_0
#define MOTOR_X_STEP_GPIO_Port GPIOA
#define MOTOR_X_DIR_Pin GPIO_PIN_15
#define MOTOR_X_DIR_GPIO_Port GPIOC
#define MOTOR_X_ENABLE_Pin GPIO_PIN_1
#define MOTOR_X_ENABLE_GPIO_Port GPIOA
*/
#define MOTOR_X_SetPulse()           __HAL_TIM_ENABLE(&htim3)
#define MOTOR_X_RemovePulse()        // dummy macro, pulse disabled by hardware
#define MOTOR_X_Forward()            MOTOR_X_DIR_GPIO_Port->BSRR    = MOTOR_X_DIR_Pin
#define MOTOR_X_Reverse()            MOTOR_X_DIR_GPIO_Port->BRR     = MOTOR_X_DIR_Pin
#define MOTOR_X_Enable()             MOTOR_X_ENABLE_GPIO_Port->BSRR = MOTOR_X_ENABLE_Pin
#define MOTOR_X_Disable()            MOTOR_X_ENABLE_GPIO_Port->BRR  = MOTOR_X_ENABLE_Pin





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

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
