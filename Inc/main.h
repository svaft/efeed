/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include "stddef.h"
#include "stdlib.h"
#include "string.h"

#include "fixedptc.h" 			// 8_24 format for spindle sync delay calculation
#include "fixedptc22_10.h" 	// steps per mm, min resolution is screw step / steps per rev / 2^10 = 1/400/1024=0,0000024mm
#include "fixedptc12_20.h" 	// mm, max 2048mm work field in this case, min resolution is about 0,000001mm

#define TRUE true
#define FALSE false
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define min_pulse 145*5
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define MOTOR_X_DIR_Pin LL_GPIO_PIN_7
#define MOTOR_X_DIR_GPIO_Port GPIOA
#define MOTOR_X_STEP_Pin LL_GPIO_PIN_0
#define MOTOR_X_STEP_GPIO_Port GPIOB
#define MOTOR_X_ENABLE_Pin LL_GPIO_PIN_1
#define MOTOR_X_ENABLE_GPIO_Port GPIOB
#define BUTTON_1_Pin LL_GPIO_PIN_8
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_2_Pin LL_GPIO_PIN_9
#define BUTTON_2_GPIO_Port GPIOA
#define MOTOR_Z_ENABLE_Pin LL_GPIO_PIN_3
#define MOTOR_Z_ENABLE_GPIO_Port GPIOB
#define MOTOR_Z_STEP_Pin LL_GPIO_PIN_4
#define MOTOR_Z_STEP_GPIO_Port GPIOB
#define MOTOR_Z_DIR_Pin LL_GPIO_PIN_5
#define MOTOR_Z_DIR_GPIO_Port GPIOB
#define ENC_A_Pin LL_GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin LL_GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOB
#define ENC_ZERO_Pin LL_GPIO_PIN_8
#define ENC_ZERO_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */


//#ifndef _SIMU
//#define _SIMU
//#endif /* _SIMU */


#define t3cr1			((uint32_t *)((0x42000000  + ((0x40000400)-0x40000000)*32)))

#define t4cr1			((uint32_t *)((0x42000000  + ((0x40000800)-0x40000000)*32)))
#define t4sr			((uint32_t *)((0x42000000  + ((0x40000810)-0x40000000)*32)))
#define t4dier		((uint32_t *)((0x42000000  + ((0x4000080C)-0x40000000)*32)))

#define disable_encoder_ticks() t4dier[TIM_DIER_UIE_Pos] = 0    
#define enable_encoder_ticks()  t4dier[TIM_DIER_UIE_Pos] = 1    

#define auto_mode_delay_ms 4000

#define step_divider 2 //stepper driver divider microstep

// main carriage

#define BIT_BAND_SRAM(RAM,BIT) (*(volatile uint32_t*)(SRAM_BB_BASE+32*((uint32_t)((void*)(RAM))-SRAM_BASE)+4*((uint32_t)(BIT))))
#define BB_PERI(c,d) *((volatile uint32_t *) ((PERIPH_BB_BASE + (uint32_t)( &( c ) - PERIPH_BASE)*32 + ( d*4 ))))
#define XDIR *((volatile uint32_t *) ((PERIPH_BB_BASE + (uint32_t)(  0x4001100C - PERIPH_BASE)*32 + ( 15*4 ))))


#define MOTOR_Z_SetPulse()           t3cr1[TIM_CR1_CEN_Pos] = 1 //bitbang version, or with LL: LL_TIM_EnableCounter(TIM3) 
#define MOTOR_Z_RemovePulse()        // dummy macro, pulse disabled by hardware

#define zdir_forward 1
#define zdir_backward 0
#define xdir_forward 1
#define xdir_backward 0

#define ZDIR *((volatile uint32_t *) ((PERIPH_BB_BASE + (uint32_t)(  0x4001080C - PERIPH_BASE)*32 + ( 7*4 ))))
//#define MOTOR_Z_Forward()            MOTOR_Z_DIR_GPIO_Port->BSRR        = MOTOR_Z_DIR_Pin
//#define MOTOR_Z_Reverse()            MOTOR_Z_DIR_GPIO_Port->BRR         = MOTOR_Z_DIR_Pin

#define MOTOR_Z_Enable()             MOTOR_Z_ENABLE_GPIO_Port->BSRR = MOTOR_Z_ENABLE_Pin
#define MOTOR_Z_Disable()            MOTOR_Z_ENABLE_GPIO_Port->BRR  = MOTOR_Z_ENABLE_Pin


#define LED_OFF()		LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin)
#define LED_ON()		LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin)

#define MOTOR_X_CHANNEL         		LL_TIM_CHANNEL_CH1
#define MOTOR_Z_CHANNEL         		LL_TIM_CHANNEL_CH3
#define MOTOR_Z_OnlyPulse()         TIM3->CCER = MOTOR_Z_CHANNEL
#define MOTOR_Z_AllowPulse()         t3ccer[TIM_CCER_CC3E_Pos] = 1
#define MOTOR_Z_BlockPulse()         t3ccer[TIM_CCER_CC3E_Pos] = 0

#define MOTOR_X_OnlyPulse()         TIM3->CCER = MOTOR_X_CHANNEL
#define MOTOR_X_AllowPulse()         t3ccer[TIM_CCER_CC1E_Pos] = 1
#define MOTOR_X_BlockPulse()         t3ccer[TIM_CCER_CC1E_Pos] = 0


#define z_to_x_factor2210	1537 //1024*200*61/16/1,27/400	todo move to some central point to modify

/*
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
*/
/*
typedef struct
{
    uint32_t current_pos;
    uint32_t end_pos;
    uint32_t end_pos_delta;
//    fsm_t mode;
//    fsm_t mode_prev;
    
    uint32_t Q824set;
    uint32_t fract_part; // Q8.24 format fract part
    uint64_t prolong_addSteps;
    uint64_t prolong_fract;
    uint8_t ramp_step;
} axis;
extern axis z_axis;
*/
struct state_s;
typedef void (*state_func_t)( struct state_s* );
typedef void (*callback_func_t)(struct state_s*);


typedef struct G_pipeline{
	int 
		X, // x axis
		Z, // z axis
		F, // feed
		P; // dwell

	int 
		I, // arc X axis delta
		K; // arc Z axis delta
//	bool sync; // wtf?
//	uint8_t code; // wtf?
} G_pipeline_t;

typedef struct G_task{
	int32_t dx, dz;
	int32_t 	x, z, x1, z1; // delta
	uint32_t steps_to_end;
	fixedptu F; //Q824
	callback_func_t callback_ref; //callback ref to iterate line or arc
	callback_func_t init_callback_ref;

	callback_func_t precalculate_init_callback_ref;
	callback_func_t precalculate_callback_ref;

	uint8_t z_direction, x_direction;

// arc
//	int rr, inc_dec;
	uint32_t a,b;
//	int64_t err, aa, bb;
} G_task_t;


typedef struct state_s
{
//	uint32_t steps_to_end;
	uint32_t current_pos;
	uint32_t end_pos;
	uint8_t ramp_step;
	uint32_t Q824set; // feed rate
	uint32_t fract_part; // Q8.24 format fract part
	
	// arc variables for precalculated in task init callback for current task:	
	int64_t arc_aa, arc_bb, arc_dx, arc_dz; // error increment
	int64_t arc_err; // error of 1.step
	
	uint32_t prescaler; // used for calculating substep delay
	int substep_axis;
	
	G_task_t current_task;
	bool task_lock;
	bool precalculate_end;
	
	bool G94G95; // 0 - unit per min, 1 - unit per rev
	uint32_t substep_mask;
  state_func_t function;
//  callback_func_t callback;
	uint32_t async_z;
	uint8_t z_period;
	bool f_encoder;
	bool f_tacho;
	bool spindle_dir;
	_Bool sync;
	_Bool main_feed_direction;
	TIM_TypeDef *syncbase;
  // other stateful data

	//	bresenham
	int err;
} state_t;

extern state_t state_precalc;
extern state_t state_hw;


#define SUBSTEP_AXIS_Z 0
#define SUBSTEP_AXIS_X 1



typedef struct {
	fixedptu Q824; //Q8.24 fixed math format
	uint8_t submenu;
	char Text[6];
	char Unit[6];
	uint8_t level;
	char infeed_mm[6];
	char infeed_inch[6];
	uint8_t infeed_strategy;
} THREAD_INFO;

extern const THREAD_INFO Thread_Info[];
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)

typedef struct {
	fixedptu Q824; //Q8.24 fixed math format
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
#define MOTOR_X_SetPulse()           t3cr1[TIM_CR1_CEN_Pos] = 1 //LL_TIM_EnableCounter(TIM3) //__HAL_TIM_ENABLE(&htim3)
#define MOTOR_X_RemovePulse()        // dummy macro, pulse disabled by hardware
#define XDIR *((volatile uint32_t *) ((PERIPH_BB_BASE + (uint32_t)(  0x4001100C - PERIPH_BASE)*32 + ( 15*4 ))))

//#define MOTOR_X_Forward()            MOTOR_X_DIR_GPIO_Port->BSRR    = MOTOR_X_DIR_Pin
//#define MOTOR_X_Reverse()            MOTOR_X_DIR_GPIO_Port->BRR     = MOTOR_X_DIR_Pin

#define MOTOR_X_Enable()             MOTOR_X_ENABLE_GPIO_Port->BSRR = MOTOR_X_ENABLE_Pin
#define MOTOR_X_Disable()            MOTOR_X_ENABLE_GPIO_Port->BRR  = MOTOR_X_ENABLE_Pin





// ***** Taho *****
//#define TahoSetPulse()               
//#define TahoRemovePulse()            


// ***** Encoder *****
//#define Enc_Line_per_Revolution      1800                         // Кол-во линий энкодера
//#define Enc_Line                     Enc_Line_per_Revolution*2    // Рабочее кол-во тиков

//#define Enc_Line_Q1648                (uint64_t)((uint64_t)Enc_Line << 48)

//#define Enc_Read()                   LL_TIM_GetCounter(TIM4) // __HAL_TIM_GetCounter(&htim4)

#define Spindle_Direction_CW         0                             // прямое вращение
#define Spindle_Direction_CCW        1                             // обратное вращение
#define feed_direction_left         0 // from right to left
#define feed_direction_right        1 // from left to right

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
