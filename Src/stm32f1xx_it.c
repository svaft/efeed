/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "fixedptc.h"
#include "ssd1306.h"

#if	defined ( _SIMU )
#define tacho tacho_debug
bool encoder;

#else
//#define tacho						t4sr[TIM_SR_CC3IF_Pos]
//#define encoder t4sr[TIM_SR_UIF_Pos]
#endif



_Bool ramp_down(void);
inline _Bool ramp_up(void);
void move(void);
void at_move_end(void);
extern bool feed_direction;

//extern TIM_HandleTypeDef htim3;
//extern uint32_t count;
uint32_t count2 = 0;
extern uint8_t Spindle_Direction;


extern uint32_t Q824set;
extern uint32_t Q824count;
//extern uint32_t infeed_steps;
uint32_t infeed_step = 0;
uint32_t infeed_steps = 10;

extern fixedptud prolong_addSteps;
fixedptud prolong_fract = 0;


extern uint32_t current_pos, thread_limit, mode;
extern uint32_t buttons_mstick;
extern uint32_t menu_changed;

uint16_t infeed_map[]= {
	0,
	188,
	264,
	322,
	370,
	413,
	452,
	488,
	521,
	552,
	582,
	610,
	637,
	662,
	687,
	711,
	734,
	756,
};


const uint8_t ramp[]= {
75,
65,
45,
28,
24,
20,
17,
15,
14,
13,
12,
11,
10,
10,
9,
9,
9,
8,
8,
8,
8,
7,
7,
7,
7,
7,
7,
6,
6,
6,
6,
6,
6,
6,
6,
6,
5,
5,
5,
5,
5,
5,
5,
5,
5,
5,
};
extern uint32_t ramp_step; // = 0;
#define ramp_map 46

extern bool auto_mode;
extern int32_t auto_mode_delay;

uint32_t tacho_cnt = 0;
uint32_t tacho_debug = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  
  /* USER CODE BEGIN SysTick_IRQn 1 */


#if	defined ( _SIMU )

//simulate spindle
	if(++tacho_cnt == 1800 ) {
		tacho_debug = 1;
		tacho_cnt = 0;
		TIM4_IRQHandler();
	}
	if(++TIM4->CNT > TIM4->ARR) {
		TIM4->CNT = 0; // overflow emulation
		encoder = true;
		TIM4_IRQHandler();
	}
#endif

//			if(auto_mode_delay > 0)
//							auto_mode_delay--;
	if( buttons_mstick > 0 )
		buttons_mstick++;
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
  if(LL_DMA_IsActiveFlag_TC4(DMA1))
  {
    LL_DMA_ClearFlag_GI4(DMA1);
    Transfer_Complete_Callback();
//    DMA1_Transfer_Complete_Callback();
  }
  else if(LL_DMA_IsActiveFlag_TE4(DMA1))
  {
    Transfer_Error_Callback();
  }

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	uint32_t t4flags = TIM4->SR;
	TIM4->SR = 0;
	_Bool dir = t4flags & TIM_CR1_DIR_Msk; // t4cr1[TIM_CR1_DIR_Pos];
	if(t4flags & TIM_SR_UIF_Msk) {
		if ( dir != Spindle_Direction ) {
// direction changed, count not updated and no pulse to motor
			Spindle_Direction		= dir;
		} else {
			switch(mode) {
			case 20:				{ // not used?
				disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
				MOTOR_Z_Enable();
				if(feed_direction)
					MOTOR_Z_Forward();
				else
					MOTOR_Z_Reverse();
				mode = 24; //intermediate state to wait tacho pulse.
				break;
			}
			case 25:				{ // direct movement: first pass, thread recording: ramp up: accel by ramp map
				MOTOR_Z_SetPulse();
				current_pos++;
				if(ramp_up()) {
					mode = 26;
					LED_GPIO_Port->BSRR = LED_Pin; //led off
				}
				break;
			}
			case 26:				{ // direct movement: first pass, thread recording: main part
				MOTOR_Z_SetPulse();
				current_pos++;
				move();
				break;
			}
			case 27:				{ // direct movement: first pass, thread recording: post-main part
				// для 1/2 микрошага нужно что бы общее количество шагов в цикле резьбы было кратно 2,(для 1/4 кратно 4 и тп).
				// это нужно для того что бы в конце шаговый мотор остановился на одном из двухсот устойчивых шагов,
				// не перескакивая на соседние шаги при потере питания.
				// поэтому проверяем общее количество на четность(0й бит), если нечетное число делаем еще один шаг,
				// иначе начинаем замедляться
				MOTOR_Z_SetPulse();
				current_pos++;
				uint32_t all_count = ramp_step + current_pos - 1;
				uint32_t masked_count = all_count & ~(step_divider - 1);
				if(masked_count != all_count) {
					move();
				} else {
					if(ramp_down()) {
						thread_limit = current_pos;
						at_move_end();
					} else {
						mode = 30;
					}
				}
				break;
			}
			case 30:				{ // direct movement: ramp down: deccel part + stop
				MOTOR_Z_SetPulse();
				current_pos++;
				if(ramp_down()) {
					thread_limit = current_pos;
					at_move_end();
				}
				break;
			}

			case 35:				{// do nothing here and wait button click event, interrupt for encoder ticks can be disabled
				break;
			}
			case 40:				{ // reverse movement: set direction for motor
				if(feed_direction)
					MOTOR_Z_Forward();
				else
					MOTOR_Z_Reverse();
				enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
				mode = 45;
				break;
			}
			case 45:				{ // reverse movement: ramp up: accel part
				MOTOR_Z_SetPulse();
				--current_pos;
				if(ramp_up())
					mode = 46;
				break;
			}
			case 46:				{ // reverse movement: main part
				MOTOR_Z_SetPulse();
				if( --current_pos > ramp_step ) {
				} else {
					mode = 47;
				}
				break;
			}
			case 47:				{ // reverse movement: ramp down: deccel part + stop
				if (current_pos > 0) {
					MOTOR_Z_SetPulse();
					--current_pos;
				}
				if(ramp_down()) {
					at_move_end();
				}
				break;
			}
			case 48:				{ // reverse movement: main part with prolong activated. todo split it with 46 mode?
				MOTOR_Z_SetPulse();
				--current_pos;
				if(current_pos == ramp_step) { // we reach end of main path and have long_pressed key, so just add additional thread full turn to shift initial start point
					prolong_fract += prolong_addSteps; // fract part from prev step
					uint32_t prolong_fixpart = prolong_fract >> 24;
					current_pos += prolong_fixpart; // add fixed part
					thread_limit += prolong_fixpart;
					prolong_fract &= FIXEDPT_FMASK; // leave fract part to accumulate with next dividing cycle
					// when long_press end, get back to 46 mode to proceed
				}
				break;
			}

			case 50:				{ // direct movement: set direction for motor
				if(feed_direction)
					MOTOR_Z_Forward();
				else
					MOTOR_Z_Reverse();
				mode = 54; //intermediate state to wait tacho pulse
				disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
				break;
			}

			case 55:				{ // direct movement: ramp up: accel by ramp map
				MOTOR_Z_SetPulse();
				current_pos++;
				if(ramp_up()) {
					LED_GPIO_Port->BSRR = LED_Pin;	 // led off
					mode = 56;
				}
				break;
			}
			case 56:				{ // direct movement: main part
				MOTOR_Z_SetPulse();
				current_pos++;
				if( current_pos < ( thread_limit - ramp_step ) ) {
					move();
				} else {
					mode = 30;
				}
				break;
			}
			}
		}
	}
// tacho event found!
	if( t4flags & TIM_SR_CC3IF_Msk ) {
//	if( TIM4->SR & TIM_SR_CC3IF ) {
		tacho_debug = 0;
		if (dir == Spindle_Direction_CW ) {
			count2++;
		} else {
			count2--;
		}
		switch(mode) {
		case 35:
			MOTOR_Z_Disable(); //disable motor
			break;

		case 24: {
			mode = 25;
//													TIM4->ARR = fixedpt_toint(Q824set) - 1;
			infeed_step = 0;
			LED_GPIO_Port->BRR = LED_Pin; //led on
			TIM4->ARR = 1; // start stepper motor ramp up procedure immediately after tacho event
			TIM4->CNT = 0;
			enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
			// set ARR register to integer part of Q8.24:
			// for example for 1.5mm thread cutting of 400step stepper for 1800 line encoder
			// we have to step every (5-1) = 4 encoder tick, so set ARR = 4
//													Q824count = fixedpt_fracpart( Q824set ); // save fract part
			break;
		}
		case 54: {
			mode = 55;
			//reinit counter
//													TIM4->ARR = fixedpt_toint(Q824set) - 1;
			LED_GPIO_Port->BRR = LED_Pin; //led on

			if(infeed_step < infeed_steps) {
				TIM4->ARR = infeed_map[infeed_step++] + 1; // start stepper motor with shifted position by infeed map
			} else {
				TIM4->ARR = 1;	// start stepper motor ramp up procedure immediately after tacho event
			}

			TIM4->CNT = 0;
			enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
//													Q824count = fixedpt_fracpart( Q824set ); // save fract part
			break;
		}
		default:
			break;
		}
	}

  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */
  /* Check SB flag value in ISR register */
  if(LL_I2C_IsActiveFlag_SB(I2C2))
  {
    /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a write request */
    LL_I2C_TransmitData8(I2C2, SSD1306_I2C_ADDR);
  }
  /* Check ADDR flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_ADDR(I2C2))
  {
    /* Enable DMA transmission requests */
    LL_I2C_EnableDMAReq_TX(I2C2);

    /* Clear ADDR flag value in ISR register */
    LL_I2C_ClearFlag_ADDR(I2C2);
  }

  /* USER CODE END I2C2_EV_IRQn 0 */
  
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/* USER CODE BEGIN 1 */
_Bool ramp_up(void)
{
	uint8_t	set_without_fract = ramp[ramp_step];
	if(Q824set > ((uint32_t)set_without_fract<< FIXEDPT_FBITS) || ramp_step == ramp_map ) { // reach desired speed or end of ramp map
		Q824count = 0;
		TIM4->ARR = fixedpt_toint(Q824set) - 1; // update register ARR
		Q824count = fixedpt_fracpart(Q824set); // save fract part for future use on next step
		return true;
	}		else {
		ramp_step++;
		TIM4->ARR = set_without_fract - 1; // update register ARR
	}
	return false;
}

inline _Bool ramp_down(void)
{
	if (ramp_step == 0)
		return true;
	uint8_t	set_without_fract = ramp[--ramp_step];
	TIM4->ARR = set_without_fract - 1; // update register ARR
	if(ramp_step == 0)
		return true;
	return false;
}

void move(void)
{
	const fixedptu set_with_fract = fixedpt_add(Q824set, Q824count); // calculate new step delay with fract from previous step
	TIM4->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
	Q824count = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
}

void at_move_end(void)
{
	disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
//			MOTOR_Z_Disable(); //disable motor
	if(auto_mode == true)		auto_mode_delay = auto_mode_delay_ms; // reengage auto mode
	feed_direction = !feed_direction; //change feed direction
	menu_changed = 1; //update menu
	mode = 35; // dummy mode
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
