/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "fixedptc.h"
#include "buttons.h"


#if  defined ( _SIMU )
#define tacho tacho_debug
bool encoder;
#else
#define tacho 	t4sr[TIM_SR_CC3IF_Pos]
#define encoder t4sr[TIM_SR_UIF_Pos]
#endif

_Bool ramp_down(void);
inline _Bool ramp_up(void);
void move(void);
void at_move_end(void);


extern bool feed_direction;
extern TIM_HandleTypeDef htim3;
extern uint8_t Spindle_Direction;

uint32_t infeed_step = 0;
uint32_t infeed_steps = 10;


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


uint32_t ramp[]= {
	0x1E000000,
//      0x00000000, // zero delay to disable ramp up
	0x15E353F7,
	0x110624DD,
	0x0E67A909,
	0x0CB5D163,
	0x0B7FEE35,
	0x0A946A82,
	0x09D9A0F5,
	0x0940CD81,
	0x08C0C265,
	0x0853743B,
	0x07F4B905,
	0x07A19758,
	0x0757DEEB,
	0x0715E90F,
	0x06DA701C,
	0x06A47489,
	0x06732AAA,
	0x0645EDE1,
	0x061C377E,
	0x05F59819,
	0x05D1B2A3,
	0x05B038B1,
	0x0590E7A4,
	0x0573867F,
	0x0557E426,
	0x053DD60D,
	0x0525371D,
	0x050DE6D9,
	0x04F7C8A5,
	0x04E2C336,
	0x04CEC017,
	0x04BBAB40,
	0x04A972C8,
	0x04980698,
	0x04875834,
	0x04775A84,
	0x046801AD,
	0x045942E9,
	0x044B1467,
	0x043D6D31,
	0x04304514,
	0x0423948C,
	0x041754AE,
	0x040B7F1D,
	0x04000DF9,
};
#define ramp_map 50

extern bool auto_mode;
extern int32_t auto_mode_delay;

uint32_t tacho_cnt = 0;
uint32_t tacho_debug = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim4;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */


#if  defined ( _SIMU )

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

//      if(auto_mode_delay > 0)
//              auto_mode_delay--;
	for(int a = 0; a<BT_TOTAL-1;a++){
		if( bt[a].buttons_mstick > 0 )
			bt[a].buttons_mstick++;
	}
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

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	_Bool dir = t4cr1[TIM_CR1_DIR_Pos];
	if(encoder) {
		if ( dir != Spindle_Direction ) {
// direction changed, count not updated and no pulse to motor
			Spindle_Direction    = dir;
		} else {
			switch(z_axis.mode) {
			case 20:        { // not used?
				disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
				MOTOR_Z_Enable();
				if(feed_direction)
					MOTOR_Z_Forward();
				else
					MOTOR_Z_Reverse();
				z_axis.mode = 24; //intermediate state to wait tacho pulse.
				break;
			}
			case 25:        { // direct movement: first pass, thread recording: ramp up: accel by ramp map
				MOTOR_Z_SetPulse();
				z_axis.current_pos++;
				if(ramp_up()) {
					z_axis.mode = 26;
					LED_GPIO_Port->BSRR = LED_Pin; //led off
				}
				break;
			}
			case 26:        { // direct movement: first pass, thread recording: main part
				MOTOR_Z_SetPulse();
				z_axis.current_pos++;
				move();
				break;
			}
			case 27:        { // direct movement: first pass, thread recording: post-main part
				// для 1/2 микрошага нужно что бы общее количество шагов в цикле резьбы было кратно 2,(для 1/4 кратно 4 и тп).
				// это нужно для того что бы в конце шаговый мотор остановился на одном из двухсот устойчивых шагов,
				// не перескакивая на соседние шаги при потере питания.
				// поэтому проверяем общее количество на четность(0й бит), если нечетное число делаем еще один шаг,
				// иначе начинаем замедляться
				MOTOR_Z_SetPulse();
				z_axis.current_pos++;
				uint32_t all_count = z_axis.ramp_step + z_axis.current_pos - 1;
				uint32_t masked_count = all_count & ~(step_divider - 1);
				if(masked_count != all_count) {
					move();
				} else {
					if(ramp_down()) {
						z_axis.end_pos = z_axis.current_pos;
						at_move_end();
					} else {
						z_axis.mode = 30;
					}
				}
				break;
			}
			case 30:        { // direct movement: ramp down: deccel part + stop
				MOTOR_Z_SetPulse();
				z_axis.current_pos++;
				if(ramp_down()) {
					z_axis.end_pos = z_axis.current_pos;
					at_move_end();
				}
				break;
			}

			case 35:        { // do nothing here and wait button click event, interrupt for encoder ticks can be disabled
				break;
			}
			case 40:        { // reverse movement: set direction for motor
				if(feed_direction)
					MOTOR_Z_Forward();
				else
					MOTOR_Z_Reverse();
				enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
				z_axis.mode = 45;
				break;
			}
			case 45:        { // reverse movement: ramp up: accel part
				MOTOR_Z_SetPulse();
				--z_axis.current_pos;
				if(ramp_up())
					z_axis.mode = 46;
				break;
			}
			case 46:        { // reverse movement: main part
				MOTOR_Z_SetPulse();
				if( --z_axis.current_pos > z_axis.ramp_step ) {
				} else {
					z_axis.mode = 47;
				}
				break;
			}
			case 47:        { // reverse movement: ramp down: deccel part + stop
				if (z_axis.current_pos > 0) {
					MOTOR_Z_SetPulse();
					--z_axis.current_pos;
				}
				if(ramp_down()) {
					at_move_end();
				}
				break;
			}
			case 48:        { // reverse movement: main part with prolong activated. todo split it with 46 mode?
				MOTOR_Z_SetPulse();
				--z_axis.current_pos;
				if(z_axis.current_pos == z_axis.ramp_step) { // we reach end of main path and have long_pressed key, so just add additional thread full turn to shift initial start point
					z_axis.prolong_fract += z_axis.prolong_addSteps; // fract part from prev step
					uint32_t prolong_fixpart = z_axis.prolong_fract >> 24;
					z_axis.current_pos += prolong_fixpart; // add fixed part
					z_axis.end_pos += prolong_fixpart;
					z_axis.prolong_fract &= FIXEDPT_FMASK; // leave fract part to accumulate with next dividing cycle
					// when long_press end, get back to 46 mode to proceed
				}
				break;
			}

			case 50:        { // direct movement: set direction for motor
				if(feed_direction)
					MOTOR_Z_Forward();
				else
					MOTOR_Z_Reverse();
				z_axis.mode = 54; // intermediate state to wait tacho pulse
				disable_encoder_ticks(); // reset interrupt for encoder ticks, only tacho
				break;
			}

			case 55:        { // direct movement: ramp up: accel by ramp map
				MOTOR_Z_SetPulse();
				z_axis.current_pos++;
				if(ramp_up()) {
					LED_GPIO_Port->BSRR = LED_Pin;   // led off
					z_axis.mode = 56;
				}
				break;
			}
			case 56:        { // direct movement: main part
				MOTOR_Z_SetPulse();
				z_axis.current_pos++;
				if( z_axis.current_pos < ( z_axis.end_pos - z_axis.ramp_step ) ) {
					move();
				} else {
					z_axis.mode = 30;
				}
				break;
			}
			}
		}
	}
// tacho event found!
	if( tacho ) {
//  if( TIM4->SR & TIM_SR_CC3IF ) {
		tacho_debug = 0;
		if (dir == Spindle_Direction_CW ) {
//                        count2++;
		} else {
//                        count2--;
		}
		switch(z_axis.mode) {
		case 35:
			MOTOR_Z_Disable(); //disable motor
			break;

		case 24: {
			z_axis.mode = 25;
//                          TIM4->ARR = fixedpt_toint(Q824set) - 1;
			infeed_step = 0;
			LED_GPIO_Port->BRR = LED_Pin; //led on
			TIM4->ARR = 1; // start stepper motor ramp up procedure immediately after tacho event
			TIM4->CNT = 0;
			enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
			// set ARR register to integer part of Q8.24:
			// for example for 1.5mm thread cutting of 400step stepper for 1800 line encoder
			// we have to step every (5-1) = 4 encoder tick, so set ARR = 4
//                          fract_part = fixedpt_fracpart( Q824set ); // save fract part
			break;
		}
		case 54: {
			z_axis.mode = 55;
			//reinit counter
//                          TIM4->ARR = fixedpt_toint(Q824set) - 1;
			LED_GPIO_Port->BRR = LED_Pin; //led on

			if(infeed_step < infeed_steps) {
				TIM4->ARR = infeed_map[infeed_step++] + 1; // start stepper motor with shifted position by infeed map
			} else {
				TIM4->ARR = 1;  // start stepper motor ramp up procedure immediately after tacho event
			}

			TIM4->CNT = 0;
			enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
//                          fract_part = fixedpt_fracpart( Q824set ); // save fract part
			break;
		}
		default:
			break;
		}
	}

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles I2C2 event interrupt.
*/
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
* @brief This function handles I2C2 error interrupt.
*/
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */
_Bool ramp_up(void)
{
	const fixedptu  set_with_fract = ramp[z_axis.ramp_step];
	if(z_axis.Q824set > set_with_fract || z_axis.ramp_step == ramp_map ) { // reach desired speed or end of ramp map
		TIM4->ARR = fixedpt_toint(z_axis.Q824set) - 1; // update register ARR
		z_axis.fract_part = fixedpt_fracpart(z_axis.Q824set); // save fract part for future use on next step
		return true;
	} else {
		z_axis.ramp_step++;
		TIM4->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
//		z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	}
	return false;
}

inline _Bool ramp_down(void)
{
	if (z_axis.ramp_step == 0)
		return true;
	const fixedptu set_with_fract = ramp[--z_axis.ramp_step];
	TIM4->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
//	z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	if(z_axis.ramp_step == 0)
		return true;
	return false;
}

void move(void)
{
	const fixedptu set_with_fract = fixedpt_add(z_axis.Q824set, z_axis.fract_part); // calculate new step delay with fract from previous step
	TIM4->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
	z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
}

void at_move_end(void)
{
	disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
	//      MOTOR_Z_Disable(); //disable motor later on next tacho event (or after some ticks count?) to completely process last step
	if(auto_mode == true)    auto_mode_delay = auto_mode_delay_ms; // reengage auto mode
	feed_direction = !feed_direction; //change feed direction
	menu_changed = 1; //update menu
	z_axis.mode = 35; // dummy mode
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
