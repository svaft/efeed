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
#include "fixedptc.h"
#include "nuts_bolts.h"
#include "gcode.h"
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

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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

  /* USER CODE BEGIN SysTick_IRQn 1 */
//      if(auto_mode_delay > 0)
//              auto_mode_delay--;

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

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}
int breakbig =0;
/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	// enable corresponding channel for sub-step:
	TIM1->SR = 0;
	*((volatile unsigned int *)state_hw.substep_pin) = state_hw.substep_pulse_off;
	if(!state_hw.jog_pulse){
		if(state_hw.current_task_ref->steps_to_end == 0 ){
			state_hw.task_lock = false; // unlock task processor to load next task
			if(task_cb.count == 0){
				do_fsm_move_end2(&state_hw);
	//			return;
			}
	//		load_next_task(&state_hw);
		} 
//		else if (state_hw.current_task_ref->steps_to_end > 4294964598) {
//			breakbig = 1;
//		}
	} else {
		state_hw.jog_pulse = false;
	}

  /* USER CODE END TIM1_UP_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
	TIM1->SR = 0;
	*((volatile unsigned int *)state_hw.substep_pin) = state_hw.substep_pulse_on;
  /* USER CODE END TIM1_CC_IRQn 0 */
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
// prescaler=((((speed=72000000)/((period=20000)/(1/hz=1)))+0,5)-1)
	TIM2->SR = 0;
	state_hw.function(&state_hw);

//	TIM2->EGR |= TIM_EGR_UG;
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */
  /* Check whether update interrupt is pending */

//  if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == 1)
//  {
    /* Clear the update interrupt flag*/
//    LL_TIM_ClearFlag_UPDATE(TIM2);
//  }
  /* USER CODE END TIM2_IRQn 1 */
}
int break11 = 0 ;
/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	TIM3->SR = 0;

	if(state_hw.current_task_ref->callback_ref){
		state_hw.current_task_ref->callback_ref(&state_hw);
	}
	if(state_hw.current_task_ref->steps_to_end == 0){
		break11 = 1;
		if(!LL_TIM_IsEnabledCounter(TIM1)){ // check for tim1 is enabled, if its true - substep is active, so load next task on end of substep
			state_hw.task_lock = false; // unlock task processor to load next task
			if(task_cb.count == 0){
				do_fsm_move_end2(&state_hw);
	//			return;
			}
	//		load_next_task(&state_hw);
		} else {
			break11 = 2;
		}
	}

  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

//	__dsb(0);
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	TIM4->SR = 0; // clear SR at the beginning of the interrupt to avoid false call it twice, http://www.keil.com/support/docs/3928.htm 
	state_hw.function(&state_hw);


	state_hw.prescaler = TIM2->CNT;
	TIM2->CNT = 0;
	state_hw.rpm = 2400000 / state_hw.prescaler;

	
	
	
//		__dsb(0);
//	}
	
//	cnt3++;
//	if(TIM2->CNT > 50){
//		cnt2 = TIM2->CNT; // measure current spindle speed to use as prescaler value for substep smoothing
	//	cnt2div = cnt2/TIM4->ARR;
//		TIM2->CNT = 0; // reset timer count to start measure next
//	if(cnt2 > 1500)
//	}
	
//	_Bool dir = t4cr1[TIM_CR1_DIR_Pos];
//	if(t4sr[TIM_SR_CC3IF_Pos]){
//		do_fsm_wait_tacho(&state);
//	}
//	if(TIM3->SMCR == 0x36) { // TIM3 connected to TIM4 as SLAVE

//		state_hw.spindle_dir = t4cr1[TIM_CR1_DIR_Pos];
//		state_hw.f_encoder = encoder;
//		state_hw.f_tacho = t4sr[TIM_SR_CC3IF_Pos];


//	state_hw.function(&state_hw);

	
	
	//		state_hw.syncbase->ARR = state_hw.z_period;
//		TIM4->ARR = state_hw.z_period;
//		state_hw.syncbase->EGR |= TIM_EGR_UG;
		
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */
//	TIM4->SR = 0;
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART_CharReception_Callback();
  } else {
		
//		while(1);
//		Error_Handler();
	}
	if(LL_USART_IsActiveFlag_TC(USART1) && LL_USART_IsEnabledIT_TC(USART1)){
		LL_USART_ClearFlag_TC(USART1);
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
		
//		while(1);
	}

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

//void TIM1_UP_IRQHandler_old(void)
//{
//  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
////	debug7();
//	// enable corresponding channel for sub-step:
//	debug1();
//	TIM3->CCER = state_hw.substep_mask;
//	// start pulse:
//	LL_TIM_EnableCounter(TIM3); 
//	// stop sub-step timer:
//	LL_TIM_DisableCounter(TIM1);
//	LL_TIM_SetCounter(TIM1,0);
//	TIM1->SR = 0;
//	return;
//  /* USER CODE END TIM1_UP_IRQn 0 */
//  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
//  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) == 1)
//  {
//    /* Clear the update interrupt flag*/
//    LL_TIM_ClearFlag_UPDATE(TIM1);
//  }

//  /* USER CODE END TIM1_UP_IRQn 1 */
//}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
