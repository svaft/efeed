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
#include "buttons.h"
#include "fsm.h"
#include "i2c_interface.h"
#include "ssd1306.h"
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
	for(int a = 0; a<BT_TOTAL;a++){
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
  if(LL_DMA_IsActiveFlag_TC4(DMA1))
  {
    LL_DMA_ClearFlag_GI4(DMA1);
		
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
		LL_DMA_ClearFlag_TC4(DMA1);
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
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
  if(LL_DMA_IsActiveFlag_TC5(DMA1))
  {
    LL_DMA_ClearFlag_GI5(DMA1);

		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
		LL_DMA_ClearFlag_TC5(DMA1);

    Transfer_Complete_Callback();
  }
  else if(LL_DMA_IsActiveFlag_TE5(DMA1))
  {
    Transfer_Error_Callback();
  }

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	// enable corresponding channel for sub-step:
	TIM1->SR = 0;
	*((volatile unsigned int *)state_hw.substep_pin) = state_hw.substep_pulse_off;
	
	if(state_hw.current_task.steps_to_end == 0){
		state_hw.task_lock = false; // unlock task processor to load next task
		if(task_cb.count == 0){
			do_fsm_move_end2(&state_hw);
//			return;
		}
//		load_next_task(&state_hw);
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

/**
* @brief This function handles TIM3 global interrupt. Прерывание срабатывает в конце пульса, в нем обрабатываем
пересчет для следующего шага, а так же выполняются функции обработки конца задания и инициализации следующего задания
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	TIM3->SR = 0;

	if(state_hw.current_task.callback_ref){
		state_hw.current_task.callback_ref(&state_hw);
	}
	if(state_hw.current_task.steps_to_end == 0 && !LL_TIM_IsEnabledCounter(TIM1)){ // check for tim1 is enabled, if its true - substep is active, so load next task on end of substep
		state_hw.task_lock = false; // unlock task processor to load next task
		if(task_cb.count == 0){
			do_fsm_move_end2(&state_hw);
//			return;
		}
//		load_next_task(&state_hw);
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
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */
  /* Check SB flag value in ISR register */
  if(LL_I2C_IsActiveFlag_SB(I2C2))
  {
    /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a write request */
    LL_I2C_TransmitData8(I2C2, ubI2C_slave_addr | ubMasterRequestDirection);

    /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a ubMasterRequestDirection request */
//    LL_I2C_TransmitData8(I2C2, SLAVE_OWN_ADDRESS | ubMasterRequestDirection);
		
  }
  /* Check ADDR flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_ADDR(I2C2))
  {
    /* Verify the transfer direction */
    if(LL_I2C_GetTransferDirection(I2C2) == LL_I2C_DIRECTION_READ)
    {
      ubMasterXferDirection = LL_I2C_DIRECTION_READ;

      if(ubMasterNbDataToReceive == 1)
      {
        /* Prepare the generation of a Non ACKnowledge condition after next received byte */
        LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);

        /* Enable DMA transmission requests */
        LL_I2C_EnableDMAReq_RX(I2C2);
      }
      else if(ubMasterNbDataToReceive == 2)
      {
        /* Prepare the generation of a Non ACKnowledge condition after next received byte */
        LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);

        /* Enable Pos */
        LL_I2C_EnableBitPOS(I2C2);
      }
      else
      {
        /* Enable Last DMA bit */
        LL_I2C_EnableLastDMA(I2C2);

        /* Enable DMA transmission requests */
        LL_I2C_EnableDMAReq_RX(I2C2);
      }
    } else {
			/* Enable DMA transmission requests */
			LL_I2C_EnableDMAReq_TX(I2C2);
		}
    /* Clear ADDR flag value in ISR register */
    LL_I2C_ClearFlag_ADDR(I2C2);
  }

  /* USER CODE END I2C2_EV_IRQn 0 */
  
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */
  Error_Handler();

  /* USER CODE END I2C2_ER_IRQn 0 */
  
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
  {
    /* RXNE flag will be cleared by reading of DR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART_CharReception_Callback();
  }
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
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
