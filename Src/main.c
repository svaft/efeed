/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f1xx_it.h"

#include "gcode.h"
#include "nuts_bolts.h"


//#define ARM_MATH_CM3
//#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
state_t state_hw;
state_t state_precalc;

__IO uint8_t ubI2C_slave_addr = 0;
__IO uint8_t ubMasterRequestDirection  = 0;


//int count;
extern bool demo;
/*
G18 - XZ-plane
G98 - retract to the position that axis was in just before this series of one or more contiguous canned cycles was started
G50 set spindle speed
G28, G28.1 Go to Predefined Position
G54-G59.3 Select Coordinate System
G96, G97 Spindle Control Mode, G97 (RPM Mode)
*/


// ***** Stepper Motor *****


#define RX_BUFFER_SIZE   64
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBuffer[RX_BUFFER_SIZE];
__IO uint8_t	uwNbReceivedChars;
__IO uint8_t	uNbReceivedCharsForUser;
//__IO uint32_t	uwBufferReadyIndication;

//uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;
__IO uint8_t ubUART3ReceptionComplete = 0;

/* Buffer used for transmission */
const uint8_t aTxFW2[] = "1.71;123456;\r\n";//crc ok, add to queue ok, continue
const uint8_t aTxCRC_OK_OK_Continue[] = "ooc\r\n";//crc ok, add to queue ok, continue
const uint8_t aTxCRC_OK_Ok_Wait[] = "oow\r\n";//crc ok, add to queue ok, wait
const uint8_t aTxCRC_OK_Fail_Wait[] = "ofw\r\n";//crc ok, add to queue ok, wait
const uint8_t aTxCRC_Fail[] = "f__\r\n";//crc fail

uint8_t ubNbDataToTransmit = 5; //sizeof(aTxBuffer);
__IO uint8_t ubTransmissionComplete = 0;
__IO uint8_t  ubMasterXferDirection     = 0;
uint8_t       aMasterReceiveBuffer[0xF] = {0};
__IO uint8_t  ubMasterNbDataToReceive   = sizeof(aMasterReceiveBuffer);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void USART_CharReception_Callback(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static void MX_TIM4_InitAsync(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 4000;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 48;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}



/**
  * @brief  Function called from DMA1 IRQ Handler when Rx transfer is completed 
  * @param  None
  * @retval None
  */
//void DMA1_ReceiveComplete_Callback(void)
//{
  // DMA Rx transfer completed:
//  ubUART2ReceptionComplete = 1;
//}

/**
  * @brief  Send Txt information message on USART Tx line (to PC Com port).
  * @param  None
  * @retval None
  */
void PrintInfo(uint8_t *String, uint32_t Size)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_USART_IsActiveFlag_TXE(USART1))
    {
    }

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in DR register */
    LL_USART_TransmitData8(USART1, *pchar++);
  }

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USART1))
  {
  }
}


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
//	uint8_t *ptemp;
  /* Read Received character. RXNE flag is cleared by reading of DR register */
	uint8_t symbol = LL_USART_ReceiveData8(USART1);
	if(symbol == '\n'){
    /* Set Buffer swap indication */
		ubUART3ReceptionComplete = 1;
		uNbReceivedCharsForUser = uwNbReceivedChars;
		memset(&aRXBuffer,0,RX_BUFFER_SIZE);
		memcpy(&aRXBuffer,&aRXBufferA,uwNbReceivedChars-1);
    uwNbReceivedChars = 0;
	} else {
		if(symbol != 0)
			pBufferReadyForReception[uwNbReceivedChars++] = symbol;
	}
}

//#define _USEENCODER
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
						
//	char info[14];
//	memcpy(info,"1.71;",5);
//	ui64toa(state_hw.global_Z_pos,&info[5]);
//	info[11] = ';';

	#define LOOP_FROM 1
//#define LOOP_COUNT 2
	#define LOOP_COUNT 4 //509//289 //158
#define _USEENCODER // uncomment tihs define to use HW rotary encoder on spindle	

	#ifdef _SIMU
	int preload = 2;//LOOP_COUNT;
	#else
	int preload = 1;
	#endif

const char * ga1[] = {
	"G95",
	"G0 X0. Z0.",
	"G1 Z-2 F100",
	"G1 Z2.1 F100",
	"G1 Z0 F100",
	"G0 Z0.",

	
	"G0 X10. Z0.",
	"G0 X9.498",
	"G33 Z-0.665 K2",
	"X10. Z-1.02 K2",
};	


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */
	memset(&state_hw,0,sizeof(state_hw));
//	state_hw.function = do_fsm_menu_lps;
	
	cb_init_ref(&task_cb, task_size, sizeof(G_task_t), &gt);
//	cb_init_ref(&task_precalc_cb, task_precalc_size, sizeof(G_task_t), &gt_precalc);
//	cb_init_ref(&gp_cb, gp_size, sizeof(G_pipeline_t),&gp);
	cb_init_ref(&substep_cb, substep_size, sizeof(substep_t),&substep_delay);
//	cb_init_ref(&substep_job_cb, substep_job_size, sizeof(substep_job_t),&substep_delay);

	cb_init_ref(&sma_cb,  8, sizeof(substep_sma_ref_t), &smaNumbers);
	cb_init_ref(&sma_substep_cb,  8, sizeof(uint32_t), &smaSubstepRefs);

// init USART
  pBufferReadyForReception = aRXBufferA;
  uwNbReceivedChars = 0;
  ubUART3ReceptionComplete = 0;
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//	LL_TIM_OC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_LOW);
//	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH1);
	
//	while(1){
//		LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
//		LL_mDelay(100);
//	}
	
//	LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
//	LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port, MOTOR_Z_STEP_Pin); 
//	LL_mDelay(50);
//	LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
//	LL_mDelay(50);


  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USART1);

/* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
	LL_USART_ClearFlag_TC(USART1);


/* Enable DMA TX Interrupt */
//  LL_USART_EnableDMAReq_TX(USART3);
  /* Enable DMA Channel Tx */
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USART3);

/* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART3);
  LL_USART_EnableIT_ERROR(USART3);
	LL_USART_ClearFlag_TC(USART3);

//  LL_TIM_EnableIT_CC3(TIM4);													// enable interrupts for TACHO events from encoder
//  LL_TIM_EnableCounter(TIM4); 												//Enable timer 4
//	TIM4->SR = 0; 																			// reset interrup flags

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//	debug();
// prevent to put TIM3 stepper chanels high on set corresponding CCER-CCxE bit.
// without this code channel is enabled on set CCER-CCxE ignoring shadow register, thats strange...	
/*
	LL_TIM_DisableIT_UPDATE(TIM3);
	LL_TIM_GenerateEvent_UPDATE(TIM3);
	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3); // */
//	LL_mDelay(5);
//		LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_0,LL_GPIO_MODE_OUTPUT);


	// prepare TIM1 for sub-step:	
	LL_TIM_ClearFlag_UPDATE(TIM1);
	LL_TIM_CC_DisablePreload(TIM1);
	LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH1);

	LL_TIM_EnableIT_CC1(TIM1);
	LL_TIM_SetOnePulseMode(TIM1,LL_TIM_ONEPULSEMODE_SINGLE);

	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
//	TIM1->CCR1 = 50;
	LL_TIM_EnableIT_UPDATE(TIM1);
	LL_TIM_DisableARRPreload(TIM1);


// calibration test
/*
	LL_TIM_DisableARRPreload(TIM4);
	TIM4->ARR = 1;

	G95(&state_hw);

	state_t *s = &state_hw;
	LL_TIM_ClearFlag_UPDATE(s->syncbase);
	LL_TIM_EnableUpdateEvent(s->syncbase);
	LL_TIM_EnableCounter(s->syncbase);
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_EnableIT_UPDATE(s->syncbase);
//	LL_TIM_DisableIT_UPDATE(TIM2);
//	LL_TIM_GenerateEvent_UPDATE(s->syncbase);

	while(1);
*/

// todo need refactor this code how to g-code parsing, precalculation and execution going to start and work together
	G_task_t record_task, homing_task;

	G_task_t *precalculating_task = 0;
	int command = 0;
	int testcommand = 0;
	LED_OFF();
	while (1) {
		// recalc substep delays
		if(substep_cb.count < substep_cb.capacity && precalculating_task){
			// get pointer to last processed task
			if(precalculating_task->precalculate_callback_ref) {
				precalculating_task->precalculate_callback_ref(&state_precalc);
				if(state_precalc.precalculating_task_ref->unlocked  == true){ // precalc finished, load next task to precalc
					precalculating_task = cb_pop_front_ref2(&task_cb);
					state_precalc.precalculating_task_ref = precalculating_task;
					memcpy(&state_precalc.current_task, precalculating_task, task_cb.sz);

					if(precalculating_task && precalculating_task->precalculate_init_callback_ref)
						precalculating_task->precalculate_init_callback_ref(&state_precalc);
				// load next task
				}
			} else {
				precalculating_task = cb_pop_front_ref2(&task_cb);
				if(precalculating_task) {
					state_precalc.precalculating_task_ref = precalculating_task;
					memcpy(&state_precalc.current_task, precalculating_task, task_cb.sz);
					if(precalculating_task->precalculate_init_callback_ref)
						precalculating_task->precalculate_init_callback_ref(&state_precalc);
				}
			}
			// call task recalculate callback until task is fully precalculated
			// get next task and repeat unitl all task recalculated or precalculater buffer is full
		} else {
			if(precalculating_task == 0 && task_cb.count2 > 0){
				precalculating_task = cb_pop_front_ref2(&task_cb); // get ref to task to start precalculating process
				if(precalculating_task) {
					memcpy(&state_precalc.current_task, precalculating_task, task_cb.sz);
					state_precalc.precalculating_task_ref = precalculating_task;
					// init precalc:
					if(precalculating_task->precalculate_init_callback_ref){
						precalculating_task->precalculate_init_callback_ref(&state_precalc);
						if(precalculating_task->precalculate_callback_ref) {
							for(int a = 0; a<3;a++)
								precalculating_task->precalculate_callback_ref(&state_precalc);
						}
					}
				}
			}
			// if buffer is full go to sleep
//			__WFI();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		__WFI();
		load_next_task(&state_hw);
		LL_IWDG_ReloadCounter(IWDG);

//		ubUART3ReceptionComplete = 1;
//		memcpy(aRXBuffer,"!test",4);

		if(ubUART3ReceptionComplete == 1){
			uint8_t cmd = aRXBuffer[0];
			ubUART3ReceptionComplete = 0;
			if(cmd == '!'){
				switch(aRXBuffer[1]){
					case '1': //wtf? start record
						break;
					case '2': // save last comand
						// break current task, set new task length as steps_to_end-dz and save this task as new record to repeat in cycle
						__disable_irq();
						memcpy(&record_task, &state_hw.current_task, task_cb.sz);
						record_task.dz -= (record_task.steps_to_end - 50);
						record_task.steps_to_end = record_task.dz;
						if(init_gp.Z > 0){
							init_gp.Z = fixedpt_fromint2210(record_task.dz);
						} else {
							init_gp.Z = -fixedpt_fromint2210(record_task.dz);
						}
						state_hw.current_task.steps_to_end = 50;

						substep_cb.count = substep_cb.count2 = 0;
						substep_t *substep = (substep_t *)substep_cb.tail;
						substep->skip = 50; //need some steps to slow down and stop
						__enable_irq();
						break;
					case '3': //repeat last command
						if(init_gp.Z != 0){
							command_parser("G0 Z0. X0.");
						} else {
							cb_push_back_item(&task_cb,&record_task);
							init_gp.Z = record_task.z_direction == zdir_backward ? -fixedpt_fromint2210(record_task.dz) : fixedpt_fromint2210(record_task.dz); 
						}
						break;
					case '4': // fast feed to start position
						command_parser("G0 Z0. X0.");
//						cb_push_back_item(&task_cb,&homing_task);
						break;
					case '5': // reset record
						break;

					case '6': // pause
						break;
					case '7': // continue
						break;
					case '0': { // reqest info{ 
						char info[14];
						memcpy(info,"1.77;",5); //version
						ui64toa(state_hw.global_Z_pos,&info[5]);
						info[11] = ';';
						info[12] = '\r';
						info[13] = '\n';
						
						sendResponce((uint32_t)info, 14);
						break;
					}

					// JOG1:
					case 'l': {// left small
						int from_Z = init_gp.Z;
					// get current Z position - 0.1mm
						init_gp.Z -= jog_Z_01_2210; // 0,1mm*z_steps_unit/z_screw_pitch and convert to 2210=0,1*400/2*1024=20480 todo need some functions to convert units
						G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, init_gp.X, init_gp.Z,  init_gp.Xr, G00code);
						break;
					}
					case 'L': {// left big	
						
						int from_Z = init_gp.Z;
					// get current Z position - 0.1mm
						init_gp.Z -= jog_Z_10_2210; // 0,1mm*z_steps_unit/z_screw_pitch and convert to 2210=0,1*400/2*1024=20480 todo need some functions to convert units
						G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, init_gp.X, init_gp.Z,  init_gp.Xr, G00code);
						break;
					}					
					case 'r': {// right small
						int from_Z = init_gp.Z;
					// get current Z position - 0.1mm
						init_gp.Z += jog_Z_01_2210; // 0,1mm*z_steps_unit/z_screw_pitch and convert to 2210=0,1*400/2*1024=20480 todo need some functions to convert units
						G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, init_gp.X, init_gp.Z,  init_gp.Xr, G00code);
						break;
					}
					case 'R': {// right small
						int from_Z = init_gp.Z;
					// get current Z position - 0.1mm
						init_gp.Z += jog_Z_10_2210; // 0,1mm*z_steps_unit/z_screw_pitch and convert to 2210=0,1*400/2*1024=20480 todo need some functions to convert units
						G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, init_gp.X, init_gp.Z,  init_gp.Xr, G00code);
						break;
					}
				}
			} else {
//				int charsCount = uNbReceivedCharsForUser - CRC_BASE64_STRLEN;
				uNbReceivedCharsForUser = 0;

//				if(charsCount>0) {
//					if(is_crc_ok(aRXBuffer,charsCount)){
						sendResponce((uint32_t)aTxCRC_OK_OK_Continue, 5);
//						*(aRXBuffer+charsCount) = 0;
	//					uint8_t cmd = *(char *)aRXBuffer;
						command_parser((char *)aRXBuffer);
						
//					} else {
//						sendResponce((uint32_t)aTxCRC_Fail, 5);
//					}
//				}
			}


		}
			

//		process_joystick();
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);


//		uint8_t level = Thread_Info[Menu_Step].level;

//		if(auto_mode == true) {
//			if ( auto_mode_delay == 0 ) {
//				buttons_flag_set = single_click_Msk; //
//			}
//		}

		if(preload > 0) {
//			if(task_cb.count2 < (task_cb.capacity - 1)) {
				if(task_cb.count < (task_cb.capacity - 2) &&  preload-- >= 0){
					command_parser((char *)ga1[command++]);
				}
			}
		
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
	return;
  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  LL_IWDG_Enable(IWDG);
  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32);
  LL_IWDG_SetReloadCounter(IWDG, 4095);
  while (LL_IWDG_IsReady(IWDG) != 1)
  {
  }

  LL_IWDG_ReloadCounter(IWDG);
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);
  NVIC_SetPriority(TIM1_CC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_CC_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 15;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 2399;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 50;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 48;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = min_pulse;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetOnePulseMode(TIM3, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1);
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

	LL_TIM_DisableIT_UPDATE(TIM3);
	LL_TIM_GenerateEvent_UPDATE(TIM3);
	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3); // */

	MOTOR_X_AllowPulse();
	MOTOR_Z_AllowPulse();
//	LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);

/*
// prevent to put TIM3 stepper chanels high on set corresponding CCER-CCxE bit.
// without this code channel is enabled on set CCER-CCxE ignoring shadow register, thats strange...	
	LL_TIM_DisableIT_UPDATE(TIM3);
	LL_TIM_GenerateEvent_UPDATE(TIM3);
	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3); // */

//	TIM3->CCMR1 = 0x48;


  /* USER CODE END TIM3_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration  
  PB0   ------> TIM3_CH3
  PB5   ------> TIM3_CH2 
  */
  GPIO_InitStruct.Pin = MOTOR_X_STEP_Pin|MOTOR_Z_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_RemapPartial_TIM3();

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration  
  PB6   ------> TIM4_CH1
  PB7   ------> TIM4_CH2
  PB8   ------> TIM4_CH3 
  */
  GPIO_InitStruct.Pin = ENC_A_Pin|ENC_B_Pin|ENC_ZERO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetEncoderMode(TIM4, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_FALLING);
  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableMasterSlaveMode(TIM4);
  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_RISING);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 1));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, MOTOR_X_ENABLE_Pin|MOTOR_X_DIR_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, MOTOR_Z_ENABLE_Pin|MOTOR_Z_DIR_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3 
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7 
                          |LL_GPIO_PIN_8|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_10|LL_GPIO_PIN_11 
                          |LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15 
                          |LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MOTOR_X_ENABLE_Pin|MOTOR_X_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MOTOR_Z_ENABLE_Pin|MOTOR_Z_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*timIC)
{
	if (timIC->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {

	}

//	if(ready!=SET)return;
//HAL_TIM_ReadCapturedValue(timIC,TIM_CHANNEL_3);
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	TIM1->CR1 = TIM2->CR1 = TIM3->CR1 = TIM4->CR1 = 0;
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
