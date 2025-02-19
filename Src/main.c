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
#include "..\buildno.txt"


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

	int debug_cmd = 0, debugnext = 0;

__IO uint8_t ubI2C_slave_addr = 0;
__IO uint8_t ubMasterRequestDirection  = 0;
char cmd1 = 0;
char cmd2 = 0;

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

__IO uint32_t rised = 0;
#define RX_BUFFER_SIZE   64
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBuffer[RX_BUFFER_SIZE];
__IO uint8_t	uwNbReceivedChars;
__IO uint8_t	uNbReceivedCharsForUser;
//__IO uint32_t	uwBufferReadyIndication;


__IO uint8_t initSync = 0;


//uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;
__IO uint8_t ubUART3ReceptionComplete = 0;

__IO uint8_t ubLIMITleftZ = 0;

/* Buffer used for transmission */
const uint8_t aTxtest[] = "A12345678901234567890BCD\r\n";//crc ok, add to queue ok, continue
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
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */


#ifndef _USEENCODER
void spindle_emulator(void);
#endif



void jog_pulse(int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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



bool dbg_ssh = false;

int dbg_ste = 0;
int dbg_gskip = 0;
int dbg_ss_cnt = 0;
void trim_substep_short(){
	// 1. check if there is any active task is running:
	if(state_hw.current_task_ref) {
		if(state_hw.current_task_ref->steps_to_end > 10){
			state_hw.current_task_ref->steps_to_end = 10;
		}
		// reset task cb:
		cb_reset(&task_cb);
	}

	// check if any task is precalculating:
	if(state_precalc.current_task_ref) {
		//stop precalculating process, reset tasks and substep cb:
		state_precalc.current_task_ref = 0;
		cb_reset(&task_cb);
		if(state_hw.current_task_ref == 0) //drop substeps only if there is no active running task
			cb_reset(&substep_cb);
	}
}
/*
void trim_substep(){
	uint32_t skip = 50;
	substep_t *tmp_head = (substep_t *)substep_cb.tail;
	if(state_precalc.current_task_ref &&  state_precalc.current_task_ref->unlocked == false){
		//stop precalc
		tmp_head->skip = 0;
		state_precalc.current_task_ref->steps_to_end = 0;
		cb_pop_front_ref2(&task_cb);
		cb_pop_front_ref(&task_cb);
		substep_cb.count = substep_cb.count2 = 0;
		substep_cb.head = substep_cb.tail;
	  dbg_ste =  1;
		steps_to_end_shadow = 0;
		state_precalc.current_task_ref = 0;

//				state_precalc.current_task_ref->unlocked == true;
		return;
	}
	if(state_hw.current_task_ref->steps_to_end > 50){
		state_hw.current_task_ref->steps_to_end = 50;
	} else {
		skip = state_hw.current_task_ref->steps_to_end;
	}
	int local_Z = state_hw.global_Z_pos;
	int local_X = state_hw.global_X_pos;
//	init_gp.Z = state_hw.global_Z_pos;
//	init_gp.X = state_hw.global_X_pos;
	
	int32_t ss_cnt = 0, ss_count = 0, substep_cnt = 0;

	while(ss_cnt < skip){
	//	ss[d_cnt++]=tmp_head->skip;
		ss_cnt += tmp_head->skip;
		if(tmp_head->delay > 0){
			substep_cnt++;
			ss_cnt++;
		}
		if(ss_cnt < skip){
			ss_count++;
			if(++tmp_head == substep_cb.buffer_end)
				tmp_head = substep_cb.buffer;
		}
	}
	if(ss_cnt == 0){
	 int tt =0;
	}
	dbg_ss_cnt = ss_cnt;
	if(ss_cnt > skip ){
		tmp_head->skip = skip - (ss_cnt - tmp_head->skip);
		dbg_gskip = tmp_head->skip;
	}

	substep_cb.count = substep_cb.count2 = ss_count;
	substep_cb.head = tmp_head;

	if(state_hw.substep_axis == SUBSTEP_AXIS_X){
		if(local_Z > 0){
			local_Z += skip;
		} else if(local_Z < 0)  {
			local_Z -= skip;
		}
		local_X += state_hw.current_task_ref->x_direction == xdir_backward ? substep_cnt : -substep_cnt;

	} else {
		if(local_X > 0){
			local_X += skip;
		} else if(local_X < 0)  {
			local_X -= skip;
		}
		local_Z += state_hw.current_task_ref->z_direction == xdir_forward ? substep_cnt : -substep_cnt;
	}
	init_gp.Z = fixedpt_fromint2210(local_Z);
	init_gp.X = fixedpt_fromint2210(local_X);
}
*/
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
		if(uwNbReceivedChars == 1)
			Error_Handler2(2);
		uNbReceivedCharsForUser = uwNbReceivedChars;
		memset(&aRXBuffer,0,uwNbReceivedChars);
		if(aRXBuffer[uwNbReceivedChars-1] == '\r')
			memcpy(&aRXBuffer,&aRXBufferA,uwNbReceivedChars-1);
		else
			memcpy(&aRXBuffer,&aRXBufferA,uwNbReceivedChars);
		memset(&aRXBufferA,0,uwNbReceivedChars);
    uwNbReceivedChars = 0;
	} else {
		if(symbol != 0)
			pBufferReadyForReception[uwNbReceivedChars++] = symbol;
		if(uwNbReceivedChars == RX_BUFFER_SIZE-1){
			Error_Handler2(1);
		}
	}
}
int aa;
substep_t *tmp_headg;
uint8_t ss[100];
						int d_cnt = 0;
bool rised2;
char str1[64];
char str2[64];
float fft = 3.14f;


 //#define _USEENCODER
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//state_hw.global_Z_pos = -1;
//	ui64toa(state_hw.global_Z_pos, &str1[0]);//generte number with base 64
//	ui16toa(&state_hw.global_Z_pos,&str1[0],4);
//	strcat(str1, "test");
//	info[11] = ';';
//	aa = sizeof(G_task_t);

	#define LOOP_FROM 1
//#define LOOP_COUNT 2
//	#define LOOP_COUNT 4 //509//289 //158
//!!!! 	_USEENCODER define  moved to gcode.h
	
	#ifdef _USEENCODER
	int preload = 2;//LOOP_COUNT;
	#else
	int preload = 4;//LOOP_COUNT;
	#endif	

const char * ga1[] = { 
	#ifdef _USEENCODER
	"G91",
	"G95",
//	"G33 Z-10 K4.5 M3",
	"G0 X0. Z0.",
	"G0 Z10",
	"G94",
	"G1 Z-600 F600", //F900",
	"G0 Z200",
	
//	"G1 Z-2 F0.05",
	#else
	"G91",
	"G94",
//	"G76 P0.05 Z-1 I-.075 J0.008 K0.045 Q29.5 L2 E0.045",
//	"G76 P2. Z-50. I-8.209 J0.25 K2. R2. Q29.5 H0. E1. L0",

	"G0 X0. Z0.",
//	"G1 Z200 F0.1",
//	"G0 X10. Z0.",
	"G1 Z-10 F600",
//	"G1 Z-200 F0.5",
	"G0 X0. Z0.",
	"G1 Z-10 F600",
	"G0 X0. Z0.",
	"G1 Z-10 F600",
	"G0 X0. Z0.",
	"G1 Z-10 F600",

	"G1 Z3",
	"G1 Z-4",
	"G1 Z5",	
	"G1 Z0",
	#endif	
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

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */
	memset(&state_hw,0,sizeof(state_hw));
	
	state_hw.retract = 100;
	G95(&state_hw);
	switch_to_sync(&state_hw); //  set sync mode as default

  state_hw.uart_header[0] = state_hw.uart_header[1] =state_hw.uart_header[2] =state_hw.uart_header[3] = '!';
	state_hw.uart_end[0] = '#';
	state_hw.uart_end[1] = '#';
	state_hw.uart_end[2] = '\r';
	state_hw.uart_end[3] = '\n';
	char info[14];
//	memcpy(info,"1.80;",5);
//	state_hw.global_Z_pos = (2<<23) - 1;

//ui64toa(state_hw.global_Z_pos,&info[0]);

//	state_hw.function = do_fsm_menu_lps;
	
	cb_init_ref(&task_cb, task_size, sizeof(G_task_t), &gt);
//	cb_init_ref(&task_precalc_cb, task_precalc_size, sizeof(G_task_t), &gt_precalc);
//	cb_init_ref(&gp_cb, gp_size, sizeof(G_pipeline_t),&gp);
	cb_init_ref(&substep_cb, substep_size, sizeof(substep_t),&substep_delay);
//	cb_init_ref(&substep_job_cb, substep_job_size, sizeof(substep_job_t),&substep_delay);

	cb_init_ref(&sma_cb,  8, sizeof(substep_sma_ref_t), &smaNumbers);
	//cb_init_ref(&sma_substep_cb,  8, sizeof(uint32_t), &smaSubstepRefs);

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
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
  LL_USART_EnableDMAReq_TX(USART1);
  /* Enable DMA Channel Tx */
//  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
  /* Clear Overrun flag, in case characters have already been sent to USART */
//  LL_USART_ClearFlag_ORE(USART3);

/* Enable RXNE and Error interrupts */
//  LL_USART_EnableIT_RXNE(USART3);
//  LL_USART_EnableIT_ERROR(USART3);
//	LL_USART_ClearFlag_TC(USART3);

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
	G_task_t record_task = {0};
	int32_t record_X = 0, record_Z = 0;
//	G_task_t *precalculating_task = 0;
	int command = 0;
	bool move_to_saved_pos = false;

// some init hardware state
	state_hw.substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));
	state_hw.substep_pulse_on = 1;
	state_hw.substep_pulse_off = 0;
	state_hw.substep_axis = SUBSTEP_AXIS_X;
	
//	state_hw.uart_header 
	uint32_t bn = Build_No;
	sendDefaultResponseDMA('R',&bn);
	LL_mDelay(1);

//	sendDefaultResponseDMA('Z',&state_hw.global_Z_pos);
//	sendResponse((uint32_t)&state_hw,SYNC_BYTES);

//	sendResponse((uint32_t)aTxtest,SYNC_BYTES);
	// debug serial ping-pong
	LED_OFF();
//	for(int a = 0;a<24;a++){
//		state_hw.mid[a]='A'+a;
//	}

	sendDefaultResponseXZ(); //							sendDefaultResponseDMA('X',&state_hw.global_X_pos);

	/*
	while (1) {
		if(ubUART3ReceptionComplete == 1){
			uint8_t cmd = aRXBuffer[0];
			ubUART3ReceptionComplete = 0;
			if(cmd == '!'){
				switch(aRXBuffer[1]){
					case '0': { // reqest info{  '!0'
						sendDefaultResponseDMA();
						state_hw.global_X_pos++;
						state_hw.global_Z_pos--;
						LED_SWITCH();
					}
				}
			}
		}
	}
	*/
	LED_OFF();
	while (1) {
		// recalc substep delays
		if(substep_cb.count < substep_cb.capacity  && state_precalc.current_task_ref){
			// get pointer to last processed task
			if(state_precalc.current_task_ref->precalculate_callback_ref) {
				state_precalc.current_task_ref->precalculate_callback_ref(&state_precalc);
				if(state_precalc.current_task_ref->unlocked  == true && steps_to_end_shadow == 0) // precalc finished, load next task to precalc
					state_precalc.current_task_ref = 0;
				}
		} else {
			if(state_precalc.current_task_ref == 0 && task_cb.count2 > 0){
				state_precalc.current_task_ref = cb_pop_front_ref2(&task_cb); // get ref to task to start precalculating process
				if(state_precalc.current_task_ref) {
					// init precalc:
					if(state_precalc.current_task_ref->precalculate_init_callback_ref){
						state_precalc.current_task_ref->precalculate_init_callback_ref(&state_precalc);
						if(state_precalc.current_task_ref->precalculate_callback_ref)
								state_precalc.current_task_ref->precalculate_callback_ref(&state_precalc);
						if(state_precalc.current_task_ref->unlocked  == true) // precalc finished, load next task to precalc
							state_precalc.current_task_ref = 0;
					}
				}
			} else if (substep_cb.count2 == substep_cb.capacity && state_precalc.current_task_ref ){ // buffer overflow, unlock task to process precalculated data
					state_precalc.current_task_ref->unlocked = true;
//				if(substep_cb.count2 < substep_cb.capacity && state_precalc.current_task_ref->precalculate_callback_ref)
//					state_precalc.current_task_ref->precalculate_callback_ref(&state_precalc);

			} 
			// if buffer is full go to sleep
//			__WFI();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		__WFI();
		load_next_task(&state_hw);
		LL_IWDG_ReloadCounter(IWDG);
		if(initSync == 1){
			initSync = 0;
			sendDefaultResponseXZ(); //					sendDefaultResponseDMA('Z',&state_hw.global_Z_pos);
		}
//		ubUART3ReceptionComplete = 1;
//		memcpy(aRXBuffer,"!test",4);
    
		// simple processing of A7 button trigger event from EXTI:
		if(ubLIMITleftZ == 1) {
			ubLIMITleftZ = 0;
			while(state_hw.rised!=0);
			trim_substep_short();
//			EOM(); //					sendDefaultResponseDMA('Z',&state_hw.global_Z_pos);
		}
		if(ubUART3ReceptionComplete == 1){
			uint8_t cmd = aRXBuffer[0];
			cmd1 = aRXBuffer[0];
			cmd2 = aRXBuffer[1];
			ubUART3ReceptionComplete = 0;
			if(cmd == '!'){
				switch(aRXBuffer[1]){
					case '=':
						switch(aRXBuffer[2]){
							case 'X':
							  state_hw.global_X_pos = ahextoui32(&aRXBuffer[3]);
							break;
							case 'Y':
							  state_hw.global_Z_pos = ahextoui32(&aRXBuffer[3]);
							break;
						}
						break;
					case '1':
//						if( abs(state_hw.global_X_pos - record_X) > abs(state_hw.retract) || abs(state_hw.global_Z_pos - record_Z) > abs(state_hw.retract) ){
							if(state_hw.G90G91 == G90mode){
								scheduleG00G01move(	fixedpt_fromint2210(state_hw.initial_task_X_pos), fixedpt_fromint2210(state_hw.initial_task_Z_pos), 0, G00code);
							} else {
								scheduleG00G01move(	fixedptu_fromint2210(record_X) - init_gp.X, fixedptu_fromint2210(record_Z) - init_gp.Z, 0, G00code);
							}
//							move_to_saved_pos = true;
//						}
						break;
					case '2': // save last comand  '!2'
						while(state_hw.rised!=0);
						LED_ON();
//						move_to_saved_pos = false;
						// break current task, set new task length as steps_to_end-dz and save this task as new record to repeat in cycle
//						__disable_irq();
						memcpy(&record_task, state_hw.last_loaded_task_ref, task_cb.sz);
//						if(dbg_ssh)
							trim_substep_short();
//						else 
//							trim_substep();
						// due to predict where machine will end its movement currently work not good do next trick:
						//wait until active task is finished to update init_gp position with actual global_Z(X)_pos:
						while(state_hw.current_task_ref != 0){
						}
 
						record_task.unlocked = false; // lock task to recalculate it again
						record_X = state_hw.initial_task_X_pos;
						record_Z = state_hw.initial_task_Z_pos;

						int local_Z = fixedpt_toint2210(init_gp.Z);
						int local_X = fixedpt_toint2210(init_gp.X);

						int back_dz = record_Z - local_Z;
						int back_dx = record_X - local_X;
						record_task.dz = abs(back_dz);
						record_task.dx = abs(back_dx);
//						record_task.F = state_hw.Q824set; // get from hw if it feed was changed on the fly

					// move back:
					// 1. move back 400 steps from blank(or move forward if ID)
						scheduleG00G01move(fixedpt_fromint2210(state_hw.retract), 0, 0, G00code);
					// repeat same movement in back direction
						scheduleG00G01move(fixedpt_fromint2210(back_dx), fixedpt_fromint2210(back_dz), 0, G00code);
          // move forward 400 steps to blank
						scheduleG00G01move(-fixedpt_fromint2210(state_hw.retract), 0, 0, G00code);
//						EOM();
//						sendResponse((uint32_t)"ok\r\n",4);
						break;
					case '3': //repeat last command '!3'
						// команда 
/*						if( abs(state_hw.global_X_pos - record_X) > abs(state_hw.retract) || abs(state_hw.global_Z_pos - record_Z) > abs(state_hw.retract) ){
							if(state_hw.G90G91 == G90mode){
								scheduleG00G01move(	fixedpt_fromint2210(state_hw.initial_task_X_pos), fixedpt_fromint2210(state_hw.initial_task_Z_pos), 0, G00code);
							} else {
								scheduleG00G01move(	fixedptu_fromint2210(record_X) - init_gp.X, fixedptu_fromint2210(record_Z) - init_gp.Z, 0, G00code);
							}
//							move_to_saved_pos = true;
						} else*/ 
						{
//							move_to_saved_pos = false;
							cb_push_back_item(&task_cb,&record_task);
							init_gp.Z += record_task.z_direction == zdir_backward ? -fixedpt_fromint2210(record_task.dz) : fixedpt_fromint2210(record_task.dz);
							init_gp.X += record_task.x_direction == zdir_backward ? fixedpt_fromint2210(record_task.dx) : -fixedpt_fromint2210(record_task.dx);

							int back_dz = record_task.z_direction ==zdir_forward ? -record_task.dz : record_task.dz;
							int back_dx = record_task.x_direction ==xdir_forward ? record_task.dx : -record_task.dx;

							scheduleG00G01move(fixedpt_fromint2210(state_hw.retract), 0, 0, G00code);
							scheduleG00G01move(fixedpt_fromint2210(back_dx), fixedpt_fromint2210(back_dz), 0, G00code);
							scheduleG00G01move(-state_hw.retract*1024, 0, 0, G00code);
						}
						break;
					case 'R': // infinite loop to reset by WD
						LL_mDelay(10000);
						break;
					case 'X': // stop current move '!X'
					case 'S': // stop current move  '!S'
						while(state_hw.rised!=0){
//							rised++;
						}
						trim_substep_short();
//						EOM();
//						sendResponse((uint32_t)"ok\r\n",4);
						break;
					case '0': { // reqest info{  '!0'
						char info[14];
//						memcpy(info,"2.10;",5); //version
						memset(info,32,5); //fill version block with spaces
						ui10toa(Build_No, (uint8_t *)&info); //copy dynamic build nubmer to info field
						
						ui64toa(state_hw.global_Z_pos,(uint8_t *)&info[5]);
						info[11] = ';';
						info[12] = '\r';
						info[13] = '\n';

						sendResponse((uint32_t)info, 14);
						break;
					}
					
					case 'v':{ // vector of carriage move, 1st argument is direction, 24 values by 15 gegree number packet to single byte, second argument is 1 byte relative speed
						uint8_t direction = aRXBuffer[2] - '0' > 9 ? aRXBuffer[2] - 'A' + 10 : aRXBuffer[2] - '0';
						uint8_t speed = aRXBuffer[3] - '0';
						if(speed>0)
							state_hw.vector = 1;
						break;
					}

					
					case 'B': {// left correction small(Back) '!B' todo refactor
						//switch to manual pulse generation
						uint32_t pin_dir = LL_GPIO_IsOutputPinSet(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
						if(pin_dir != zdir_backward){ // change DIR
							LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
							LL_mDelay(1);
						}
						LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
						for(int a = 0;a<20;a++){
							LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
							LL_mDelay(2);
							LL_GPIO_ResetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
							LL_mDelay(5);
						}
						if(pin_dir != zdir_backward){ // change DIR back
							LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
						}
						LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
						break;
					}
					case 'F': {// right correction small '!F' todo refactor
						//switch to manual pulse generation
						uint32_t pin_dir = LL_GPIO_IsOutputPinSet(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
						if(pin_dir != zdir_forward){ // change DIR
							LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
							LL_mDelay(1);
						}
						LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
						for(int a = 0; a<20; a++){
							LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
							LL_mDelay(2);
							LL_GPIO_ResetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
							LL_mDelay(5);
						}
						if(pin_dir != zdir_backward){ // change DIR back
							LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
						}
						LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
						break;
					}
					// JOG1:
					//X axis
					// X jog is allowed only when no active job on this axis is performed
					case 'w': {// go forward by 1 step '!w'
							if(state_hw.jog_pulse == true)
								break; 
						if(state_hw.task_lock == false || ( state_hw.substep_axis == SUBSTEP_AXIS_X && state_hw.current_task_ref->dx ==0 && !LL_TIM_IsEnabledCounter(TIM1))){
							if(XDIR != xdir_forward){
								XDIR = xdir_forward;
								LL_mDelay(1); // time to switch direction
							}
							state_hw.global_X_pos--;
							record_X--;
							jog_pulse(0);
							sendDefaultResponseXZ(); //							sendDefaultResponseDMA('X',&state_hw.global_X_pos);
						}
						break;
					}
					case 's': {// go backward by 1 step '!s'
						if(state_hw.jog_pulse == true)
							break; 
						if(state_hw.task_lock == false || (state_hw.current_task_ref && state_hw.substep_axis == SUBSTEP_AXIS_X && state_hw.current_task_ref->dx ==0 && !LL_TIM_IsEnabledCounter(TIM1))){
							if(XDIR != xdir_backward){
								XDIR = xdir_backward;
								LL_mDelay(1); // time to switch direction
							}
							state_hw.global_X_pos++;
							record_X++;
							jog_pulse(0);
							sendDefaultResponseXZ(); //							sendDefaultResponseDMA('X',&state_hw.global_X_pos);
						}
						break;
					}
					case 'a': {// left small '!a'
						scheduleG00G01move(0, -jog_Z_01_2210, 0, G00code);
						/*
						int from_Z = init_gp.Z;
					// get current Z position - 0.1mm
						if(state_hw.G90G91 == G90mode) {
							init_gp.Z -= jog_Z_01_2210; // 0,1mm*z_steps_unit/z_screw_pitch and convert to 2210=0,1*400/2*1024=20480 todo need some functions to convert units
							G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, init_gp.X, init_gp.Z,  init_gp.Xr, G00code);
						} else {
							G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, 0, -jog_Z_01_2210,  0, G00code);
							record_Z -= jog_Z_01_2210 / 1024;
						}
						*/
						break;
					}
					case 'd': {// right small '!d'
						scheduleG00G01move(0, jog_Z_01_2210, 0, G00code);
						/*
						int from_Z = init_gp.Z;
					// get current Z position - 0.1mm
						if(state_hw.G90G91 == G90mode) {
							init_gp.Z += jog_Z_01_2210; // 0,1mm*z_steps_unit/z_screw_pitch and convert to 2210=0,1*400/2*1024=20480 todo need some functions to convert units
							G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, init_gp.X, init_gp.Z,  init_gp.Xr, G00code);
						} else {
							G01parsed(init_gp.X, init_gp.Xr, from_Z, init_gp.F, 0, jog_Z_01_2210,  0, G00code);
							record_Z += jog_Z_01_2210 / 1024;
						}
						*/
						break;
					}

					case 'g':{ //decrease feed speed on the fly, resolution 0.1mm (0.1*1000=100)
						int32_t ff = feed_on_the_fly_factor / state_hw.Q824set - 100;
						if(ff < 100) // limit min FOTF by 0.1mm/rev
							ff = 100;
						record_task.F = state_hw.Q824set = feed_on_the_fly_factor / ff;
						sendDefaultResponseDMA('f',&state_hw.Q824set);
//						record_task.F = state_hw.Q824set;
/*record_task.F = = 1474560.0f / state_hw.Q824set;
					  ff -=0.1f;
						if(ff < 0.1)
							ff = 0.1;
						state_hw.Q824set = 1474560 / ff;*/
						break;
					}
					case 't': { //increase feed speed on the fly
						// 1474560 = 65536*encoder_resolution*z_screw_pitch/z_steps_unit = 3600*0,00625*65536 and * 1000 to get integer math
						int32_t ff = feed_on_the_fly_factor / state_hw.Q824set + 100;
						if(ff > 3000) // limit max FOTF by 3mm/rev
							ff = 3000;
						record_task.F = state_hw.Q824set = feed_on_the_fly_factor / ff;
						sendDefaultResponseDMA('f',&state_hw.Q824set);
/*						float ff = 1474560.0 / state_hw.Q824set;
					  ff +=0.1f;
						if(ff > 3)
							ff = 3;
						state_hw.Q824set = 1474560.0 / ff; */
						break;
					}

					
					case 'I': // set ID
						state_hw.ODID = 1;
						state_hw.retract = -100;
						break;
					case 'O': // set OD
						state_hw.ODID = 0;
						state_hw.retract = 100;
						break;
				}
			} else {
//				int charsCount = uNbReceivedCharsForUser - CRC_BASE64_STRLEN;
				uNbReceivedCharsForUser = 0;

//				if(charsCount>0) {
//					if(is_crc_ok(aRXBuffer,charsCount)){
//						sendResponse((uint32_t)aTxCRC_OK_OK_Continue, 5);
//						*(aRXBuffer+charsCount) = 0;
	//					uint8_t cmd = *(char *)aRXBuffer;
				command_parser((char *)aRXBuffer);
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
//					if(!LL_USART_IsActiveFlag_TC(USART1))
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
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
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
	#ifndef _USEENCODER

	return;
	#endif
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
  TIM_InitStruct.Autoreload = 65535;
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
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH4);
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
  PB1   ------> TIM3_CH4
  PB5   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = MOTOR_X_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(MOTOR_X_STEP_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOTOR_Z_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(MOTOR_Z_STEP_GPIO_Port, &GPIO_InitStruct);

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

	#ifndef _USEENCODER
	spindle_emulator();
	return;
#endif	


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
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_DOWN;
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
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 921600; //115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_4|LL_GPIO_PIN_5
                          |LL_GPIO_PIN_6|LL_GPIO_PIN_8|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_2|LL_GPIO_PIN_12|LL_GPIO_PIN_13
                          |LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_9;
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

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE7);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

#ifndef _USEENCODER
void spindle_emulator(void){
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

	
  TIM_InitStruct.Prescaler = 2399;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 50;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 48;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableMasterSlaveMode(TIM4);
}
#endif

volatile int codeg = 0;
void Error_Handler2(int code)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	codeg = code;
	/* User can add his own implementation to report the HAL error return state */
	TIM1->CR1 = TIM2->CR1 = TIM3->CR1 = TIM4->CR1 = 0;
	while (1) {
	}
  /* USER CODE END Error_Handler_ Debug */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler1(int code)
{
  /* USER CODE BEGIN Error_Handler_Debug */
//	codeg = code;
	/* User can add his own implementation to report the HAL error return state */
	TIM1->CR1 = TIM2->CR1 = TIM3->CR1 = TIM4->CR1 = 0;
	while (1) {
	}
  /* USER CODE END Error_Handler_ Debug */
}

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
