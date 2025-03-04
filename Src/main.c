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



#define jogQueue_size 50
circular_buffer jogQueue_cb;
char jogQueue[jogQueue_size];




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







#define uartBuffer_size 400
circular_buffer uart_cb;
ubuf_t uartBuffer[uartBuffer_size];


__IO uint32_t rised = 0;
#define RX_BUFFER_SIZE   32
uint8_t aRXBufferA[RX_BUFFER_SIZE];
__IO uint8_t	uwNbReceivedChars;
__IO uint8_t	uNbReceivedCharsForUser;

__IO uint8_t initSync = 0;
uint8_t *pBufferReadyForReception;
__IO uint8_t ubUART3ReceptionComplete = 0;

__IO uint8_t ubLIMITleftZ = 0;


uint8_t ubNbDataToTransmit = 5; //sizeof(aTxBuffer);
__IO uint8_t ubTransmissionComplete = 0;
__IO uint8_t  ubMasterXferDirection     = 0;
uint8_t       aMasterReceiveBuffer[0xF] = {0};
__IO uint8_t  ubMasterNbDataToReceive   = sizeof(aMasterReceiveBuffer);


extern __IO bool xPulse;
extern __IO bool zPulse;

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

#define rampdown 10

bool dbg_ssh = false;

int dbg_ste = 0;
int dbg_gskip = 0;
int dbg_ss_cnt = 0;
void trim_substep_short(){
	// 1. check if there is any active task is running:
	if(state_hw.current_task_ref) {
		if(state_hw.current_task_ref->steps_to_endZ > rampdown){
			state_hw.current_task_ref->steps_to_endZ = rampdown;
		}
		if(state_hw.current_task_ref->steps_to_endX > rampdown){
			state_hw.current_task_ref->steps_to_endX = rampdown;
		}
		// reset task cb:
		cb_reset(&task_cb);
	}
}

//	cb_init_ref(&uart_cb, 10, sizeof(ubuf_t), &ut);
__STATIC_INLINE 
ubuf_t * add_empty_uart(){
	cb_push_back_empty(&uart_cb);
	return uart_cb.top;
}


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of DR register */
	uint8_t symbol = LL_USART_ReceiveData8(USART1);
	if(symbol == '\n'){
		if(uart_cb.count<uart_cb.capacity-2) {
			aRXBufferA[uwNbReceivedChars] = 0;
			if(uwNbReceivedChars == 2 && (aRXBufferA[1] == 'w' || aRXBufferA[1] == 's') ){
				if(jogQueue_cb.capacity-2 > jogQueue_cb.count) {
					cb_push_back_unclean(&jogQueue_cb);
					char *cp = jogQueue_cb.top;
					*cp = aRXBufferA[1];
				}
			} else {
				cb_push_back_unclean(&uart_cb);
				memcpy(uart_cb.top,&aRXBufferA,uwNbReceivedChars+1);
			}
		}
    uwNbReceivedChars = 0;
	} else {
		if(symbol != 0)
			pBufferReadyForReception[uwNbReceivedChars++] = symbol;
		if(uwNbReceivedChars == RX_BUFFER_SIZE-1){
			Error_Handler2(1);
		}
	}
}

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
//	ui16toa2(&state_hw.flags, (uint8_t *)&state_hw.uart_header[3],1);
	
	
	
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
	"G94",
//	"G33 Z-10 K4.5 M3",
	"G0 X0. Z0.",
	"G0 X0. Z0.1",
	"G0 X0. Z0.1",
	"G0 X0. Z0.1",
	"G0 X0. Z0.1",
	"G1 X10 Z100 F100",
	"G94",
	"G1 Z-600 F600", //F900",
	"G0 Z200",
	
//	"G1 Z-2 F0.05",
	#else
	"G91",
	"G95",
//	"G76 P0.05 Z-1 I-.075 J0.008 K0.045 Q29.5 L2 E0.045",
//	"G76 P2. Z-50. I-8.209 J0.25 K2. R2. Q29.5 H0. E1. L0",

	"G0 X0. Z0.",
//	"G1 Z200 F0.1",
//	"G0 X10. Z0.",
//	"G1 X1 Z-20 F600",
	"G1 Z-200 F0.5",
	"G0 X0. Z0.",
	"G1 X10. Z-10 F600",
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

  state_hw.uart_header[0] = HEADER_0;
	state_hw.uart_header[1] =state_hw.uart_header[2] =state_hw.uart_header[3] = HEADER_1;
	state_hw.uart_end[0] = '#';
	state_hw.uart_end[1] = '#';
	state_hw.uart_end[2] = '\r';
	state_hw.uart_end[3] = '\n';
	char info[14];
//	memcpy(info,"1.80;",5);
//	state_hw.global_Z_pos = (2<<23) - 1;
	LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_IWDG_STOP);
//ui64toa(state_hw.global_Z_pos,&info[0]);



	cb_init_ref(&jogQueue_cb, jogQueue_size, sizeof(char), &jogQueue);
	cb_init_ref(&uart_cb, uartBuffer_size, sizeof(char), &uartBuffer);


	cb_init_ref(&task_cb, task_size, sizeof(G_task_t), &gt);
//	cb_init_ref(&task_precalc_cb, task_precalc_size, sizeof(G_task_t), &gt_precalc);
//	cb_init_ref(&gp_cb, gp_size, sizeof(G_pipeline_t),&gp);
//	cb_init_ref(&substep_cb, substep_size, sizeof(substep_t),&substep_delay);
//	cb_init_ref(&substep_job_cb, substep_job_size, sizeof(substep_job_t),&substep_delay);

// g03
//	cb_init_ref(&sma_cb,  8, sizeof(substep_sma_ref_t), &smaNumbers);
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

	LL_TIM_CC_DisablePreload(TIM3);
	LL_TIM_OC_DisablePreload(TIM3, LL_TIM_CHANNEL_CH2);

	LL_TIM_EnableIT_CC1(TIM3);
//	LL_TIM_SetOnePulseMode(TIM1,LL_TIM_ONEPULSEMODE_SINGLE);

	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH2);

  /* Clear Overrun flag, in case characters have already been sent to USART */
  LL_USART_ClearFlag_ORE(USART1);

/* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
	LL_USART_ClearFlag_TC(USART1);


/* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(USART1);

	G95(&state_hw);
	switch_to_sync(&state_hw); //  set sync mode as default

// todo need refactor this code how to g-code parsing, precalculation and execution going to start and work together
	G_task_t record_task = {0};
	int32_t record_X = 0, record_Z = 0;
//	G_task_t *precalculating_task = 0;
	int command = 0;
	bool move_to_saved_pos = false;

// some init hardware state
//	state_hw.substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));
//	state_hw.substep_pulse_on = 1;
//	state_hw.substep_pulse_off = 0;
//	state_hw.substep_axis = SUBSTEP_AXIS_X;
	
//	state_hw.uart_header 
	uint32_t bn = Build_No;
	sendDefaultResponseDMA(RESET_code,&bn);
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
	bool skipPop = false; 
	LED_OFF();
	while (1) {
		// recalc substep delays
		if(state_precalc.current_task_ref){
			// get pointer to last processed task
			if(state_precalc.current_task_ref->precalculate_callback_ref) {
				state_precalc.current_task_ref->precalculate_callback_ref(&state_precalc);
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
						if(state_precalc.current_task_ref->flags.unlocked  == true) // precalc finished, load next task to precalc
							state_precalc.current_task_ref = 0;
					} else {
						state_precalc.current_task_ref = 0;
					}
				}
			} else 
			if (state_precalc.current_task_ref ){ // buffer overflow, unlock task to process precalculated data
					state_precalc.current_task_ref->flags.unlocked = true;
			} 
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
//process commands received by uart that should be executed in parallel with G-commands(jog x)
		if(jogQueue_cb.count > 0){
		  char *next_jogcmd = cb_get_front_ref(&jogQueue_cb);
//			char cmd  = ((char * )next_jogcmd);
			switch(*next_jogcmd){
#define jogwindow 5				
					case 'w': {// go forward by 1 step '!w'
						if(state_hw.current_task_ref == 0 || state_hw.current_task_ref->dx ==0){
							if(XDIR != xdir_forward){
								XDIR = xdir_forward;
								delay3us();
							}
							state_hw.global_X_pos--;
							record_X--;

							LL_GPIO_SetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
							delay3us();
							LL_GPIO_ResetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
							sendDefaultResponseXZ();
						} else {
							if(xPulse)
								break;
//							while(xPulse != false){
//								__NOP();
//							}

							uint32_t x_axis = fixedpt_toint(state_hw.x1616set) - 1; // x delay
							// arr = 300 is 600mm/min async 
							volatile uint32_t waitw = 0;
							if(x_axis >700){
								x_axis = x_axis / 2;
								while(1){
									volatile uint32_t t1cnt = LL_TIM_GetCounter(TIM1);
									if(t1cnt > (x_axis - jogwindow) && t1cnt < ((x_axis + jogwindow))){
										break;
									}
									waitw++;
								}
//								while(LL_TIM_GetCounter(TIM1)< (x_axis - jogwindow) || LL_TIM_GetCounter(TIM1) > (x_axis+jogwindow)){
//									__NOP();
//								}

								bool restore_dir = false;
								if(XDIR != xdir_forward){
									restore_dir = true;
									XDIR = xdir_forward;
									delay3us();
								}

								state_hw.global_X_pos--;
								record_X--;
								LL_GPIO_SetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
								delay3us();
								LL_GPIO_ResetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
								if(restore_dir) {
									delay3us();
									XDIR = xdir_backward;
//									delay3us();
								}
							}
						}
						break;
					}

					case 's': {// go backward by 1 step '!s'
						if(state_hw.current_task_ref == 0 || state_hw.current_task_ref->dx ==0){
							if(XDIR != xdir_backward){
								XDIR = xdir_backward;
								delay3us();
							}
							state_hw.global_X_pos++;
							record_X++;
							LL_GPIO_SetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
							delay3us();
							LL_GPIO_ResetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
							sendDefaultResponseXZ(); //							sendDefaultResponseDMA('X',&state_hw.global_X_pos);
						} else {
//							while(xPulse != false){ // wait until pulse by timer interrupt finished
//								__NOP();
//							}
/* 1. check current movement speed(by delay between pulses). if speed is to high to insert jog pulse between them - jog step not possible.
if movement speed allow to insert jog step then start to alanyse it
2. simple way is to place jog pulse in the middle between axis main pulses
(min delay * 2 equal max speed/2  )							
3. todo
what to do if main delay is too big(slow movement)? few jog pulses can be placed there
neet to check acceleration
bug when jogs too fast multiple pulses is appear in middle. need some flag for it? need to move jog pulses to another circular buffer?
*/
							uint32_t x_axis = fixedpt_toint(state_hw.x1616set) - 1; // x delay
							// arr = 300 is 600mm/min async 
							if(x_axis >700){
								x_axis =  x_axis / 2;
								while(1){
									volatile uint32_t t1cnt = LL_TIM_GetCounter(TIM1);
									if(t1cnt > (x_axis - jogwindow) && t1cnt < ((x_axis + jogwindow))){
										break;
									}
//									waitw++;
								}
								
								bool restore_dir = false;
								if(XDIR != xdir_backward){
									restore_dir = true;
									XDIR = xdir_backward;
									delay3us();
								}

								state_hw.global_X_pos++;
								record_X++;
								LL_GPIO_SetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
								delay3us();
								LL_GPIO_ResetOutputPin(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin);
								if(restore_dir) {
									delay3us();
									XDIR = xdir_forward;
//									delay3us();
								}
							}
						}
						break;
					}
			}
			cb_pop_front_ref(&jogQueue_cb);
		}

		// process uart buffer(except jog x):
		if( (uart_cb.count > 0 && task_cb.count < task_cb.capacity-2) ){
		  ubuf_t *next_uartcmd = cb_get_front_ref(&uart_cb);
//			uint8_t cmd;
//			cmd  = ((uint8_t * )next_uartcmd)[0];
			cmd1 = ((uint8_t * )next_uartcmd)[1];
			cmd2 = ((uint8_t * )next_uartcmd)[2];
			ubUART3ReceptionComplete = 0;

			if(((uint8_t * )next_uartcmd)[0] == '!'){
				switch(cmd1){
					case '=':
						switch(cmd2){
							case 'X':
							  state_hw.global_X_pos = ahextoui32(&((uint8_t * )next_uartcmd)[3]);
							break;
							case 'Y':
							  state_hw.global_Z_pos = ahextoui32(&((uint8_t * )next_uartcmd)[3]);
							break;
						}
						break;
					case '1':
//						if( abs(state_hw.global_X_pos - record_X) > abs(state_hw.retract) || abs(state_hw.global_Z_pos - record_Z) > abs(state_hw.retract) ){
							if(state_hw.flags.G90G91 == G90mode){
								scheduleG00G01move3(	fixedpt_fromint2210(state_hw.initial_task_X_pos), fixedpt_fromint2210(state_hw.initial_task_Z_pos), 0,0, G00code);
							} else {
								scheduleG00G01move3(	fixedptu_fromint2210(record_X) - init_gp.X, fixedptu_fromint2210(record_Z) - init_gp.Z, 0, 0, G00code);
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
					  
						record_task.callback_refZ = dz_callback;
						record_task.callback_refX = dx_callback;
//						if(dbg_ssh)
							trim_substep_short();
//						else 
//							trim_substep();
						// due to predict where machine will end its movement currently work not good do next trick:
						//wait until active task is finished to update init_gp position with actual global_Z(X)_pos:
						while(state_hw.current_task_ref != 0){
						}

							
						record_X = state_hw.initial_task_X_pos;
						record_Z = state_hw.initial_task_Z_pos;

						int local_Z = fixedpt_toint2210(init_gp.Z);
						int local_X = fixedpt_toint2210(init_gp.X);

						int back_dz = record_Z - local_Z;
						int back_dx = record_X - local_X;
						record_task.dz = abs(back_dz);
						record_task.dx = abs(back_dx);
						record_task.steps_to_endX = record_task.dx;
						record_task.steps_to_endZ = record_task.dz;
//						record_task.F = state_hw.Q824set; // get from hw if it feed was changed on the fly

					// move back:
					// 1. move back 400 steps from blank(or move forward if ID)
						scheduleG00G01move3(fixedpt_fromint2210(state_hw.retract), 0, 0,0, G00code);
					// repeat same movement in back direction
						scheduleG00G01move3(fixedpt_fromint2210(back_dx), fixedpt_fromint2210(back_dz), 0,0, G00code);
          // move forward 400 steps to blank
						scheduleG00G01move3(-fixedpt_fromint2210(state_hw.retract), 0, 0,0, G00code);
//						EOM();
//						sendResponse((uint32_t)"ok\r\n",4);
						break;
					case '3': //repeat last command '!3'
						{
							cb_push_back_item(&task_cb,&record_task);
							init_gp.Z += record_task.flags.z_direction == zdir_backward ? -fixedpt_fromint2210(record_task.dz) : fixedpt_fromint2210(record_task.dz);
							init_gp.X += record_task.flags.x_direction == zdir_backward ? fixedpt_fromint2210(record_task.dx) : -fixedpt_fromint2210(record_task.dx);

							int back_dz = record_task.flags.z_direction ==zdir_forward ? -record_task.dz : record_task.dz;
							int back_dx = record_task.flags.x_direction ==xdir_forward ? record_task.dx : -record_task.dx;

							scheduleG00G01move3(fixedpt_fromint2210(state_hw.retract), 0, 0, 0,G00code);
							scheduleG00G01move3(fixedpt_fromint2210(back_dx), fixedpt_fromint2210(back_dz), 0,0, G00code);
							scheduleG00G01move3(-fixedpt_fromint2210(state_hw.retract), 0, 0,0, G00code);
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
						uint8_t direction = *next_uartcmd[2] - '0' > 9 ? *next_uartcmd[2] - 'A' + 10 : *next_uartcmd[2] - '0';
						uint8_t speed = *next_uartcmd[3] - '0';
						if(speed>0)
							state_hw.vector = 1;
						break;
					}

					case 'B': {// left correction small(Back) '!B' todo refactor
						//switch to manual pulse generation
						uint32_t pin_dir = LL_GPIO_IsOutputPinSet(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
						if(pin_dir != zdir_backward){ // change DIR
							LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
							delay3us();
						}
						LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
						for(int a = 0;a<20;a++){
							LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
							delay3us();
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
							delay3us();
						}
						LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
						for(int a = 0; a<20; a++){
							LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
							delay3us();
							LL_GPIO_ResetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
							LL_mDelay(5);
						}
						if(pin_dir != zdir_backward){ // change DIR back
							LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port,MOTOR_Z_DIR_Pin);
						}
						LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
						break;
					}
					case 'a': {// left small '!a'
						scheduleG00G01move3(0, -jog_Z_01_2210, 0,0, G00code);
						break;
					}
					case 'd': {// right small '!d'
						scheduleG00G01move3(0, jog_Z_01_2210, 0,0, G00code);
						break;
					}

					case 'g':{ //decrease feed speed on the fly, resolution 0.1mm (0.1*1000=100)
						int32_t ff = feed_on_the_fly_factor / state_hw.z1616set - 100;
						if(ff < 100) // limit min FOTF by 0.1mm/rev
							ff = 100;
						record_task.Fz = state_hw.z1616set = feed_on_the_fly_factor / ff;
						sendDefaultResponseDMA(FEED_code,&state_hw.z1616set);
						break;
					}
					case 't': { //increase feed speed on the fly
						// 1474560 = 65536*encoder_resolution*z_screw_pitch/z_steps_unit = 3600*0,00625*65536 and * 1000 to get integer math
						int32_t ff = feed_on_the_fly_factor / state_hw.z1616set + 100;
						if(ff > 3000) // limit max FOTF by 3mm/rev
							ff = 3000;
						record_task.Fz = state_hw.z1616set = feed_on_the_fly_factor / ff;
						sendDefaultResponseDMA(FEED_code,&state_hw.z1616set);
						break;
					}
					
					case 'I': // set ID
//						state_hw.ODID = 1;
						state_hw.flags.ODID = IDmode;
						state_hw.retract = -100;
						break;
					case 'O': // set OD
//						state_hw.ODID = 0;
						state_hw.flags.ODID = ODmode;
						state_hw.retract = 100;
						break;
				}
				// 
				
			}  else { // if its not cmd started by
				command_parser(*next_uartcmd);
			}
//			if(!skipPop)
			cb_pop_front_ref(&uart_cb);
//		else
//				skipPop = false;
		}
		// process preload buffer:
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

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);
  NVIC_SetPriority(TIM1_CC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_CC_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 100;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_DisableIT_TRIG(TIM1);
  LL_TIM_DisableDMAReq_TRIG(TIM1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
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

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 239;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM2);
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
  TIM_InitStruct.Autoreload = 100;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

//	LL_TIM_DisableIT_UPDATE(TIM3);
//	LL_TIM_GenerateEvent_UPDATE(TIM3);
//	LL_TIM_ClearFlag_UPDATE(TIM3);
//	LL_TIM_EnableIT_UPDATE(TIM3); // */

//	MOTOR_X_AllowPulse();
//	MOTOR_Z_AllowPulse();
	
	
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
  LL_TIM_SetEncoderMode(TIM4, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
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
  USART_InitStruct.BaudRate = 921600;
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
  LL_GPIO_ResetOutputPin(GPIOB, MOTOR_X_STEP_Pin|MOTOR_Z_ENABLE_Pin|MOTOR_Z_DIR_Pin|MOTOR_Z_STEP_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, MOTOR_X_ENABLE_Pin|MOTOR_X_DIR_Pin);

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
  GPIO_InitStruct.Pin = MOTOR_X_STEP_Pin|MOTOR_Z_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
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

	
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 999;
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

volatile  int codeg = 0;
volatile uint64_t eh = 0;
void Error_Handler2(int code)
{
	codeg = code;
	/* User can add his own implementation to report the HAL error return state */
	TIM1->CR1 = TIM2->CR1 = TIM3->CR1 = TIM4->CR1 = 0;
	while (1) {
	}
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
  __disable_irq();
  while (1)
  {
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
