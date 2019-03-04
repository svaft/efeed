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

#include "screen.h"
//#include "ssd1306.h"


#include "i2c_interface.h"

#include "buttons.h"
#include "fsm.h"
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

//axis z_axis;
state_t state;

/* Private variables ---------------------------------------------------------*/
//int count;
extern bool demo;


//uint32_t current_pos = 0, cpv = 0, end_pos = 0, mode = 10, mode_prev = 10;

// ***** Stepper Motor *****

bool Spindle_Direction = Spindle_Direction_CW;
bool feed_direction = feed_direction_left;

uint32_t menu_changed = 0;

//#define _SIMU
bool auto_mode = false;
int32_t auto_mode_delay = -1; // default delay between change direction is 6 secons

uint32_t rs = 0; // temp tamp_step to update screen

/*
infeed steps count depends on lathe-tool-part rigid
recommendation: mm(tpi) - passes
				0,50-0,75(48-32) - 4-5 passes,
				0,80-1,00(28-24) - 5-6,
				1,25-1,50(20-16) - 6-8
				1,75-2,00(14-12) - 8-10,
				2,50-3,00(11,5-9) - 9-12
*/

// основное меню. Считаем по формуле:
// Enc_Line/(Step_Per_Revolution/Feed_Screw*Thread_mm)
// перегенерация есть в excel файле
const THREAD_INFO Thread_Info[] = {
//	{ 0x18000000, 0, "0.50", "mm", 0, ".34", ".013", 0 },
	{ 0x06000000, 0, "1.50", "mm", 0, ".95", ".037", 0 },
#ifndef _SIMU
	{ 0x02400000, 0, "4.00", "mm", 10, "1.26", ".050", 0 },
//	{ 0xF0000000, 0, "1.00", "mm", 0, ".65", ".026", 0 },
	{ 0x09000000, 0, "1.00", "mm", 0, ".65", ".026", 1 },
	{ 0x04800000, 0, "2.00", "mm", 0, "1.26", ".050", 2 },
	{ 0x06000000, 0, "1.50", "mm", 0, ".95", ".037", 0 },
	{ 0x09000000, 0, "1.00", "mm", 0, ".65", ".026", 0 },
	{ 0x00000000, 20, "F", "mm", 0, "", "", 0 },
	{ 0x2D000000, 0, "0.20", "mm", 20, "", "", 0 },
	{ 0x32000000, 0, "0.18", "mm", 20, "", "", 0 },
	{ 0x3C000000, 0, "0.15", "mm", 20, "", "", 0 },
	{ 0x4B000000, 0, "0.12", "mm", 20, "", "", 0 },
	{ 0x64000000, 0, "0.09", "mm", 20, "", "", 0 },
	{ 0x96000000, 0, "0.06", "mm", 20, "", "", 0 },
	{ 0xE1000000, 0, "0.04", "mm", 20, "", "", 0 },
	{ 0x00000000, 0, "..", "up", 20, "", "", 0 },
	{ 0x00000000, 10, "T", "mm", 0, "", "", 0 },
	{ 0x07333333, 0, "1.25", "mm", 10, ".79", ".031", 0 },
	{ 0x05249249, 0, "1.75", "mm", 10, "1.11", ".044", 0 },
	{ 0x04800000, 0, "2.00", "mm", 10, "1.26", ".050", 0 },
	{ 0x12000000, 0, "0.50", "mm", 10, ".34", ".013", 0 },
	{ 0x0C000000, 0, "0.75", "mm", 10, ".50", ".020", 0 },
	{ 0x00000000, 0, "..", "up", 10, "", "", 0 },
	{ 0x00000000, 30, "T", "tpi", 0, "", "", 0 },
	{ 0x09912244, 0, "27", "tpi", 30, "", "", 0 },
	{ 0x09366CD9, 0, "26", "tpi", 30, "", "", 0 },
	{ 0x08810204, 0, "24", "tpi", 30, "", "", 0 },
	{ 0x07CB972E, 0, "22", "tpi", 30, "", "", 0 },
	{ 0x07162C58, 0, "20", "tpi", 30, "", "", 0 },
	{ 0x06BB76ED, 0, "19", "tpi", 30, "", "", 0 },
	{ 0x0660C183, 0, "18", "tpi", 30, "", "", 0 },
	{ 0x05AB56AD, 0, "16", "tpi", 30, "", "", 0 },
	{ 0x04F5EBD7, 0, "14", "tpi", 30, "", "", 0 },
	{ 0x04408102, 0, "12", "tpi", 30, "", "", 0 },
	{ 0x00000000, 0, "..", "up", 30, "", "", 0 },
#endif
};


uint8_t Menu_Step = 0;																					// выборка из массива по умолчанию (1.5mm)
const uint8_t Menu_size = sizeof(Thread_Info)/sizeof(Thread_Info[0]);
//uint16_t text_buffer[100];
//uint32_t tbc = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

S_WORK_SETUP work_setup;

const fixedptud enc_setup = 0x9000000000000;

#define RX_BUFFER_SIZE   12
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
__IO uint32_t     uwNbReceivedChars;
__IO uint32_t     uwBufferReadyIndication;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;

__IO uint8_t ubUART2ReceptionComplete = 0;

/* Buffer used for transmission */
const uint8_t aTxBuffer[] = "test\r\n";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
__IO uint8_t ubTransmissionComplete = 0;



/**
  * @brief  Function called from DMA1 IRQ Handler when Rx transfer is completed 
  * @param  None
  * @retval None
  */
void DMA1_ReceiveComplete_Callback(void)
{
  /* DMA Rx transfer completed */
  ubUART2ReceptionComplete = 1;
}


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART_CharReception_Callback(void)
{
	uint8_t *ptemp;
  /* Read Received character. RXNE flag is cleared by reading of DR register */
	uint8_t symbol = LL_USART_ReceiveData8(USART2);
	if(symbol == '\n' || symbol == '\r'){
    /* Set Buffer swap indication */
		ubUART2ReceptionComplete = 1;

    /* Swap buffers for next bytes to be received */
    ptemp = pBufferReadyForUser;
    pBufferReadyForUser = pBufferReadyForReception;
    pBufferReadyForReception = ptemp;
    uwNbReceivedChars = 0;
	} else {
		pBufferReadyForReception[uwNbReceivedChars++] = symbol;
	}
}


uint64_t tst = 20318984;
fixedpt f1;


uint32_t rr = 174240000; //33mm*33*400*400
uint32_t y1 = 5800;
uint32_t y2 = 12480;
uint32_t x1 = 0;

//G_pipeline gpp[100];
//LL_GPIO_SetOutputPin
//LL_GPIO_ResetOutputPin


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	MOTOR_Z_DIR_GPIO_Port->BSRR
//	gt[0].z_direction = zdir_forward; //oDR 0x4001080C xdir-odr 0x4001100C
//	gt[0].x_direction = xdir_forward; //oDR 0x4001080C xdir-odr 0x4001100C
//	zdir = gt[0].z_direction;
	memset(&state,0,sizeof(state));
//	memset(&z_axis,0,sizeof(z_axis));
	state.function = do_fsm_menu_lps;
	state.callback = dxdz_callback;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /**NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
/*
	MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//	LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port, MOTOR_Z_DIR_Pin);
//	debug();
//	ZDIR = 1;
//	TIMER_B0 = 1;
/*
// perfomance measurement block
	debug();
	for (int a = 0; a<6680;a++){
		y1+=a;
//		x1 = sqrt(rr - y1*y1); //37us
		x1 = SquareRoot(rr - y1*y1); //3,22us
	}
	debug();
*/
	//	return 0;
	
//	z_axis.Q824set = Thread_Info[Menu_Step].Q824;

//	char *end;


	cb_init_ref(&task_cb, 100, sizeof(G_task), &gt);
	cb_init_ref(&gp_cb, 100, sizeof(G_pipeline),&gp);

//	int size = 10;
//	cb_init(&gp_cb, size, sizeof(G_pipeline));
	G01parse("X1. Z-2.5 F1.");
//	G01parse("X10.");
//	G03parse("X12. Z-4.5 I-1.99 K-2.245");
	debug();

	static const char * const garray[] = { 
		"G1 X0. Z-2.5 F1.",
		"G1 X10.", 
		"G3 X12. Z-4.5 I-1.99 K-2.245" 
//"G1 X40.279 Z-31.064",
//"X40.554 Z-31.09",
//"X40.822 Z-31.132",
	};

	static const char * shape1[] = {
"G1 X2.828 F1",
"X0. Z-1",
"X30.",
"G3 X40. Z-6 K-5",
"G1 Z-11.056",
"G2 Z-26.056 I8.597 K-7.5",
"G1 Z-31.056",
"X40.279 Z-31.064",
"X40.554 Z-31.09",
"X40.822 Z-31.132",
"X41.078 Z-31.188",
"X41.322 Z-31.258",
"X41.553 Z-31.34",
"X41.771 Z-31.431",
"X41.976 Z-31.53",
"X42.347 Z-31.744",
"X42.688 Z-31.983",
"X43.012 Z-32.251",
"X43.321 Z-32.548",
"X43.614 Z-32.87",
"X43.893 Z-33.215",
"X44.157 Z-33.583",
"X44.408 Z-33.97",
"X44.648 Z-34.379",
"X44.877 Z-34.808",
"X45.302 Z-35.72",
"X45.688 Z-36.698",
"X46.035 Z-37.732",
"X46.347 Z-38.812",
"X46.626 Z-39.928",
"X46.874 Z-41.071",
"X47.094 Z-42.234",
"X47.292 Z-43.436",
"X47.471 Z-44.68",
"X47.63 Z-45.962",
"X47.771 Z-47.28",
"X47.895 Z-48.631",
"X48.001 Z-50.011",
"X48.092 Z-51.418",
"X48.167 Z-52.849",
"X48.227 Z-54.301",
"X48.273 Z-55.771",
"X48.306 Z-57.255",
"X48.326 Z-58.751",
"X48.335 Z-60.256",
"X48.333 Z-61.767",
"X48.32 Z-63.28",
"X48.297 Z-64.794",
"X48.266 Z-66.304",
"X48.226 Z-67.808",
"X48.179 Z-69.304",
"X48.125 Z-70.787",
"X48.065 Z-72.255",
"X48. Z-73.705",
"G3 Z-101.073 I-15.839 K-13.684",
"G1 X50.828 Z-99.659",
"X55.068"
	};
	for(int a = 0; a < sizeof(garray); a++ ){
		command_parser((char *)garray[a]);
		process_G_pipeline();
	}

	return 0;


  /* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(USART2);
  /* Enable DMA Channel Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);

/* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);



// Timers post init:
	LL_TIM_GenerateEvent_UPDATE(TIM2);
//  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // if we need output on leg
  LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);

//	LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin);


//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
//  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1);
//	TIM3->SR = 0;
//	TIM2->SR = 0;
	TIM3->ARR = 0;
	LL_TIM_GenerateEvent_UPDATE(TIM3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	TIM3->SR = 0;
	LL_TIM_EnableIT_UPDATE(TIM3);

//	LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin);



	if(LL_GPIO_IsInputPinSet(BUTTON_1_GPIO_Port, BUTTON_1_Pin)){
		demo = true;
	}
//	MOTOR_Z_Disable();
//	MOTOR_X_Disable();
// инициализация дисплея
#ifndef _SIMU
	Activate_I2C_Master();
	init_screen(I2C2);
//	update_screen();
//	i2c_device_init(I2C2);
	LL_mDelay(250);
#endif
	init_buttons();
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


///// from STM examples:
//  /**************************/
//  /* Start pulse generation */
//  /**************************/
//  /* Enable channel 1 */
//  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
//  
//  /* Enable TIM3 outputs */
//  LL_TIM_EnableAllOutputs(TIM3);
//  
//  /* Enable auto-reload register preload */
//  LL_TIM_EnableARRPreload(TIM3);

//  /* Force update generation */
//  LL_TIM_GenerateEvent_UPDATE(TIM3);  


//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);




//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
//  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1); 				//trigger by TIM2(async mode)
//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);

//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
//  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3); 				//trigger by spindle encoder timer TIM4(sync mode)
//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);


//	MOTOR_X_BlockPulse(); // LL_TIM_OC_SetCompareCH3(TIM3, 0);
//	MOTOR_Z_BlockPulse(); // LL_TIM_OC_SetCompareCH3(TIM3, 0);

//LL_TIM_EnableCounter(TIM3);


//TIM3->SR = 0;
//TIM3->EGR |= TIM_EGR_UG;
//		LL_TIM_GenerateEvent_UPDATE(TIM3); /* Force update generation */

//  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	//GPIOB->BSRR
//	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);
//	LL_mDelay(50);
//	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);
//	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
	
//	LL_TIM_EnableAllOutputs(TIM3);
//MOTOR_X_AllowPulse();
//MOTOR_Z_AllowPulse();
//		LL_mDelay(50);
//	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
  LL_TIM_EnableIT_CC3(TIM4);													// enable interrupts for TACHO events from encoder
  LL_TIM_EnableCounter(TIM4); 												//Enable timer 4
//	enable_encoder_ticks(); 														// enable interrup for encoder ticks
	TIM4->SR = 0; 																			// reset interrup flags

//	LL_TIM_EnableIT_UPDATE(TIM1);
//	LL_TIM_EnableIT_UPDATE(TIM2);
//	LL_TIM_EnableCounter(TIM2);
	
//	do_fsm_move_start(&state);

	
  /* Enable counter */
//  LL_TIM_EnableCounter(TIM2);
  /* Force update generation */
//  LL_TIM_GenerateEvent_UPDATE(TIM2);

// init buttons
//	LL_mDelay(5);
	do_fsm_menu(&state);
//	LL_mDelay(5);
//	LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin);
	LED_GPIO_Port->BSRR = LED_Pin; // led off
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifndef _SIMU		
//		reqest_sample_i2c_dma(); // init reqest to joystick by DMA, when process_button complete i2c done its job
#endif		
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);
		process_G_pipeline();
		process_button();
//		process_joystick();
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);


//		uint8_t level = Thread_Info[Menu_Step].level;

//		if(auto_mode == true) {
//			if ( auto_mode_delay == 0 ) {
//				buttons_flag_set = single_click_Msk; //
//			}
//		}

		if(buttons_flag_set) {
			do_fsm_menu(&state);
			buttons_flag_set = 0; // reset button flags
		}

//		if(z_axis.ramp_step != rs) {
//			rs = z_axis.ramp_step;
//			menu_changed = 1;
//		}

//		if(z_axis.current_pos != rs) {
//			rs = z_axis.current_pos;
//			menu_changed = 1;
//		}

// update display info
		if(menu_changed == 1){ // haltodo && hi2c2.hdmatx->State == HAL_DMA_STATE_READY) {
			menu_changed = update_screen();
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
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
#ifdef _SIMU
	return;
#endif	
  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration  
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* I2C2 DMA Init */
  
  /* I2C2_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* I2C2 interrupt Init */
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  /**I2C Initialization 
  */
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 720;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
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
  TIM_InitStruct.Prescaler = 2400; //for 30kHz  //720 //;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 50;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 48;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
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
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetOnePulseMode(TIM3, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1);
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration  
  PA6   ------> TIM3_CH1
  PB0   ------> TIM3_CH3 
  */
  GPIO_InitStruct.Pin = MOTOR_X_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(MOTOR_X_STEP_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOTOR_Z_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(MOTOR_Z_STEP_GPIO_Port, &GPIO_InitStruct);

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
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 8;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetEncoderMode(TIM4, LL_TIM_ENCODERMODE_X2_TI1);
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  pBufferReadyForReception = aRXBufferA;
  pBufferReadyForUser      = aRXBufferB;
  uwNbReceivedChars = 0;
  ubUART2ReceptionComplete = 0;
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

  /* USART2 DMA Init */
  
  /* USART2_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_7,
                         (uint32_t)aTxBuffer,
                         LL_USART_DMA_GetRegAddr(USART2),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, ubNbDataToTransmit);


  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  LL_GPIO_ResetOutputPin(GPIOC, LED_Pin|MOTOR_X_DIR_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, MOTOR_X_ENABLE_Pin|MOTOR_Z_DIR_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin|MOTOR_X_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_10 
                          |LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MOTOR_X_ENABLE_Pin|MOTOR_Z_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MOTOR_Z_ENABLE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(MOTOR_Z_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14 
                          |LL_GPIO_PIN_15|LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5 
                          |LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
