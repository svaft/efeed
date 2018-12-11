
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
#include "main.h"

/* USER CODE BEGIN Includes */
#include "screen.h"
//#include "ssd1306.h"
#include "fixedptc.h"
#include "i2c_interface.h"
#include "buttons.h"
#include "fsm.h"
#include "stm32f1xx_it.h"

//#define ARM_MATH_CM3
//#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

axis z_axis = { 0,0,0,0,0,0 };
state_t state = { do_fsm_menu_lps };

/* Private variables ---------------------------------------------------------*/
//int count;


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
THREAD_INFO Thread_Info[] = {
//	{ 0x12000000, 0, "0.50", "mm", 0, ".34", ".013", 0 },
	{ 0x02400000, 0, "4.00", "mm", 10, "1.26", ".050", 0 },
//{ 0xF0000000, 0, "1.00", "mm", 0, ".65", ".026", 0 },
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
};


uint8_t Menu_Step = 0;																					// выборка из массива по умолчанию (1.5mm)
const uint8_t Menu_size = sizeof(Thread_Info)/sizeof(Thread_Info[0]);
uint16_t text_buffer[100];
uint32_t tbc = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

S_WORK_SETUP work_setup;

const fixedptud enc_setup = 0x9000000000000;


/* todo from refactor branch
const fixedptu sqrt_map[]={ //precaclulated sqrt map for infeed
		0,
		16777216,
		23726566,
		29058991,
		33554432,
		37514995,
		41095619,
		44388341,
		47453133,
		50331648,
		53054215,
		55643730,
		58117981,
		60491113,
		62774594,
		64977878,
		67108864,
		69174234,
		71179699,
		73130189,
		75029991,
		76882862,
		78692118,
		80460701,
		82191237,
		83886080,
		85547352,
		87176972,
		88776682,
		90348073,
		91892597,
		93411585,
		94906266,
		96377768,
		97827139,
		99255348,
		100663296,
		102051821,
		103421705,
		104773680,
		106108431,
		107426598,
		108728787,
		110015563,
		111287461,
		112544986,
		113788615,
		115018798,
		116235962,
		117440512,
		118632832,
		119813287,
		120982225,
		122139976,
		123286856,
		124423164,
		125549188,
		126665203,
		127771470,
		128868241,
		129955756,
		131034246,
		132103931,
		133165024,
		134217728,
		135262240,
		136298747,
		137327431,
		138348467,
		139362023,
		140368260,
		141367335,
		142359398,
		143344596,
		144323069,
		145294953,
		146260378,
		147219473,
		148172360,
		149119158,
		150059982,
		150994944,
		151924152,
		152847712,
		153765725,
		154678289,
		155585501,
		156487453,
		157384237,
		158275939,
		159162646,
		160044440,
		160921403,
		161793612,
		162661144,
		163524074,
		164382474,
		165236415,
		166085965,
		166931191,
		167772160,
};

void recalculate_setup(){			 //recalculate current setup
		work_setup.Q824		 = Thread_Info[Menu_Step].Q824;
		work_setup.pitch		= (fixedptud)enc_setup / (fixedptud)work_setup.Q824;
		fixedpt_str( work_setup.pitch, (char *)&work_setup.Text, 2 );

		work_setup.thread_depth = ( (fixedptud)work_setup.pitch *(fixedptud)10905190 ) >> FIXEDPT_FBITS;
//	work_setup.thread_depth = fixedpt_mul( (fixedptud)work_setup.pitch, (fixedptud)10905190 );
		fixedpt_str( work_setup.thread_depth, (char *)&work_setup.infeed_mm, 2 );
		fixedpt_str( fixedptu_div( work_setup.thread_depth, Q824inch ), (char *)&work_setup.infeed_inch, 3 );


		// predict total_pass: recommended pass calculated by formula x=4y+2,
		// but it can be increased(modifyed) for small laithe
		work_setup.total_pass = fixedpt_toint( fixedpt_mul(0x6000000, work_setup.pitch) + 0x2000000 );
		work_setup.pass = 0;

		work_setup.infeed_mod = 7823344;

		fixedptu sqrt_steps = sqrt_map[work_setup.total_pass - 1]; // fixedptu_fromint( work_setup.total_pass - 1 );
//	sqrt_steps = fixedpt_sqrt( sqrt_steps );

// first step coefficient = 0.3, sqrt(0.3) = 0,547722558 = fpt9189259
// step 1 with coeff. 0.3
		work_setup.deltap_mm[0]		 = fixedptu_div( fixedptu_mul( work_setup.thread_depth, 9189259 ), sqrt_steps );
		work_setup.deltap_inch[0] = fixedptu_div( work_setup.deltap_mm[0], Q824inch );

		for(int step = 1; step < work_setup.total_pass; step++ ) {
				work_setup.deltap_mm[step]			= fixedptu_div( fixedptu_mul( work_setup.thread_depth, sqrt_map[step] ), sqrt_steps );
				work_setup.deltap_inch[step]		= fixedptu_div( work_setup.deltap_mm[step], Q824inch );
		}
}
*/


void recalculate_setup()  // todo: not ready yet
{
	work_setup.Q824			= Thread_Info[Menu_Step].Q824;
	work_setup.pitch				= (fixedptud)enc_setup / (fixedptud)work_setup.Q824;
	fixedpt_str( work_setup.pitch, (char *)&work_setup.Text, 2 );

	work_setup.thread_depth = fixedpt_mul( work_setup.pitch, 10905190 );
	work_setup.total_pass = 10;
	work_setup.pass = 0;

	work_setup.infeed_mod = 7823344;

	fixedptu sqrt_steps = fixedptu_fromint( work_setup.total_pass - 1 );
	sqrt_steps = fixedpt_sqrt( sqrt_steps );


// first step coefficient = 0.3, sqrt(0.3) = 0,547722558 = fpt9189259
// step 1 with coeff. 0.3
	work_setup.deltap_mm[0]			= fixedptu_div( fixedptu_mul( work_setup.thread_depth, 9189259 ), sqrt_steps );
	work_setup.deltap_inch[0] = fixedptu_div( work_setup.deltap_mm[0], 426141286 ); // 426141286 = Q824inch

	for(int step = 1; step < work_setup.total_pass; step++ ) {
		work_setup.deltap_mm[step]					= fixedptu_div( fixedptu_mul( work_setup.thread_depth, fixedpt_sqrt( fixedptu_fromint( step ) ) ), sqrt_steps );
		work_setup.deltap_inch[step]				= fixedptu_div( work_setup.deltap_mm[step], 426141286 );
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	z_axis.mode = fsm_menu_lps;
	rs = 11;
//	z_axis.end_pos = 50;
//	z_axis.Q824set = Thread_Info[Menu_Step].Q824;

//	state.main_feed_direction = 1;

	//	do_fsm_move_start(&state);
	//	do_fsm_wait_tacho(&state);
	
//	TIM4_IRQHandler();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

// инициализация дисплея
#ifndef _SIMU
	Activate_I2C_Master();
	init_screen(I2C2);
//	update_screen();
//	i2c_device_init(I2C2);
#endif
	LL_mDelay(250);

	init_buttons();

/*
	//72MGz процессор, 1 так = 1/72us, 1 цикл пустого for(для света до 255) равен 14 тактам + 6 тактов,
	//для больших чисел около 16 тактов на цикл + загрузка
	//	на первичную загрузку в первом цикле, т.е. 10*14+6=146 тактов, чуть более 2us

	count = 1000;
					while(1){
	//	for(int i=0;i<1000;i++){
									for(int i=0;i<(72*count/14+1);i++); // 2us delay
					__HAL_TIM_ENABLE(&htim3);
									count--;
									if(count < 500) count = 500;
	//					GPIOC->BRR = GPIO_PIN_13;
	//					for(unsigned int i=0;i<(72*1000/16);i++); // 1000us delay
					}
	*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	TIM3->CCER = TIM_CCER_CC1E; /* Enable the Compare output channel 1 */

	LL_TIM_SetCounter(TIM4,0);
//	__HAL_TIM_SET_COUNTER(&htim4,0);
  LL_TIM_EnableIT_CC3(TIM4);

  /***********************/
  /* Start input capture */
  /***********************/
  /* Enable output channel 1 & 2 */
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableIT_CC1(TIM4);
	LL_TIM_EnableIT_CC2(TIM4);
  /* Enable counter */
  LL_TIM_EnableCounter(TIM4);
//	LL_TIM_Ena
//	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2 );

  LL_TIM_EnableIT_CC3(TIM4);
/*
	if(HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
*/
	
	TIM4->SR = 0; // reset interrup flags
//	TIM4->CR1 |= TIM_CR1_URS; // to update ARR register immediateley(skip shadow mechanism)

	TIM4->ARR = 1;  // start stepper motor ramp up procedure immediately after tacho event
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CNT = 0;
	enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
	LL_TIM_EnableIT_UPDATE(TIM4);


	LL_TIM_EnableIT_UPDATE(TIM1);
	LL_TIM_EnableIT_UPDATE(TIM2);
	
//	do_fsm_move_start(&state);

	
  /* Enable counter */
//  LL_TIM_EnableCounter(TIM2);
  /* Force update generation */
//  LL_TIM_GenerateEvent_UPDATE(TIM2);

	//	LL_TIM_Bae
//	HAL_TIM_Base_Start_IT(&htim2);
// init buttons

	LED_GPIO_Port->BSRR = LED_Pin; // led off
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
#ifndef _SIMU		
		reqest_sample_i2c_dma(); // init reqest to joystick by DMA, when process_button complete i2c done its job
#endif		
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);
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

		if(z_axis.ramp_step != rs) {
			rs = z_axis.ramp_step;
			menu_changed = 1;
		}

		if(z_axis.current_pos != rs) {
			rs = z_axis.current_pos;
			menu_changed = 1;
		}

// update display info
		if(menu_changed == 1){ // haltodo && hi2c2.hdmatx->State == HAL_DMA_STATE_READY) {
			menu_changed = 0;
			update_screen();
		}
	}
  /* USER CODE END 3 */

}

static void LL_Init(void)
{
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

    /**NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
    */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

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

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  LL_I2C_InitTypeDef I2C_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

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
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(I2C2_ER_IRQn);

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

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct;

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);

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

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  TIM_InitStruct.Prescaler = 720;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);

  LL_TIM_EnableARRPreload(TIM2);

  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);

  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);

  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = min_pulse;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM3);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);

  LL_TIM_SetOnePulseMode(TIM3, LL_TIM_ONEPULSEMODE_SINGLE);

  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM3);

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

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  LL_TIM_InitTypeDef TIM_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  
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

  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM4);

  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_ACTIVEINPUT_DIRECTTI);

  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_ICPSC_DIV1);

  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_IC_FILTER_FDIV32_N8);

  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_RISING);

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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct;

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
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4 
                          |LL_GPIO_PIN_5|LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12 
                          |LL_GPIO_PIN_15;
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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
