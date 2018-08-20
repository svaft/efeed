
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "screen.h"
//#include "ssd1306.h"
#include "fixedptc.h"
#include "i2c_interface.h"
#include "buttons.h"

//#define ARM_MATH_CM3
//#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

axis z_axis = { 0,0,fsm_menu,fsm_menu,0,0,0,0 };

/* Private variables ---------------------------------------------------------*/
//int count;


//uint32_t current_pos = 0, cpv = 0, end_pos = 0, mode = 10, mode_prev = 10;

// ***** Stepper Motor *****

bool Spindle_Direction = Spindle_Direction_CW;
bool feed_direction = feed_direction_left;

uint32_t menu_changed = 0;


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
	{ 0x12000000, 0, "0.50", "mm", 0, ".34", ".013", 0 },
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

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
	z_axis.mode = fsm_menu_lps;
	rs = 11;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

// инициализация дисплея
	init_screen(&hi2c2);

	i2c_device_init(&hi2c2);

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

	__HAL_TIM_SET_COUNTER(&htim4,0);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2 );
	if(HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
	TIM4->SR = 0; // reset interrup flags
//	TIM4->CR1 |= TIM_CR1_URS; // to update ARR register immediateley(skip shadow mechanism)

// init buttons

	LED_GPIO_Port->BSRR = LED_Pin; // led off

	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		reqest_sample_i2c_dma(); // init reqest to joystick by DMA, when process_button complete i2c done its job
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);
		process_button();
//		process_joystick();
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);
/*
//main Finite-state machine(Nondeterministic finite automaton):
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
*/

		uint8_t level = Thread_Info[Menu_Step].level;

		if(auto_mode == true) {
			if ( auto_mode_delay == 0 ) {
				buttons_flag_set = single_click_Msk; //
			}
		}

		switch(buttons_flag_set) {
			case single_click_Msk3: {
			feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
			menu_changed = 1;
			break;
		}
		case single_click_Msk2: {
			feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
			menu_changed = 1;
			break;
		}
		case single_click_Msk: {
			if(z_axis.end_pos != 0) {
				if(auto_mode == true && auto_mode_delay > 0) { // single click in auto mode temporary disable auto_mode, processing to be continued at next single click
					auto_mode_delay = -1;
					break;
				}

				// first pass of thread cut was complete, so just use single click
				//	to switch between modes to process all other cuts
				MOTOR_Z_Enable(); // time to wakeup motor from sleep is quite high(1.7ms), so enable it as soon as possible
				for(unsigned int i=0; i<(72*1700/16); i++); // wait 1700us delay to wakeup motor driver
				z_axis.mode = z_axis.current_pos > 0 ? fsm_sclick_event : fsm_main_cut_wait_tacho;
			} else { // controller in initial state, scroll menu
				z_axis.mode = fsm_menu_lps;
				for (int a = Menu_Step+1; a<Menu_size; a++) {
					if(Thread_Info[a].level == level) {
						Menu_Step = a;
						menu_changed = 1;
						break;
					}
				}
				if(menu_changed != 1) {
					for (int a = 0; a<Menu_Step; a++) {
						if(Thread_Info[a].level == level) {
							Menu_Step = a;
							menu_changed = 1;
							break;
						}
					}
				}
			}
			break;
		}
		case double_click_Msk: {
			feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
			menu_changed = 1;
			break;
		}
		case (long_press_start_Msk | long_press_start_Msk2): { // two buttons long pressed same time
			// todo check if it work
			break;
		}
		case long_press_start_Msk: {
			switch(z_axis.mode) {
			case fsm_menu_lps: {
				if(Thread_Info[Menu_Step].Q824 != 0) { // long press detected, start new thread from current position
					//mode 20:
					disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
					MOTOR_Z_Enable(); // time to wakeup motor from sleep is quite high(1.7ms), so enable it as soon as possible
					if( feed_direction == feed_direction_right )
						MOTOR_Z_Forward();
					else
						MOTOR_Z_Reverse();
					for(unsigned int i=0; i<(72*1700/16); i++); // wait 1700us delay to wakeup motor driver

					z_axis.Q824set = Thread_Info[Menu_Step].Q824;
					z_axis.end_pos = z_axis.current_pos = 0;
					const uint64_t upl = (uint64_t)3600 << 48;
					z_axis.prolong_addSteps = upl / (fixedptud)z_axis.Q824set;

					z_axis.mode = fsm_wait_tacho; // go straight to 24 to wait tacho
//																																																			count = 0;
				} else { // goto submenu
					for (int a = 0; a<Menu_size; a++) {
						if(Thread_Info[a].level == Thread_Info[Menu_Step].submenu) {
							Menu_Step = a;
							menu_changed = 1;
							//																													mode = 0;
							break;
						}
					}
				}
				break;
			}
			case fsm_main_cut_back: { // we going back to initial position and want to add some additional travel.
//															usecase: set thread cutter into current thread, with long press go to end of the thread and on way back home enlarge path to get full thread into work
				z_axis.mode = fsm_main_cut_back_prolong; // go to 48 mode to add threads until long_press end
				break;
			}
			}
			break;
		}
		case long_press_end_Msk: {
			switch(z_axis.mode) {
			case fsm_first_cut_main_part: {
//																					if(auto_mode == true){
//																									auto_mode_delay = auto_mode_delay_ms; //engage countdown timer to auto generate click event
//																					}
//																					Q824count = 0;
				z_axis.mode = fsm_first_cut_lpe;
				break;
			}
			case fsm_main_cut_back_prolong: { // end of prolonged mode
				z_axis.mode = fsm_main_cut_back;
				break;
			}
			}
			break;
		}
		}
		buttons_flag_set = 0; // reset button flags

//								if(z_axis.current_pos != z_axis.cpv) {
//										z_axis.cpv = z_axis.current_pos;
//												menu_changed = 1;
//								}

		if(z_axis.ramp_step != rs) {
			rs = z_axis.ramp_step;
			menu_changed = 1;
		}


		if(z_axis.mode != z_axis.mode_prev) {
			z_axis.mode_prev = z_axis.mode;
			menu_changed = 1;
		}
// update display info
		if(menu_changed == 1 && hi2c2.hdmatx->State == HAL_DMA_STATE_READY) {
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = min_pulse;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = min_pulse;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|MOTOR_X_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_X_ENABLE_Pin|MOTOR_Z_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin MOTOR_X_DIR_Pin */
  GPIO_InitStruct.Pin = LED_Pin|MOTOR_X_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA3 PA4 
                           PA5 PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_X_ENABLE_Pin MOTOR_Z_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR_X_ENABLE_Pin|MOTOR_Z_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_Z_ENABLE_Pin */
  GPIO_InitStruct.Pin = MOTOR_Z_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_Z_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*timIC)
{
	if (timIC->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {

	}

//	if(ready!=SET)return;
//HAL_TIM_ReadCapturedValue(timIC,TIM_CHANNEL_3);
}

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
