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
#include "ssd1306.h"


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

bool Spindle_Direction = Spindle_Direction_CW;
bool feed_direction = feed_direction_left;

bool menu_changed = false;

//#define _SIMU
bool auto_mode = false;
int32_t auto_mode_delay = -1; // default delay between change direction is 6 secons

//uint32_t rs = 0; // temp tamp_step to update screen

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
	{ 0x0C000000, 0, "1.50", "mm", 0, ".95", ".037", 0 },
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

S_WORK_SETUP work_setup;
const fixedptud enc_setup = 0x9000000000000;


#define RX_BUFFER_SIZE   12
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
__IO uint32_t	uwNbReceivedChars;
__IO uint32_t	uwBufferReadyIndication;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;
__IO uint8_t ubUART2ReceptionComplete = 0;

/* Buffer used for transmission */
const uint8_t aTxBuffer[] = "ok\r\n";
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
__IO uint8_t ubTransmissionComplete = 0;
__IO uint8_t  ubMasterXferDirection     = 0;
uint8_t       aMasterReceiveBuffer[0xF] = {0};
__IO uint8_t  ubMasterNbDataToReceive   = sizeof(aMasterReceiveBuffer);


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
void USART_CharReception_Callback(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
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

/*
	const char *ga1[]={
"G95",
"G0 X10. Z1.038",
"G1 Z-20 F0.1",
"G1 Z0 F0.1",
"G0 X20. Z1.038",
"G0 X9.502",

"G33 Z-19.713 K4",
"X10. Z-19.962 K4",
"G0 X12.",
"Z0.979",
"X9.296",
"G33 Z-19.669 K4",
"X10. Z-20.021 K4",
"G0 X12.",
"Z1.024",
"X9.137",
"G33 Z-19.545 K4",
"X10. Z-19.976 K4",
"G0 X12.",
"Z0.986",
"X9.004",
"G33 Z-19.516 K4",
"X10. Z-20.014 K4",
"G0 X12.",
"Z1.02",
"X8.886",
"G33 Z-19.424 K4",
"X10. Z-19.98 K4",
"G0 X12.",
"Z0.989",
"X8.78",
"G33 Z-19.401 K4",
"X10. Z-20.011 K4",
"G0 X12.",
"Z1.017",
"X8.682",
"G33 Z-19.324 K4",
"X10. Z-19.983 K4",
"G0 X12.",
"Z0.991",
"X8.591",
"G33 Z-19.305 K4",
"X10. Z-20.009 K4",
"G0 X12.",
"Z1.016",
"X8.506",
"G33 Z-19.237 K4",
"X10. Z-19.984 K4",
"G0 X12.",
"Z0.992",
"X8.425",
"G33 Z-19.22 K4",
"X10. Z-20.008 K4",
"G0 X12.",
"Z1.014",
"X8.348",
"G33 Z-19.16 K4",
"X10. Z-19.986 K4",
"G0 X12.",
"Z0.993",
"X8.275",
"G33 Z-19.144 K4",
"X10. Z-20.007 K4",
"G0 X12.",
"Z1.014",
"X8.204",
"G33 Z-19.089 K4",
"X10. Z-19.986 K4",
"G0 X12.",
"Z0.994",
"X8.137",
"G33 Z-19.074 K4",
"X10. Z-20.006 K4",
"G0 X12.",
"Z1.013",
"X8.071",
"G33 Z-19.023 K4",
"X10. Z-19.987 K4",
"G0 X12.",
"Z0.995",
"X8.008",
"G33 Z-19.009 K4",
"X10. Z-20.005 K4",
"G0 X12.",
"Z1.012",
"X7.947",
"G33 Z-18.961 K4",
"X10. Z-19.988 K4",
"G0 X12.",
"Z0.995",
"X7.887",
"G33 Z-18.948 K4",
"X10. Z-20.005 K4",
"G0 X12.",
"Z1.012",
"X7.829",
"G33 Z-18.903 K4",
"X10. Z-19.988 K4",
"G0 X12.",
"Z0.996",
"X7.773",
"G33 Z-18.891 K4",
"X10. Z-20.004 K4",
"G0 X12.",
"Z1.011",
"X7.718",
"G33 Z-18.848 K4",
"X10. Z-19.989 K4",
"G0 X12.",
"Z0.996",
"X7.664",
"G33 Z-18.836 K4",
"X10. Z-20.004 K4",
"G0 X12.",
"Z1.011",
"X7.612",
"G33 Z-18.795 K4",
"X10. Z-19.989 K4",
"G0 X12.",
"Z0.996",
"X7.56",
"G33 Z-18.784 K4",
"X10. Z-20.004 K4",
"G0 X12.",
"Z1.011",
"X7.51",
"G33 Z-18.744 K4",
"X10. Z-19.989 K4",
"G0 X12.",
"Z0.997",
"X7.461",
"G33 Z-18.734 K4",
"X10. Z-20.003 K4",
"G0 X12.",
"Z1.011",
"X7.412",
"G33 Z-18.696 K4",
"X10. Z-19.989 K4",
"G0 X12.",
"Z0.997",
"X7.365",
"G33 Z-18.686 K4",
"X10. Z-20.003 K4",
"G0 X12.",
"Z1.01",
"X7.318",
"G33 Z-18.649 K4",
"X10. Z-19.99 K4",
"G0 X12.",
"Z0.997",
"X7.272",
"G33 Z-18.639 K4",
"X10. Z-20.003 K4",
"G0 X12.",
"Z1.01",
"X7.227",
"G33 Z-18.604 K4",
"X10. Z-19.99 K4",
"G0 X12.",
"Z0.997",
"X7.183",
"G33 Z-18.594 K4",
"X10. Z-20.003 K4",
"G0 X12.",
"Z1.01",
"X7.139",
"G33 Z-18.56 K4",
"X10. Z-19.99 K4",
"G0 X12.",
"Z0.997",
"X7.096",
"G33 Z-18.551 K4",
"X10. Z-20.003 K4",
"G0 X12.",
"Z1.01",
"X7.054",
"G33 Z-18.517 K4",
"X10. Z-19.99 K4",
"G0 X12.",
"Z0.998",
"X7.012",
"G33 Z-18.508 K4",
"X10. Z-20.002 K4",
"G0 X12.",
"Z1.01",
"X6.971",
"G33 Z-18.476 K4",
"X10. Z-19.99 K4",
"G0 X12.",
"Z0.998",
"X6.93",
"G33 Z-18.467 K4",
"X10. Z-20.002 K4",
"G0 X12.",
"Z1.009",
"X6.89",
"G33 Z-18.436 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.998",
"X6.85",
"G33 Z-18.427 K4",
"X10. Z-20.002 K4",
"G0 X12.",
"Z1.009",
"X6.811",
"G33 Z-18.396 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.998",
"X6.773",
"G33 Z-18.388 K4",
"X10. Z-20.002 K4",
"G0 X12.",
"Z1.009",
"X6.734",
"G33 Z-18.358 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.998",
"X6.697",
"G33 Z-18.35 K4",
"X10. Z-20.002 K4",
"G0 X12.",
"Z1.009",
"X6.659",
"G33 Z-18.321 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.998",
"X6.622",
"G33 Z-18.313 K4",
"X10. Z-20.002 K4",
"G0 X12.",
"Z1.009",
"X6.586",
"G33 Z-18.284 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.998",
"X6.55",
"G33 Z-18.276 K4",
"X10. Z-20.002 K4",
"G0 X12.",
"Z1.009",
"X6.514",
"G33 Z-18.248 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.999",
"X6.479",
"G33 Z-18.241 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.009",
"X6.444",
"G33 Z-18.213 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.999",
"X6.409",
"G33 Z-18.206 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.009",
"X6.375",
"G33 Z-18.179 K4",
"X10. Z-19.991 K4",
"G0 X12.",
"Z0.999",
"X6.34",
"G33 Z-18.172 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X6.307",
"G33 Z-18.145 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X6.273",
"G33 Z-18.138 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X6.24",
"G33 Z-18.112 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X6.207",
"G33 Z-18.105 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X6.175",
"G33 Z-18.079 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X6.143",
"G33 Z-18.072 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X6.11",
"G33 Z-18.047 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X6.079",
"G33 Z-18.04 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X6.047",
"G33 Z-18.016 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X6.016",
"G33 Z-18.009 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X5.985",
"G33 Z-17.984 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X5.954",
"G33 Z-17.978 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X5.924",
"G33 Z-17.954 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X5.893",
"G33 Z-17.947 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X5.863",
"G33 Z-17.924 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X5.833",
"G33 Z-17.917 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X5.804",
"G33 Z-17.894 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X5.774",
"G33 Z-17.888 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X5.745",
"G33 Z-17.865 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X5.716",
"G33 Z-17.859 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X5.687",
"G33 Z-17.836 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z0.999",
"X5.659",
"G33 Z-17.83 K4",
"X10. Z-20.001 K4",
"G0 X12.",
"Z1.008",
"X5.63",
"G33 Z-17.807 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z1",
"X5.602",
"G33 Z-17.801 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.008",
"X5.574",
"G33 Z-17.779 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z1",
"X5.546",
"G33 Z-17.773 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.008",
"X5.518",
"G33 Z-17.751 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z1",
"X5.49",
"G33 Z-17.746 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.008",
"X5.463",
"G33 Z-17.724 K4",
"X10. Z-19.992 K4",
"G0 X12.",
"Z1",
"X5.436",
"G33 Z-17.718 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.409",
"G33 Z-17.697 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.382",
"G33 Z-17.691 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.355",
"G33 Z-17.67 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.328",
"G33 Z-17.664 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.302",
"G33 Z-17.644 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.276",
"G33 Z-17.638 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.249",
"G33 Z-17.617 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.223",
"G33 Z-17.612 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.197",
"G33 Z-17.591 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.172",
"G33 Z-17.586 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.146",
"G33 Z-17.566 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.121",
"G33 Z-17.56 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.095",
"G33 Z-17.54 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.07",
"G33 Z-17.535 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1.007",
"X5.045",
"G33 Z-17.515 K4",
"X10. Z-19.993 K4",
"G0 X12.",
"Z1",
"X5.02",
"G33 Z-17.51 K4",
"X10. Z-20 K4",
"G0 X12.",
"Z1",
"X5.02",
"G33 Z-17.51 K4",
"X10. Z-20 K4",
"G0 X12.",
		
	};
*/
//#define _USEENCODER
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
#define LOOP_FROM 1
//#define LOOP_COUNT 2
#define LOOP_COUNT 4 //509//289 //158
//#define _USEENCODER // uncomment tihs define to use HW rotary encoder on spindle	
  /* USER CODE BEGIN 1 */
	#ifdef _SIMU
	int preload = LOOP_COUNT;
	#else
	int preload = 1;
	#endif

const char * ga1[] = {
"G95",
"G0 X10. Z0.",
"G0 X9.498",
"G33 Z-0.665 K2",
"X10. Z-1.02 K2",
};	
	/*	
const char * ga1[] = {
"G95",
"G0 X10. Z1.04",
"G1 Z-10 F0.1",
"G1 Z0 F0.1",

"G0 X10. Z1.04",
"G0 X9.498",
	todo два подряд G33(сама резьба+вывод резца из резьбы), замедлять нужно только в последнем, и синхронизация только по Z,
	т.е. для второй команды ход по X будет больше и она будет основной осью, соответственно нужно пересчитать итоговую скорость 
	так что бы скорость по Z осталась неизменной(значение К).
	Ну и доделать substep, сейчас он не работает корректно. а может он не особо то и нужен?
	"G33 Z-9.709 K3", 
"X10. Z-9.96 K3",
"G0 X12.",
"Z0.98",
"X9.289",
"G33 Z-9.665 K3",
"X10. Z-10.02 K3",
"G0 X12.",
"Z1.026",
"X9.13",
"G33 Z-9.539 K3",
"X10. Z-9.974 K3",
"G0 X12.",
"Z0.987",
"X8.995",
"G33 Z-9.511 K3",
"X10. Z-10.013 K3",
"G0 X12.",
"Z1.021",
"X8.876",
"G33 Z-9.417 K3",
"X10. Z-9.979 K3",
"G0 X12.",
"Z0.99",
"X8.769",
"G33 Z-9.395 K3",
"X10. Z-10.01 K3",
"G0 X12.",
"Z1.019",
"X8.671",
"G33 Z-9.317 K3",
"X10. Z-9.981 K3",
"G0 X12.",
"Z0.992",
"X8.579",
"G33 Z-9.297 K3",
"X10. Z-10.008 K3",
"G0 X12.",
"Z1.017",
"X8.493",
"G33 Z-9.229 K3",
"X10. Z-9.983 K3",
"G0 X12.",
"Z0.993",
"X8.411",
"G33 Z-9.212 K3",
"X10. Z-10.007 K3",
"G0 X12.",
"Z1.016",
"X8.334",
"G33 Z-9.151 K3",
"X10. Z-9.984 K3",
"G0 X12.",
"Z0.994",
"X8.259",
"G33 Z-9.135 K3",
"X10. Z-10.006 K3",
"G0 X12.",
"Z1.015",
"X8.188",
"G33 Z-9.079 K3",
"X10. Z-9.985 K3",
"G0 X12.",
"Z0.995",
"X8.12",
"G33 Z-9.065 K3",
"X10. Z-10.005 K3",
"G0 X12.",
"Z1.014",
"X8.054",
"G33 Z-9.013 K3",
"X10. Z-9.986 K3",
"G0 X12.",
"Z0.996",
"X7.99",
"G33 Z-8.999 K3",
"X10. Z-10.004 K3",
"G0 X12.",
"Z1.014",
"X7.928",
"G33 Z-8.951 K3",
"X10. Z-9.986 K3",
"G0 X12.",
"Z0.996",
"X7.868",
"G33 Z-8.938 K3",
"X10. Z-10.004 K3",
"G0 X12.",
"Z1.013",
"X7.81",
"G33 Z-8.892 K3",
"X10. Z-9.987 K3",
"G0 X12.",
"Z0.997",
"X7.753",
"G33 Z-8.88 K3",
"X10. Z-10.003 K3",
"G0 X12.",
"Z1.013",
"X7.697",
"G33 Z-8.836 K3",
"X10. Z-9.987 K3",
"G0 X12.",
"Z0.997",
"X7.643",
"G33 Z-8.825 K3",
"X10. Z-10.003 K3",
"G0 X12.",
"Z1.012",
"X7.59",
"G33 Z-8.783 K3",
"X10. Z-9.988 K3",
"G0 X12.",
"Z0.997",
"X7.539",
"G33 Z-8.772 K3",
"X10. Z-10.003 K3",
"G0 X12.",
"Z1.012",
"X7.488",
"G33 Z-8.732 K3",
"X10. Z-9.988 K3",
"G0 X12.",
"Z0.998",
"X7.438",
"G33 Z-8.721 K3",
"X10. Z-10.002 K3",
"G0 X12.",
"Z1.012",
"X7.389",
"G33 Z-8.683 K3",
"X10. Z-9.988 K3",
"G0 X12.",
"Z0.998",
"X7.341",
"G33 Z-8.673 K3",
"X10. Z-10.002 K3",
"G0 X12.",
"Z1.012",
"X7.294",
"G33 Z-8.636 K3",
"X10. Z-9.988 K3",
"G0 X12.",
"Z0.998",
"X7.248",
"G33 Z-8.626 K3",
"X10. Z-10.002 K3",
"G0 X12.",
"Z1.011",
"X7.202",
"G33 Z-8.59 K3",
"X10. Z-9.989 K3",
"G0 X12.",
"Z0.998",
"X7.158",
"G33 Z-8.58 K3",
"X10. Z-10.002 K3",
"G0 X12.",
"Z1.011",
"X7.114",
"G33 Z-8.546 K3",
"X10. Z-9.989 K3",
"G0 X12.",
"Z0.999",
"X7.07",
"G33 Z-8.536 K3",
"X10. Z-10.001 K3",
"G0 X12.",
"Z1.011",
"X7.027",
"G33 Z-8.503 K3",
"X10. Z-9.989 K3",
"G0 X12.",
"Z0.999",
"X6.985",
"G33 Z-8.494 K3",
"X10. Z-10.001 K3",
"G0 X12.",
"Z1.011",
"X6.944",
"G33 Z-8.461 K3",
"X10. Z-9.989 K3",
"G0 X12.",
"Z0.999",
"X6.903",
"G33 Z-8.452 K3",
"X10. Z-10.001 K3",
"G0 X12.",
"Z1.011",
"X6.862",
"G33 Z-8.42 K3",
"X10. Z-9.989 K3",
"G0 X12.",
"Z0.999",
"X6.822",
"G33 Z-8.412 K3",
"X10. Z-10.001 K3",
"G0 X12.",
"Z1.011",
"X6.783",
"G33 Z-8.381 K3",
"X10. Z-9.989 K3",
"G0 X12.",
"Z0.999",
"X6.744",
"G33 Z-8.373 K3",
"X10. Z-10.001 K3",
"G0 X12.",
"Z1.01",
"X6.705",
"G33 Z-8.342 K3",
"X10. Z-9.99 K3",
"G0 X12.",
"Z0.999",
"X6.667",
"G33 Z-8.334 K3",
"X10. Z-10.001 K3",
"G0 X12.",
"Z1.01",
"X6.629",
"G33 Z-8.304 K3",
"X10. Z-9.99 K3",
"G0 X12.",
"Z0.999",
"X6.592",
"G33 Z-8.297 K3",
"X10. Z-10.001 K3",
"G0 X12.",
"Z1.01",
"X6.555",
"G33 Z-8.268 K3",
"X10. Z-9.99 K3",
"G0 X12.",
"Z1",
"X6.519",
"G33 Z-8.26 K3",
"X10. Z-10 K3",
"G0 X12.",
"Z1.01",
"X6.483",
"G33 Z-8.231 K3",
"X10. Z-9.99 K3",
"G0 X12.",
"Z1",
"X6.447",
"G33 Z-8.224 K3",
"X10. Z-10 K3",
"G0 X12.",
"Z1.01",
"X6.412",
"G33 Z-8.196 K3",
"X10. Z-9.99 K3",
"G0 X12.",
"Z1",
"X6.377",
"G33 Z-8.189 K3",
"X10. Z-10 K3",
"G0 X12.",
"Z1.01",
"X6.342",
"G33 Z-8.161 K3",
"X10. Z-9.99 K3",
"G0 X12.",
"Z1",
"X6.308",
"G33 Z-8.154 K3",
"X10. Z-10 K3",
"G0 X12.",
"Z1.01",
"X6.274",
"G33 Z-8.127 K3",
"X10. Z-9.99 K3",
"G0 X12.",
"Z1",
"X6.24",
"G33 Z-8.12 K3",
"X10. Z-10 K3",
"G0 X12.",
"Z1",
"X6.24",
"G33 Z-8.12 K3",
"X10. Z-10 K3",
"G0 X12.",
};
	*/
	
	
/*
	static const char * ga2[] = {
	"G90 G95 G18",//async
//	"G1 X0. Z0. F500",

"G0 X18. Z0.05",
"G0 X15.9",
"G1 Z-20.8 F0.1",
"X16.1",
"G0 Z0.05",
"X15.7",
"G1 Z-20.8 F0.1",
"X15.9",
"X17.9 Z-19.8",
"G0 Z0.05",
"X15.5",
"G1 Z-20.8 F0.1",
"X15.7",
"X17.7 Z-19.8",
"G0 Z0.05",
"X15.3",
"G1 Z-20.8 F0.1",
"X15.5",
"X17.5 Z-19.8",
"G0 Z0.05",
"X15.1",
"G1 Z-20.8 F0.1",
"X15.3",
"X17.3 Z-19.8",
"G0 Z0.05",
"X14.9",
"G1 Z-20.8 F0.1",
"X15.1",
"X17.1 Z-19.8",
"G0 Z0.05",
"X14.7",
"G1 Z-20.8 F0.1",
"X14.9",
"X16.9 Z-19.8",
"G0 Z0.05",
"X14.5",
"G1 Z-20.8 F0.1",
"X14.7",
"X16.7 Z-19.8",
"G0 Z0.05",
"X14.3",
"G1 Z-20.8 F0.1",
"X14.5",
"X16.5 Z-19.8",
"G0 Z0.05",
"X14.1",
"G1 Z-20.8 F0.1",
"X14.3",
"X16.3 Z-19.8",
"G0 Z0.05",
"X13.9",
"G1 Z-20.8 F0.1",
"X14.1",
"X16.1 Z-19.8",
"G0 Z0.05",
"X13.7",
"G1 Z-20.8 F0.1",
"X13.9",
"X15.9 Z-19.8",
"G0 Z0.05",
"X13.5",
"G1 Z-20.8 F0.1",
"X13.7",
"X15.7 Z-19.8",
"G0 Z0.05",
"X13.3",
"G1 Z-20.8 F0.1",
"X13.5",
"X15.5 Z-19.8",
"G0 Z0.05",
"X13.1",
"G1 Z-20.8 F0.1",
"X13.3",
"X15.3 Z-19.8",
"G0 Z0.05",
"X12.9",
"G1 Z-20.8 F0.1",
"X13.1",
"X15.1 Z-19.8",
"G0 Z0.05",
"X12.7",
"G1 Z-20.8 F0.1",
"X12.9",
"X14.9 Z-19.8",
"G0 Z0.05",
"X12.5",
"G1 Z-20.8 F0.1",
"X12.7",
"X14.7 Z-19.8",
"G0 Z0.05",
"X12.3",
"G1 Z-20.8 F0.1",
"X12.5",
"X14.5 Z-19.8",
"G0 Z0.05",
"X12.1",
"G1 Z-20.8 F0.1",
"X12.3",
"X14.3 Z-19.8",
"G0 Z0.05",
"X11.9",
"G1 Z-20.8 F0.1",
"X12.1",
"X14.1 Z-19.8",
"G0 Z0.05",
"X11.7",
"G1 Z-20.8 F0.1",
"X11.9",
"X13.9 Z-19.8",
"G0 Z0.05",
"X11.5",
"G1 Z-20.8 F0.1",
"X11.7",
"X13.7 Z-19.8",
"G0 Z0.05",
"X11.3",
"G1 Z-20.8 F0.1",
"X11.5",
"X13.5 Z-19.8",
"G0 Z0.05",
"X11.1",
"G1 Z-20.8 F0.1",
"X11.3",
"X13.3 Z-19.8",
"G0 Z0.05",
"X10.9",
"G1 Z-20.8 F0.1",
"X11.1",
"X13.1 Z-19.8",
"G0 Z0.05",
"X10.7",
"G1 Z-20.8 F0.1",
"X10.9",
"X12.9 Z-19.8",
"G0 Z0.05",
"X10.5",
"G1 Z-20.8 F0.1",
"X10.7",
"X12.7 Z-19.8",
"G0 Z0.05",
"X10.3",
"G1 Z-20.8 F0.1",
"X10.5",
"X12.5 Z-19.8",
"G0 Z0.05",
"X10.15",
"G1 Z-20.8 F0.1",
"X10.3",
"X12.3 Z-19.8",
"G0 Z0.05",
"X10.",
"G1 Z-20.8 F0.1",
"X10.15",
"X12.15 Z-19.8",
"G0 X16.1",
"Z0.05",
"X18.",
		
		
	"G0 X0. Z0.",
	"X-1.",
	"G1 X0. Z-20. F1.5",
	"G0 X4",
	"Z0.",
		

//		"Z-.02",
//	"Z0. F200",
//	"Z-.02 F300",

//	"Z0. F400",
//	"Z-.02 F500",



	"Z-.02",
	"G95",

	"G1 Z0.",
	"Z-.02",
	"G94",

  "G1 Z0.",
	"Z-.02",


	"Z0.",
		
	"Z-.02",



		"G3 X12. Z-6 K-6",
		
	"X80. Z-30",
//	"G3 X94. Z-37 K-7",
		
	"X94. Z-42",
		"G3 X97.017 Z-42.915 I-2. K-5",
	//"G3 X100.561 Z-45.944 I-2. K-5", // r=5.385
		
		
	"X0. Z-30",
	"X80.",
	"G3 X94. Z-37 K-7", //r=7
	"G1 Z-42",
	//"G3 X100.561 Z-45.944 I-2. K-5", // r=5.385
	"G3 X97.017 Z-42.915 I-2. K-5",
	};
*/
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
	state_hw.function = do_fsm_menu_lps;
	
	cb_init_ref(&task_cb, task_size, sizeof(G_task_t), &gt);
//	cb_init_ref(&task_precalc_cb, task_precalc_size, sizeof(G_task_t), &gt_precalc);
//	cb_init_ref(&gp_cb, gp_size, sizeof(G_pipeline_t),&gp);
	cb_init_ref(&substep_cb, substep_size, sizeof(substep_t),&substep_delay);
	cb_init_ref(&substep_job_cb, substep_job_size, sizeof(substep_job_t),&substep_delay);

	cb_init_ref(&sma_cb,  8, sizeof(substep_sma_ref_t), &smaNumbers);
	cb_init_ref(&sma_substep_cb,  8, sizeof(uint32_t), &smaSubstepRefs);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//	LL_TIM_OC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH1,LL_TIM_OCPOLARITY_LOW);
//	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH1);
	
//	while(1);
	
//	LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
//	LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port, MOTOR_Z_STEP_Pin); 
//	LL_mDelay(50);
//	LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
//	LL_mDelay(50);


	/* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(USART2);
  /* Enable DMA Channel Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);

/* Enable RXNE and Error interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);

	if(LL_GPIO_IsInputPinSet(BUTTON_1_GPIO_Port, BUTTON_1_Pin)){
		demo = true;
	}
	// init I2C display: 
	#ifndef _SIMU
	Activate_I2C_Master();
	init_screen(I2C2);
	while(ubTransferComplete==0);
	update_screen();
	SSD1306_UpdateScreen();
	i2c_device_init(I2C2);
//	LL_mDelay(250);
	#endif
	init_buttons();

//	uint8_t  jdata[6];
//	while(1){
//		reqest_sample_i2c_dma(I2C2);	
//		Handle_I2C_Master_Receive(I2C2, i2c_device_id, jdata, 6, 10);
//		LL_mDelay(250);
//	}
	
//  LL_TIM_EnableIT_CC3(TIM4);													// enable interrupts for TACHO events from encoder
//  LL_TIM_EnableCounter(TIM4); 												//Enable timer 4
//	TIM4->SR = 0; 																			// reset interrup flags

	do_fsm_menu(&state_hw);
//	LED_OFF();


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
	G_task_t *precalculating_task = 0;
//	G_task_t *precalculating_task = cb_pop_front_ref2(&task_cb); // get ref to task to start precalculating process
//  memcpy(&state_precalc.current_task, precalculating_task, task_cb.sz);

	// init precalc
//	if(precalculating_task && precalculating_task->precalculate_init_callback_ref)
//		precalculating_task->precalculate_init_callback_ref(&state_precalc);

// do first step precalc
//	if(precalculating_task->precalculate_callback_ref)
//		precalculating_task->precalculate_callback_ref(&state_precalc);

	// start gcode execution
//	G94(&state_hw);
//	do_fsm_move_start2(&state_hw);
	int command = 0;
	int testcommand = 0;
//	debug();
//	LL_mDelay(50);

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
//		reqest_sample_i2c_dma(); // init reqest to joystick by DMA, when process_button complete i2c done its job
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);
//		process_G_pipeline();
		load_next_task(&state_hw);


		process_button();
//		process_joystick();
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);


//		uint8_t level = Thread_Info[Menu_Step].level;

//		if(auto_mode == true) {
//			if ( auto_mode_delay == 0 ) {
//				buttons_flag_set = single_click_Msk; //
//			}
//		}

		if(preload >= 0) {
//			if(task_cb.count2 < (task_cb.capacity - 1)) {
				if(task_cb.count < (task_cb.capacity - 2) &&  preload-- >= 0){
					command_parser((char *)ga1[command++]);
				}
//			}
			switch(buttons_flag_set) {
				case single_click_Msk:
					// emulate receive g-code line by usart interrupt(bluetooth) 
					switch(testcommand) {
						case 0:
							command_parser("X2. Z20. F640");
		//					command_parser("X0.666");
		//					command_parser("Z-10. F180");
							break;
						case 1:
							command_parser("X0.");
		//					command_parser("Z0.");
							break;
						case 2:
							command_parser("X0.333");
							break;
						case 3:
							command_parser("X0.");
							break;
						case 4:
							command_parser("X1. Z10.");
							break;
						case 5:
							command_parser("X0. Z0.");
							break;
					} 
					testcommand++;
					if(testcommand > 5 )
						testcommand = 0;
				
//				if(feed_direction == feed_direction_left)
//						command_parser("Z-10. F1");
//						command_parser("Z-10. F240");
//					else
//						command_parser("Z0.");

					feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
					menu_changed = 1;
					break;
			}//
			buttons_flag_set = 0; // reset button flags
		} else {
//			command = LOOP_FROM;
//			preload = LOOP_COUNT -1;
		}
#ifdef _SIMU
#endif
		
#ifndef _SIMU
//		if(buttons_flag_set) {
//			do_fsm_menu(&state_hw);
//			buttons_flag_set = 0; // reset button flags
//		}

//		if(z_axis.ramp_step != rs) {
//			rs = z_axis.ramp_step;
//			menu_changed = 1;
//		}

//		if(z_axis.current_pos != rs) {
//			rs = z_axis.current_pos;
//			menu_changed = 1;
//		}

// update display info
		if(menu_changed == 1){
			menu_changed = update_screen();
//			SSD1306_UpdateScreen();
		}
#endif		
		
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

  /* I2C2_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* I2C2 interrupt Init */
  NVIC_SetPriority(I2C2_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(I2C2_EV_IRQn);
  NVIC_SetPriority(I2C2_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  /** I2C Initialization 
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
  TIM_InitStruct.Autoreload = 0xFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
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
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
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

	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM3,LL_TIM_CHANNEL_CH3);	
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
  PB4   ------> TIM3_CH1 
  */
  GPIO_InitStruct.Pin = MOTOR_X_STEP_Pin|MOTOR_Z_STEP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_RemapPartial_TIM3();

}
#ifdef _USEENCODER
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
#else
static void MX_TIM4_Init(void)
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
#endif

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
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 1));
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
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
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
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 1));
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
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, MOTOR_Z_ENABLE_Pin|MOTOR_Z_DIR_Pin|MOTOR_X_DIR_Pin);

  /**/
  LL_GPIO_ResetOutputPin(MOTOR_X_ENABLE_GPIO_Port, MOTOR_X_ENABLE_Pin);

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
  GPIO_InitStruct.Pin = MOTOR_Z_ENABLE_Pin|MOTOR_Z_DIR_Pin|MOTOR_X_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_10 
                          |LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MOTOR_X_ENABLE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(MOTOR_X_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14 
                          |LL_GPIO_PIN_15|LL_GPIO_PIN_3|LL_GPIO_PIN_5|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUTTON_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(BUTTON_1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUTTON_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BUTTON_2_GPIO_Port, &GPIO_InitStruct);

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
