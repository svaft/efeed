/*
 * buttons.h
 *
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "stm32f1xx_hal.h"


#define BT_TOTAL	4

#define BB_VAR   0x20004000
//#define BB_VAR 0x20004000
#define BITBAND_SRAM_REF   0x20000000
#define BITBAND_SRAM_BASE  0x22000000
#define BITBAND_SRAM(a,b) ((BITBAND_SRAM_BASE + (a-BITBAND_SRAM_REF)*32 + (b*4)))  // Convert SRAM address


// Button timing variables
#define DEBOUNCE_MS 20                 // ms debounce period to prevent flickering when pressing or releasing the button
#define DOUBLECLICK_GAP_MS 150                       // max ms between clicks for a double click event
#define HOLDTIME_MS 500                // ms hold period: how long to wait for press+hold event
#define CLICKTIME_MS 250 
//#define LONGHOLDTIME_MS 3000       // ms long hold period: how long to wait for press+hold event

#define long_press_start			buttons_flag_setbb[0]
#define long_press_start_Pos	(0U)
#define long_press_start_Msk  (0x1U << long_press_start_Pos)

#define long_press_end        buttons_flag_setbb[1]
#define long_press_end_Pos    (1U)
#define long_press_end_Msk    (0x1U << long_press_end_Pos)

#define single_click          buttons_flag_setbb[2]
#define single_click_Pos      (2U)
#define single_click_Msk      (0x1U << single_click_Pos)

#define double_click          buttons_flag_setbb[3]
#define double_click_Pos      (3U)
#define double_click_Msk      (0x1U << double_click_Pos)

// events for 2nd button
#define long_press_start2			buttons_flag_setbb[4]
#define long_press_start_Pos2	(4U)
#define long_press_start_Msk2  (0x1U << long_press_start_Pos2)

#define long_press_end2        buttons_flag_setbb[5]
#define long_press_end_Pos2    (5U)
#define long_press_end_Msk2    (0x1U << long_press_end_Pos2)

#define single_click2          buttons_flag_setbb[6]
#define single_click_Pos2      (6U)
#define single_click_Msk2      (0x1U << single_click_Pos2)

#define double_click2          buttons_flag_setbb[7]
#define double_click_Pos2      (7U)
#define double_click_Msk2      (0x1U << double_click_Pos2)


typedef struct
{
	GPIO_TypeDef *GPIOx;
	uint16_t button_pin;
	uint32_t downTime;               // time the button was pressed down
	uint32_t buttons, buttons_mstick, buttons_flag, buttons_mask, clk_mode;
} BUTTON;

extern BUTTON bt[BT_TOTAL];

extern uint32_t buttons_flag_set;//  __attribute__((at(BB_VAR)));
extern __IO uint8_t  ubTransferComplete;


//#define buttons_flag_setbb ((BITBAND_SRAM_BASE + (&buttons_flag_set-BITBAND_SRAM_REF)*32))  // Convert SRAM address
#define buttons_flag_setbb ((uint32_t *)((0x22000000  + ((BB_VAR)-0x20000000)*32)))

//#define buttons_flag_setbb ((uint32_t *)((0x22000000  + ((BB_VAR)-0x20000000)*32)))
//uint32_t gap = 0;


void init_buttons(void);
void process_button(void);
void process_joystick(void);
//uint32_t downTime = 0;               // time the button was pressed down
//uint32_t buttons = 0, buttons_mstick = 0, buttons_flag = 0, buttons_mask = 0;






#endif /* BUTTONS_H_ */
