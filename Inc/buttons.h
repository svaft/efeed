/*
 * buttons.h
 *
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "stm32f1xx_hal.h"

void process_button(void);


// Button timing variables
#define debounce 20                 // ms debounce period to prevent flickering when pressing or releasing the button
#define DCgap 150                       // max ms between clicks for a double click event
#define holdTime 500                // ms hold period: how long to wait for press+hold event
#define clickTime 250 
#define longHoldTime 3000       // ms long hold period: how long to wait for press+hold event

#define long_press_start        buttons_flag_setbb[0]
#define long_press_start_Pos        (0U)
#define long_press_start_Msk        (0x1U << long_press_start_Pos)

#define long_press_end                          buttons_flag_setbb[1]
#define long_press_end_Pos          (1U)
#define long_press_end_Msk          (0x1U << long_press_end_Pos)

#define single_click                                        buttons_flag_setbb[2]
#define single_click_Pos                        (2U)
#define single_click_Msk                        (0x1U << single_click_Pos)

#define double_click                                        buttons_flag_setbb[3]
#define double_click_Pos                        (3U)
#define double_click_Msk                        (0x1U << double_click_Pos)


typedef struct
{
        uint32_t downTime;               // time the button was pressed down
        uint32_t buttons, buttons_mstick, buttons_flag, buttons_mask, clk_mode;
} BUTTON;

extern BUTTON bt;

extern uint32_t buttons_flag_set;//  __attribute__((at(0x20004000)));
#define buttons_flag_setbb ((uint32_t *)((0x22000000  + ((0x20004000)-0x20000000)*32)))
//uint32_t gap = 0;


//uint32_t downTime = 0;               // time the button was pressed down
//uint32_t buttons = 0, buttons_mstick = 0, buttons_flag = 0, buttons_mask = 0;






#endif /* BUTTONS_H_ */
