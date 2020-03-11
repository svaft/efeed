/*
 * buttons.h
 *
 */

#ifndef FSM_H_
#define FSM_H_

#include "main.h"
#include "gcode.h"
//#define debug()	LL_GPIO_TogglePin(MOTOR_X_ENABLE_GPIO_Port, MOTOR_X_ENABLE_Pin)
//#define debug1() LL_GPIO_TogglePin(MOTOR_Z_DIR_GPIO_Port, MOTOR_Z_DIR_Pin)
//#define debug7() LL_GPIO_TogglePin(MOTOR_X_DIR_GPIO_Port, MOTOR_X_DIR_Pin)

struct state;

void do_fsm_menu(state_t*);										//0 . menu mode, if long_press_start event: go to sub-menu or up-menu, DOUBLE_CLICK: initial direction change
void do_fsm_menu_lps(state_t*);							//10. long_press_start: end_pos = current_pos = 0, идем в п. fsm_first_cut_lps
void z_move(uint32_t , uint32_t  , bool, bool);



void do_fsm_move_start(state_t* );
void do_fsm_ramp_up(state_t* );
void do_fsm_move(state_t*);
void do_fsm_ramp_down(state_t* );
void do_fsm_move_end(state_t* );

void do_long_press_end_callback(state_t* );


void dwell_callback(state_t* s);
void P04init_callback(state_t* s);


void do_fsm_dwell(state_t*);

// TIM3->CCER register bitbang access:
#define t3ccer			((uint32_t *)((0x42000000  + ((0x40000420)-0x40000000)*32)))



//void dxdz_callback(state_t* );
void G01(int dx, int dz, int feed);


#endif /* FSM_H_ */
