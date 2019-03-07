/*
 * buttons.h
 *
 */

#ifndef FSM_H_
#define FSM_H_

#include "main.h"
#include "gcode.h"
#define debug()	LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin)

struct state;
typedef void (*state_func_t)( struct state* );

typedef struct state
{
	uint32_t steps_to_end;
	uint32_t current_pos;
	uint32_t end_pos;
	uint8_t ramp_step;
	uint32_t Q824set; // feed rate
	uint32_t fract_part; // Q8.24 format fract part
	
	G_task current_task;
	bool G94G95; // 0 - unit per min, 1 - unit per rev
	
  state_func_t function;
//  callback_func_t callback;
	uint32_t async_z;
	uint8_t z_period;
	bool f_encoder;
	bool f_tacho;
	bool spindle_dir;
	_Bool sync;
	_Bool main_feed_direction;
	TIM_TypeDef *syncbase;
  // other stateful data

	fixedpt state_X, state_Z; // absolute position
  // X axis
//  state_func_t function_x;
//	_Bool sync_x;
//	TIM_TypeDef *syncbase_x;
//	uint8_t x_period;

	//	bresenham
//	int dx,dz;//,d,d1,d2;
//	int rr, inc_dec;
//	int sx,sz;
	int err;
//  int i, x,z;
//  state_func_t set_pulse_function;
} state_t;


extern state_t state;

void do_fsm_menu(state_t*);										//0 . menu mode, if long_press_start event: go to sub-menu or up-menu, DOUBLE_CLICK: initial direction change
void do_fsm_menu_lps(state_t*);							//10. long_press_start: end_pos = current_pos = 0, идем в п. fsm_first_cut_lps
void z_move(uint32_t , uint32_t  , bool, bool);


void do_fsm_ramp_up2(state_t* );
void do_fsm_move2(state_t*);
void do_fsm_ramp_down2(state_t* );
void do_fsm_move_end2(state_t* );
void load_next_task(void);

void do_fsm_move_start(state_t* );
void do_fsm_ramp_up(state_t* );
void do_fsm_move(state_t*);
void do_fsm_ramp_down(state_t* );
void do_fsm_move_end(state_t* );

void do_long_press_end_callback(state_t* );


_Bool z_axis_ramp_up2(state_t* );
_Bool z_axis_ramp_down2(state_t* );
void z_axis_move2(state_t* );



void do_fsm_ramp_up_async(state_t* );
void do_fsm_move_async(state_t*);
void do_fsm_ramp_down_async(state_t* );
_Bool z_axis_ramp_up_async(state_t* );
_Bool z_axis_ramp_down_async(state_t* );

void dzdx_init(int dx, int dz, state_t* s);



void arc_dx_callback(void);// arc movement callback
void arc_dz_callback(void); // arc movement callback
void dxdz_callback(void); // line movement callback
void dwell_callback(void);
void P04init_callback(void);
void do_fsm_dwell(state_t*);

// TIM3->CCER register bitbang access:
#define t3ccer			((uint32_t *)((0x42000000  + ((0x40000420)-0x40000000)*32)))



//void dxdz_callback(state_t* );
void G01(int dx, int dz, int feed);


#endif /* FSM_H_ */
