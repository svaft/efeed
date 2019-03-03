/*
 * buttons.h
 *
 */

#ifndef FSM_H_
#define FSM_H_

#include "main.h"

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
	
	
	
  state_func_t function;
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
	int dx,dz;//,d,d1,d2;
//	int sx,sz;
	int err;
  int i, x,z;
//  state_func_t set_pulse_function;
} state_t;

extern state_t state;

void do_fsm_menu(state_t*);										//0 . menu mode, if long_press_start event: go to sub-menu or up-menu, DOUBLE_CLICK: initial direction change
void do_fsm_menu_lps(state_t*);							//10. long_press_start: end_pos = current_pos = 0, идем в п. fsm_first_cut_lps
/*
void do_fsm_first_cut_lps(state_t*);						//20. init selected mode, init direction, motor on, goto fsm_wait_tacho
void do_fsm_wait_tacho(state_t*); 						//24. wait tacho pulse, go to 25
void do_fsm_first_cut_ramp_up(state_t*); 			//25. tacho pulse interrupt: включаем прерывание по тикам энкодера, начинаем разгоняться(ramp up) по таблице пока не выйдем на расчетную скорость,далее в режим 26
void do_fsm_first_cut_main_part(state_t*); 		//26. step until long_press_end event, then go to 27
void do_fsm_first_cut_lpe(state_t*);					//27. long_press_end event: проверяем общее расчетное количество шагов(разгон+infeed+основной путь+торможение), при необходимости делаем дошагиваем до кратного целого в зависимости от микрошага, далее в режим торможения, п. 30
void do_fsm_first_cut_ramp_down(state_t*);		//30. режим торможения(ramp down) пока по таблице разгона не дойдем обратно до нуля, останавливаем мотор, end_pos = current_pos, меняем направление, обновляем экран, идем в п.35
void do_fsm_wait_sclick(state_t*);          	//35. ждем SINGLE_CLICK: если current_pos > 0 ? идем в mode = 40 иначе в	mode = 50
void do_fsm_sclick_event(state_t*);         	//40. клик: включаем моторы обратно, идем в п.45
void do_fsm_main_cut_back_ramp_up(state_t*);	//45. если счетчик current_pos > 0 то едем обратно до нуля: разгон
void do_fsm_main_cut_back(state_t*);					//46. main path back to initial position. If long_press_start detected during process, activate prolonged mode ( 48).
void do_fsm_main_cut_back_ramp_down(state_t*);//47. аналогично 27, торможение, останавливаем мотор, меняем направление, обновляем экран, идем в п.35
void do_fsm_main_cut_back_prolong(state_t*);	//48. prolonged mode used to extend cutting path until long_press released. step back until current_pos reach start position add full revolution steps of servo. when released go back to 46
void do_fsm_main_cut_wait_tacho(state_t*);		//50. клик: включаем моторы вперед, ждем тахо, идем в п.52
void do_fsm_main_cut_ramp_up(state_t*);				//54. тахо пульс обнаружен, включаем прерывание по тикам энкодера, можно шагать, идем в п.55
void do_fsm_main_cut(state_t*);								//55. если счетчик current_pos = 0 то в зависимости от выбранной стратегии вычисляем infeed и идем в режим резьбы до end_pos: разгон, далее идем в п.56
void do_fsm_main_cut_infeed(state_t*);				//56. infeed для резьбы: в зависимости от номера прохода сдвигаем каретку на определенное количество шагов для облегчения резания+основной путь, далее в п. 30
*/
void z_move(uint32_t , uint32_t  , bool, bool);


void do_fsm_ramp_up2(state_t* );
void do_fsm_move2(state_t*);
void do_fsm_ramp_down2(state_t* );
void do_fsm_move_end2(state_t* );


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

// TIM3->CCER register bitbang access:
#define t3ccer			((uint32_t *)((0x42000000  + ((0x40000420)-0x40000000)*32)))


__STATIC_INLINE void dxdz_callback(state_t* s){
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int e2 = s->err;
	if (e2 > -s->dx)	{ // step X axis
		s->err -= s->dz; 
		t3ccer[TIM_CCER_CC1E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
	}
	if (e2 < s->dz)	{ // step Z axis
		s->err += s->dx;
		t3ccer[TIM_CCER_CC3E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
	}
}

//void dxdz_callback(state_t* );
void G01(int dx, int dz, int feed);


#endif /* FSM_H_ */
