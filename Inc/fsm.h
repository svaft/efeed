/*
 * buttons.h
 *
 */

#ifndef FSM_H_
#define FSM_H_

#include "main.h"

struct state;
typedef void (*state_func_t)( struct state* );

typedef struct state
{
// Z axis
  state_func_t function;
	uint32_t async_z;
	uint32_t current_pos;
	uint8_t z_period;
	_Bool f_encoder;
	_Bool f_tacho;
	_Bool sync;
	_Bool main_feed_direction;
	TIM_TypeDef *syncbase;
  // other stateful data

  // X axis
  state_func_t function_x;
	_Bool sync_x;
	TIM_TypeDef *syncbase_x;
	uint8_t x_period;

} state_t;

extern state_t state;

void do_fsm_menu(state_t*);										//0 . menu mode, if long_press_start event: go to sub-menu or up-menu, DOUBLE_CLICK: initial direction change
void do_fsm_menu_lps(state_t*);							//10. long_press_start: end_pos = current_pos = 0, идем в п. fsm_first_cut_lps
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



void do_fsm_move_start(state_t* );
void do_fsm_ramp_up(state_t* );
void do_fsm_move(state_t*);
void do_fsm_ramp_down(state_t* );
void do_fsm_move_end(state_t* );

_Bool z_axis_ramp_up2(state_t* );
_Bool z_axis_ramp_down2(state_t* );
void z_axis_move2(state_t* );



void do_fsm_ramp_up_async(state_t* );
void do_fsm_move_async(state_t*);
void do_fsm_ramp_down_async(state_t* );
_Bool z_axis_ramp_up_async(state_t* );
_Bool z_axis_ramp_down_async(state_t* );



#endif /* FSM_H_ */
