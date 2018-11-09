#include "fsm.h"
#include "buttons.h"


uint32_t ramp[]= {
	0x1E000000,
//      0x00000000, // zero delay to disable ramp up
	0x15E353F7,
	0x110624DD,
	0x0E67A909,
	0x0CB5D163,
	0x0B7FEE35,
	0x0A946A82,
	0x09D9A0F5,
	0x0940CD81,
	0x08C0C265,
	0x0853743B,
	0x07F4B905,
	0x07A19758,
	0x0757DEEB,
	0x0715E90F,
	0x06DA701C,
	0x06A47489,
	0x06732AAA,
	0x0645EDE1,
	0x061C377E,
	0x05F59819,
	0x05D1B2A3,
	0x05B038B1,
	0x0590E7A4,
	0x0573867F,
	0x0557E426,
	0x053DD60D,
	0x0525371D,
	0x050DE6D9,
	0x04F7C8A5,
	0x04E2C336,
	0x04CEC017,
	0x04BBAB40,
	0x04A972C8,
	0x04980698,
	0x04875834,
	0x04775A84,
	0x046801AD,
	0x045942E9,
	0x044B1467,
	0x043D6D31,
	0x04304514,
	0x0423948C,
	0x041754AE,
	0x040B7F1D,
	0x04000DF9,
};
#define ramp_map 5

/*//main Finite-state machine(Nondeterministic finite automaton):
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
extern bool feed_direction;
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)
extern const uint8_t Menu_size;
extern TIM_HandleTypeDef htim3;

extern uint32_t infeed_step;
extern uint32_t infeed_steps;


uint32_t infeed_step = 0;
uint32_t infeed_steps = 10;


uint16_t infeed_map[]= {
	0,
	188,
	264,
	322,
	370,
	413,
	452,
	488,
	521,
	552,
	582,
	610,
	637,
	662,
	687,
	711,
	734,
	756,
};



_Bool z_axis_ramp_up(void)
{
	const fixedptu  set_with_fract = ramp[z_axis.ramp_step];
	if(z_axis.Q824set > set_with_fract || z_axis.ramp_step == ramp_map ) { // reach desired speed or end of ramp map
		TIM4->ARR = fixedpt_toint(z_axis.Q824set) - 1; // update register ARR
		TIM4->EGR |= TIM_EGR_UG;
		z_axis.fract_part = fixedpt_fracpart(z_axis.Q824set); // save fract part for future use on next step
		return true;
	} else {
		z_axis.ramp_step++;
		TIM4->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
		TIM4->EGR |= TIM_EGR_UG;
//		z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	}
	return false;
}

_Bool z_axis_ramp_down(void)
{
	if (z_axis.ramp_step == 0)
		return true;
	const fixedptu set_with_fract = ramp[--z_axis.ramp_step];
	TIM4->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
	TIM4->EGR |= TIM_EGR_UG;
//	z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	if(z_axis.ramp_step == 0)
		return true;
	return false;
}

void z_axis_move(void)
{
	const fixedptu set_with_fract = fixedpt_add(z_axis.Q824set, z_axis.fract_part); // calculate new step delay with fract from previous step
	TIM4->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
	TIM4->EGR |= TIM_EGR_UG;
	z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
}


void z_axis_at_move_end(state_t* s)
{
	disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
	//      MOTOR_Z_Disable(); //disable motor later on next tacho event (or after some ticks count?) to completely process last step
//	if(auto_mode == true)    auto_mode_delay = auto_mode_delay_ms; // reengage auto mode
	feed_direction = !feed_direction; //change feed direction
	menu_changed = 1; //update menu
	s->function = do_fsm_wait_sclick;
//	z_axis.mode = fsm_wait_sclick; // dummy mode
}




void do_fsm_wait_tacho(state_t* s)
{
	if(s->f_tacho) { // if tacho event
		s->function = do_fsm_first_cut_ramp_up;
//		infeed_step = 0; todo
		LED_GPIO_Port->BRR = LED_Pin; //led on
		TIM4->ARR = 1; // start stepper motor ramp up procedure immediately after tacho event
		TIM4->EGR |= TIM_EGR_UG;
		TIM4->CNT = 0;
		enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
	}
}

void do_fsm_menu(state_t* s)
{
	uint8_t level = Thread_Info[Menu_Step].level;
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

			// first pass of thread cut was complete, so just use single click
			//	to switch between modes to process all other cuts
			MOTOR_Z_Enable(); // time to wakeup motor from sleep is quite high(1.7ms), so enable it as soon as possible
			for(unsigned int i=0; i<(72*1700/16); i++); // wait 1700us delay to wakeup motor driver
			s->function = z_axis.current_pos > 0 ? do_fsm_sclick_event : do_fsm_main_cut_wait_tacho;
		} else { // controller in initial state, scroll menu
			s->function = do_fsm_menu_lps;
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
		if(s->function == do_fsm_menu_lps){
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

				s->function = do_fsm_wait_tacho; // go straight to 24 to wait tacho
			} else { // goto submenu
				for (int a = 0; a<Menu_size; a++) {
					if(Thread_Info[a].level == Thread_Info[Menu_Step].submenu) {
						Menu_Step = a;
						menu_changed = 1;
						break;
					}
				}
			}
		} else if(s->function == do_fsm_main_cut_back){
			s->function = do_fsm_main_cut_back_prolong; // go to 48 mode to add threads until long_press end
		}
		break;
	}
	case long_press_end_Msk: {
		if(s->function == do_fsm_first_cut_main_part){
			s->function = do_fsm_first_cut_lpe;
			break;
		}
		if(s->function == do_fsm_main_cut_back_prolong){ // end of prolonged mode
			s->function = do_fsm_main_cut_back;
			break;
		}
		/*
		switch(z_axis.mode) {
		case fsm_first_cut_main_part: {
//																					if(auto_mode == true){
//																									auto_mode_delay = auto_mode_delay_ms; //engage countdown timer to auto generate click event
//																					}
//																					Q824count = 0;
			s->function = do_fsm_first_cut_lpe;
			break;
		}
		case fsm_main_cut_back_prolong: { // end of prolonged mode
			s->function = do_fsm_main_cut_back;
			break;
		}
		}
		*/
		break;
	}
	}
}

void do_fsm_menu_lps(state_t* s)
{
}



void do_fsm_first_cut_lps(state_t* s)  // not used?
{
	disable_encoder_ticks(); //reset interrupt for encoder ticks, only tacho
	MOTOR_Z_Enable();
	if(feed_direction)
		MOTOR_Z_Forward();
	else
		MOTOR_Z_Reverse();
	s->function = do_fsm_wait_tacho; //intermediate state to wait tacho pulse.
	s->function = do_fsm_wait_tacho;
}






void do_fsm_first_cut_ramp_up(state_t* s)          // direct movement: first pass, thread recording: ramp up: accel by ramp map
{
	MOTOR_Z_SetPulse();
	z_axis.current_pos++;
	if(z_axis_ramp_up()) {
		s->function = do_fsm_first_cut_main_part;
		LED_GPIO_Port->BSRR = LED_Pin; //led off
	}
}


void do_fsm_first_cut_main_part(state_t* s)          // direct movement: first pass, thread recording: main part
{
	MOTOR_Z_SetPulse();
	z_axis.current_pos++;
	z_axis_move();
}


void do_fsm_first_cut_lpe(state_t* s)          // direct movement: first pass, thread recording: post-main part
{
	// для 1/2 микрошага нужно что бы общее количество шагов в цикле резьбы было кратно 2,(для 1/4 кратно 4 и тп).
	// это нужно для того что бы в конце шаговый мотор остановился на одном из двухсот устойчивых шагов,
	// не перескакивая на соседние шаги при потере питания.
	// поэтому проверяем общее количество на четность(0й бит), если нечетное число делаем еще один шаг,
	// иначе начинаем замедляться
	MOTOR_Z_SetPulse();
	z_axis.current_pos++;
	uint32_t all_count = z_axis.ramp_step + z_axis.current_pos - 1;
	uint32_t masked_count = all_count & ~(step_divider - 1);
	if(masked_count != all_count) {
		z_axis_move();
	} else {
		if(z_axis_ramp_down()) {
			z_axis.end_pos = z_axis.current_pos;
			z_axis_at_move_end(s);
		} else {
			s->function = do_fsm_first_cut_ramp_down;
		}
	}
}






void do_fsm_first_cut_ramp_down(state_t* s)          // direct movement: ramp down: deccel part + stop
{
	MOTOR_Z_SetPulse();
	z_axis.current_pos++;
	if(z_axis_ramp_down()) {
		z_axis.end_pos = z_axis.current_pos;
		z_axis_at_move_end(s);
	}
}



void do_fsm_sclick_event(state_t* s)   // reverse movement: set direction for motor
{
	if(feed_direction)
		MOTOR_Z_Forward();
	else
		MOTOR_Z_Reverse();
	enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
	s->function = do_fsm_main_cut_back_ramp_up;
}





void do_fsm_main_cut_back_ramp_up(state_t* s)          // reverse movement: ramp up: accel part
{
	MOTOR_Z_SetPulse();
	--z_axis.current_pos;
	if(z_axis_ramp_up())
		s->function = do_fsm_main_cut_back;
}







void do_fsm_main_cut_back(state_t* s)          // reverse movement: main part
{
	MOTOR_Z_SetPulse();
	if( --z_axis.current_pos > z_axis.ramp_step ) {
	} else {
		s->function = do_fsm_main_cut_back_ramp_down;
	}
}



void do_fsm_main_cut_back_ramp_down(state_t* s)   // reverse movement: ramp down: deccel part + stop
{
	if (z_axis.current_pos > 0) {
		MOTOR_Z_SetPulse();
		--z_axis.current_pos;
	}
	if(z_axis_ramp_down()) {
		z_axis_at_move_end(s);
	}
}








void do_fsm_main_cut_back_prolong(state_t* s)   // reverse movement: main part with prolong activated. todo split it with 46 mode?
{
	MOTOR_Z_SetPulse();
	--z_axis.current_pos;
	if(z_axis.current_pos == z_axis.ramp_step) { // we reach end of main path and have long_pressed key, so just add additional thread full turn to shift initial start point
		z_axis.prolong_fract += z_axis.prolong_addSteps; // fract part from prev step
		uint32_t prolong_fixpart = z_axis.prolong_fract >> 24;
		z_axis.current_pos += prolong_fixpart; // add fixed part
		z_axis.end_pos += prolong_fixpart;
		z_axis.prolong_fract &= FIXEDPT_FMASK; // leave fract part to accumulate with next dividing cycle
		// when long_press end, get back to 46 mode to proceed
	}
}

void do_fsm_main_cut_wait_tacho(state_t* s)   // direct movement: set direction for motor
{
	if(feed_direction)
		MOTOR_Z_Forward();
	else
		MOTOR_Z_Reverse();
	s->function = do_fsm_main_cut_ramp_up; // intermediate state to wait tacho pulse
	disable_encoder_ticks(); // reset interrupt for encoder ticks, only tacho
}

void do_fsm_main_cut(state_t* s)   // direct movement: ramp up: accel by ramp map
{
	MOTOR_Z_SetPulse();
	z_axis.current_pos++;
	if(z_axis_ramp_up()) {
		LED_GPIO_Port->BSRR = LED_Pin;   // led off
		s->function = do_fsm_main_cut_infeed;
	}
}


void do_fsm_main_cut_infeed(state_t* s)   // direct movement: main part
{
	MOTOR_Z_SetPulse();
	z_axis.current_pos++;
	if( z_axis.current_pos < ( z_axis.end_pos - z_axis.ramp_step ) ) {
		z_axis_move();
	} else {
		s->function = do_fsm_first_cut_ramp_down;
	}
}


void do_fsm_wait_sclick(state_t* s)
{
	MOTOR_Z_Disable(); //disable motor
}

void do_fsm_main_cut_ramp_up(state_t* s)
{
	s->function = do_fsm_main_cut;
	//reinit counter
//                          TIM4->ARR = fixedpt_toint(Q824set) - 1;
	LED_GPIO_Port->BRR = LED_Pin; //led on

	if(infeed_step < infeed_steps) {
		TIM4->ARR = infeed_map[infeed_step++] + 1; // start stepper motor with shifted position by infeed map
	} else {
		TIM4->ARR = 1;  // start stepper motor ramp up procedure immediately after tacho event
	}
	TIM4->EGR |= TIM_EGR_UG;

	TIM4->CNT = 0;
	enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
}
