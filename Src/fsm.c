#include "fsm.h"
#include "buttons.h"
#include "stdlib.h"

#define steps_z 300
#define steps_x 1

// precalculated ramp for  100 000Hz frequency timer with prescaler of 720, values in TAB is ARR registed value.
// for details see file rampup.xlsx
uint8_t async_ramp_profile[]= {
	250,
	217,
	152,
	94,
	79,
	66,
	57,
	50,
	46,
	42,
	39,
	36,
	34,
	33,
	31,
	30,
	29,
	28,
	27,
	26,
	25,
	24,
	24,
	23,
	23,
	22,
	22,
	21,
	21,
	20,
	20,
	20,
};

#define async_ramp_profile_len 32
#define slew_speed_period 50 // 22


#define sync_ramp_profile_len 5
uint8_t sync_ramp_profile[]= {
	0xFF,
	0x1E,
	0x15,
	0x11,
	0x0E,
	0x0C,
	0x0B,
	0x0A,
	0x09,
};

extern bool feed_direction;
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)
extern const uint8_t Menu_size;

bool demo;

void do_fsm_menu(state_t* s)
{
	uint8_t level = Thread_Info[Menu_Step].level;
#ifdef _SIMU
	buttons_flag_set = long_press_start_Msk;
#endif	
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

//			z_move(feed_direction, z_axis.end_pos, s->main_feed_direction == feed_direction ? true : false, true);
			if(demo)
				z_move(feed_direction, z_axis.end_pos, false, true); //test case, always async
			else
				z_move(feed_direction, z_axis.end_pos, s->main_feed_direction == feed_direction ? true : false, true);
//			z_move(feed_direction, 400*2, false, true);
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

				z_axis.Q824set = Thread_Info[Menu_Step].Q824;
				const uint64_t upl = (uint64_t)3600 << 48; //calculate some constants for prolong mode
				z_axis.prolong_addSteps = upl / (fixedptud)z_axis.Q824set;
				// 200*step_divider*z_feed_screw(mm)*len(mm) = desired length in steps, in my case its 200*2*1*x

				
				MOTOR_X_Enable();
				MOTOR_Z_Enable(); // time to wakeup motor from sleep is quite high(1.7ms), so enable it as soon as possible
//				LL_mDelay(2);
				if(demo){
					G01(steps_x,steps_z,0);
//					z_move(feed_direction, steps, false, true); //test case, move async 10mm
//					z_move(feed_direction, 31, false, true); //test case, move async 10mm
				}
				else
					z_move(feed_direction, 0, true, true);

				//do_fsm_move_start
			} else { // goto submenu
				for (int a = 0; a<Menu_size; a++) {
					if(Thread_Info[a].level == Thread_Info[Menu_Step].submenu) {
						Menu_Step = a;
						menu_changed = 1;
						break;
					}
				}
			}
		} 
/* todo prolong
		else if(s->function == do_fsm_main_cut_back){
			s->function = do_fsm_main_cut_back_prolong; // go to 48 mode to add threads until long_press end
		}
*/
		break;
	}
	case long_press_end_Msk: {
		if(s->function == do_fsm_move)
			s->function = do_long_press_end_callback;
		break;
	}
	}
}

void do_fsm_menu_lps(state_t* s)
{
}

/*
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
*/

void do_fsm_wait_sclick(state_t* s)
{
}

void z_move(uint32_t direction, uint32_t length, bool sync, bool autostart){
	MOTOR_X_Enable();
	MOTOR_Z_Enable(); // time to wakeup motor from sleep is quite high(1.7ms), so enable it as soon as possible

	if(direction == feed_direction_left) {
		feed_direction = feed_direction_left;
		MOTOR_Z_Reverse();
		MOTOR_X_Reverse();
	} else {
		feed_direction = feed_direction_right;
		MOTOR_Z_Forward();
		MOTOR_X_Forward();
	}
	LL_mDelay(2);

	state.sync = sync;
	if(sync){
		state.main_feed_direction = feed_direction;
	}

	z_axis.current_pos = 0;
	z_axis.end_pos = length;
	if(z_axis.end_pos > 0){
		z_axis.end_pos &= 0xFFFFFFFF - step_divider + 1;
//		z_axis.end_pos |= step_divider; // to make sure that we'll not stop between full steps

	} else {
		state.sync = true;
	}

	do_fsm_move_start(&state);
}

//---------------------------------------------------------------------------------------------
void do_fsm_move_start(state_t* s){
	if(s->sync && !s->f_tacho){
		s->function = do_fsm_move_start;// return here from interrupt when TACHO event
		// enable and wait tacho event on spindle encoder
		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
		return;
	}

	if(s->f_tacho || !s->sync) { // if tacho event or we going to start back feed to initial position with async clock
//		LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
		if(s->sync && s->f_tacho) {
			s->function = do_fsm_ramp_up;
			s->async_z = 0;
			s->syncbase = TIM4; 									// sync with spindle

			LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3); 				//trigger by spindle encoder timer TIM4(sync mode)

// disable TACHO events, we dont need'em until next start			
			LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
			enable_encoder_ticks(); 									// enable thread specific interrupt controlled by Q824set
		} else {
			s->function = do_fsm_ramp_up_async;
//			s->async_z = 1;
			s->syncbase = TIM2; 									// sync with internal clock source(virtual spindle, "async" to main spindle)

//			s->set_pulse_function(s);
//			dxdz_callback(s);
//			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
			TIM3->ARR = min_pulse*5;
			LL_TIM_GenerateEvent_UPDATE(TIM3);
//			LL_TIM_EnableCounter(TIM3);
//			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
//			LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin);

//			TIM3->SR = 0;
//			LL_TIM_EnableCounter(TIM3);
//			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
//			LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1); 				//trigger by TIM2(async mode)

			
//			LL_TIM_EnableCounter(TIM2); /* Enable counter */

//			MOTOR_Z_AllowPulse();
//			MOTOR_X_AllowPulse();
//			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);

			TIM2->ARR = 10;
//			LL_TIM_GenerateEvent_UPDATE(TIM2); // start first step on motor
			LL_TIM_EnableCounter(TIM2);


//			TIM2->ARR = 1;
//			LL_TIM_EnableCounter(TIM2);

//		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
//			TIM3->SR = 0;
//			LL_TIM_EnableCounter(TIM3);
//			LL_TIM_GenerateEvent_TRIG(TIM2); // start first step on motor
	//		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
//			s->syncbase->ARR = 1; 					// start stepper motor ramp up procedure immediately after tacho event
			s->async_z = 1;
//			TIM2->CNT = 0;
//			LL_TIM_GenerateEvent_UPDATE(TIM2); /* Force update generation */

		}

//		LL_mDelay(20);
	}	
}

void do_fsm_ramp_up(state_t* s)
{
	z_axis.current_pos++;
	if(z_axis_ramp_up2(s)) {
		s->function = do_fsm_move;
	}
}

void do_fsm_move(state_t* s)
{
//	if(s->spindle_dir)	z_axis.current_pos++;
//	else z_axis.current_pos--;
	if( ++z_axis.current_pos <= ( z_axis.end_pos - z_axis.ramp_step ) ) { // when end_pos is zero, end_pos-ramp_step= 4294967296 - ramp_step, so it will be much more lager then current_pos
		z_axis_move2(s);
	} else {
		if(z_axis_ramp_down2(s)) {
			if(z_axis.end_pos != z_axis.current_pos) {
				z_axis.end_pos = z_axis.current_pos;
			}
			s->function = do_fsm_move_end;
		}
		s->function = do_fsm_ramp_down;
	}
}

void do_long_press_end_callback(state_t* s)          // direct movement: first pass, thread recording: long press release callback
{
	// для 1/2 микрошага нужно что бы общее количество шагов в цикле резьбы было кратно 2,(для 1/4 кратно 4 и тп).
	// это нужно для того что бы в конце шаговый мотор остановился на одном из двухсот устойчивых шагов,
	// не перескакивая на соседние шаги при потере питания.
	if(z_axis.end_pos == 0) //s->sync?
		z_axis.end_pos = ( z_axis.ramp_step + z_axis.current_pos ) | (step_divider - 1);
	s->function = do_fsm_move;
	do_fsm_move(s);
}


void do_fsm_ramp_down(state_t* s)
{
//	if(s->spindle_dir)	
	z_axis.current_pos++;
//	else 
//		z_axis.current_pos--;
	if(z_axis_ramp_down2(s)) {
		if(z_axis.end_pos != z_axis.current_pos) {
			z_axis.end_pos = z_axis.current_pos;
		}
		s->function = do_fsm_move_end;
	}
}

void do_fsm_move_end(state_t* s){
	s->async_z = 0;
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);

	//	MOTOR_Z_BlockPulse();
	if (s->sync) {
		disable_encoder_ticks(); 										//reset interrupt for encoder ticks, only tacho todo async mode not compatible now
		LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
	} else {
		LL_TIM_DisableCounter(TIM2); // pause async timer
//		LL_TIM_DisableIT_UPDATE(TIM2);
	}
	s->syncbase = 0; // reset syncbase to stop calling it from timer interrupt

//	LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin); //debug
	LL_mDelay(2);
  MOTOR_Z_Disable();
  MOTOR_X_Disable();
//	feed_direction = feed_direction == feed_direction_left ? feed_direction_right : feed_direction_left;
	feed_direction = !feed_direction; 					//autochange feed direction
	menu_changed = 1; 													//update menu
	s->function = do_fsm_wait_sclick;

//	z_axis.current_pos = 0;
}


_Bool z_axis_ramp_up2(state_t* s)
{
	const fixedptu  set_with_fract = sync_ramp_profile[z_axis.ramp_step] << 24;
	if(z_axis.Q824set > set_with_fract || z_axis.ramp_step == sync_ramp_profile_len) { 	// reach desired speed or end of ramp map
		s->syncbase->ARR = fixedpt_toint(z_axis.Q824set) - 1; 			// update register ARR
//		s->syncbase->EGR |= TIM_EGR_UG;
		z_axis.fract_part = fixedpt_fracpart(z_axis.Q824set); 								// save fract part for future use on next step
//		z_axis.end_minus_ramp_delta =
		return true;
	} else {
		z_axis.ramp_step++;
		s->syncbase->ARR = fixedpt_toint(set_with_fract) - 1; 			// update register ARR
//		s->syncbase->EGR |= TIM_EGR_UG;
//		z_axis.fract_part = fixedpt_fracpart( set_with_fract ); 						// save fract part for future use on next step
	}
	return false;
}

static inline _Bool z_axis_ramp_down2(state_t* s)
{
	if (z_axis.ramp_step == 0)
		return true;
//	const fixedptu set_with_fract = ramp2[--z_axis.ramp_step] << 24;
//	s->syncbase->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR

	s->syncbase->ARR = sync_ramp_profile[--z_axis.ramp_step];
	//	s->syncbase->EGR |= TIM_EGR_UG;
//	z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	if(z_axis.ramp_step == 0)
		return true;
	return false;
}

void z_axis_move2(state_t* s)
{
	const fixedptu set_with_fract = fixedpt_add(z_axis.Q824set, z_axis.fract_part); // calculate new step delay with fract from previous step
	s->syncbase->ARR = fixedpt_toint(set_with_fract) - 1; // update register ARR
	s->syncbase->CNT = 0;
	s->syncbase->EGR |= TIM_EGR_UG;
	z_axis.fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
}





//------------------------------------ ASYNC block -----------------------------------
//------------------------------------ ASYNC block -----------------------------------
//------------------------------------ ASYNC block -----------------------------------
void do_fsm_ramp_up_async(state_t* s)
{
//	LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin); //debug
//	MOTOR_Z_SetPulse();
	z_axis.current_pos++;
	const uint8_t z_arr = async_ramp_profile[z_axis.ramp_step++];
	const uint16_t rs2 = z_axis.ramp_step << 1;
	if(z_arr < slew_speed_period || rs2 >= z_axis.end_pos  ) { 	// reach desired speed
		if( rs2 < z_axis.end_pos) {
			s->z_period = slew_speed_period;
			s->function = do_fsm_move_async;
		}
		else {
			s->z_period = z_arr;
			s->function = do_fsm_ramp_down_async;
			z_axis.ramp_step--;
		}
	} else {
//		z_axis.ramp_step++;
		s->z_period = z_arr;
	}
}

void do_fsm_move_async(state_t* s)
{
	
//	LL_GPIO_TogglePin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin); //debug
	// todo precalculate delta: z_axis.end_pos - z_axis.ramp_step
//	uint32_t a = z_axis.end_pos;
//	uint32_t b = z_axis.ramp_step;
//	uint32_t cp = ++z_axis.current_pos;
//	uint32_t pre = a - b - 1;
	uint32_t pre = z_axis.end_pos - z_axis.ramp_step - 1;
	if( ++z_axis.current_pos < pre ) { // when end_pos is zero, end_pos-ramp_step= 4294967296 - ramp_step, so it will be much more lager then current_pos
		s->z_period = slew_speed_period;
	} else {
		s->function = do_fsm_ramp_down_async;
	}
}

void do_fsm_ramp_down_async(state_t* s)
{
	z_axis.current_pos++;

	s->z_period = async_ramp_profile[--z_axis.ramp_step];
	if (z_axis.ramp_step == 0) {
//	} else {
// for last step there is no need to wail long, motor can be start to disabled after 145 processor ticks, so with prescaler =145 and more ARR = 1 is enought
//		s->z_period = 2; 
//		LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED); // stop pulse generation on timer
		if(z_axis.end_pos != z_axis.current_pos) {
			z_axis.end_pos = z_axis.current_pos;
		}
		s->function = do_fsm_move_end;
//		do_fsm_move_end(s);
	}
}

void dzdx_init(int dx, int dz, state_t* s) {
	s->dx = abs(dx); 
	s->sx = dx > 0 ? 1 : -1;
  s->dz = abs(dz);
	s->sz = dz > 0 ? 1 : -1; 
  s->e2 = s->err = (s->dx > s->dz ? s->dx : -s->dz) >> 1;
	s->set_pulse_function = dxdz_callback;
}

void G01(int dx, int dz, int feed){
	dzdx_init(dx, dz, &state);

	if(dz<0) {
		feed_direction = feed_direction_left;
		MOTOR_Z_Reverse();
	} else {
		feed_direction = feed_direction_right;
		MOTOR_Z_Forward();
	}

	if(dx<0) {
		MOTOR_X_Reverse();
	} else {
		MOTOR_X_Forward();
	}

	state.sync = false;
	if(state.sync){
		state.main_feed_direction = feed_direction;
	}

	z_axis.current_pos = 0;
	z_axis.end_pos = abs(dz);
	if(z_axis.end_pos > 0){
		z_axis.end_pos &= 0xFFFFFFFF - step_divider + 1;
//		z_axis.end_pos |= step_divider; // to make sure that we'll not stop between full steps

	} else {
		state.sync = true;
	}
	do_fsm_move_start(&state);
}

