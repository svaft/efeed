#include "fsm.h"
#include "buttons.h"
#include "stdlib.h"

#include "gcode.h"
#include "nuts_bolts.h"



#define steps_z 30
#define steps_x 1

bool g_lock = false;

/**
* @brief  precalculated ramp for  30 000Hz frequency timer in 8.24 format.
* 30khz is 500rpm on main spindle with 1800x2 encoder on it, 500/60*3600=30000Hz. 
* ramp map calculated for accel = 460000ppsps with base speed of 400pps.
	* @note	for details see file rampup.xlsx
  */
static const uint32_t ramp_profile[]={ //for 30khz timer precalculated
0x4B000000,
0x4137A6F4,
0x2D745D17,
0x1C50D509,
0x17D84530,
0x13EF9BE2,
0x111EC621,
0x0F21BDA2,
0x0DABFAC7,
0x0C8D85E6,
0x0BA9FACA,
0x0AF000F7,
0x0A5480B9,
0x09D011A2,
0x095D905F,
0x08F94EAD,
0x08A096A5,
0x08515D0A,
0x080A0F2A,
0x07C97188,
0x078E891B,
0x07588B5A,
0x0726D2DE,
0x06F8D714,
0x06CE261C,
0x06A66029,
0x068133FD,
0x065E5C2E,
0x063D9D09,
0x061EC2E2,
0x0601A0C1,
0x05E60F4D,
0x05CBEBED,
0x05B31812,
0x059B78A0,
0x0584F573,
0x056F78F9,
0x055AEFD6,
0x054748A0,
0x0534739F,
0x05226297,
0x0511089E,
0x050059F0,
0x04F04BD4,
0x04E0D47A,
0x04D1EAE5,
0x04C386D1,
0x04B5A0A5,
0x04A83160,
0x049B3286,
0x048E9E1B,
0x04826E91,
};

extern bool feed_direction;
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)
extern const uint8_t Menu_size;

bool demo = true;

void do_fsm_menu(state_t* s){
	uint8_t level = Thread_Info[Menu_Step].level;
#ifdef _SIMU
//	buttons_flag_set = long_press_start_Msk;
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
		if(s->end_pos != 0) {
			// first pass of thread cut was complete, so just use single click
			//	to switch between modes to process all other cuts

//			z_move(feed_direction, s->end_pos, s->main_feed_direction == feed_direction ? true : false, true);
//			if(demo)
//				z_move(feed_direction, s->end_pos, false, true); //test case, always async
//			else
//				z_move(feed_direction, s->end_pos, s->main_feed_direction == feed_direction ? true : false, true);
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

				s->Q824set = Thread_Info[Menu_Step].Q824;
//				const uint64_t upl = (uint64_t)3600 << 48; //calculate some constants for prolong mode
//				s->prolong_addSteps = upl / (fixedptud)s->Q824set; // todo prolong
				// 200*step_divider*z_feed_screw(mm)*len(mm) = desired length in steps, in my case its 200*2*1*x

//				MOTOR_X_Enable();
//				MOTOR_Z_Enable(); // time to wakeup motor from sleep is quite high(1.7ms), so enable it as soon as possible

				//				LL_mDelay(2);
				if(demo){
//					G01(steps_x,steps_z,0);
//					z_move(feed_direction, steps, false, true); //test case, move async 10mm
//					z_move(feed_direction, 31, false, true); //test case, move async 10mm
				}
//				else
//					z_move(feed_direction, 0, true, true);

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

void do_long_press_end_callback(state_t* s){          // direct movement: first pass, thread recording: long press release callback
	// для 1/2 микрошага нужно что бы общее количество шагов в цикле резьбы было кратно 2,(для 1/4 кратно 4 и тп).
	// это нужно для того что бы в конце шаговый мотор остановился на одном из двухсот устойчивых шагов,
	// не перескакивая на соседние шаги при потере питания.
	if(s->end_pos == 0) //s->sync?
		s->end_pos = ( s->ramp_step + s->current_pos ) | (step_divider - 1);
	s->function = do_fsm_move;
	do_fsm_move(s);
}



void do_fsm_menu_lps(state_t* s){
}

void do_fsm_wait_sclick(state_t* s){
}

__STATIC_INLINE 
void set_ARR(state_t* s, fixedptu value){
//TODO
/*
	добавить здесь корректировку времени шага в зависимости от того, шагнул один мотор или оба сразу.
	При шаге одним мотором по одной оси подача(feed) у нас линейная и устанавливается в соответствии с Q824
	при шаге двумя моторами пройденное расстояние будет по диагонали, соответственно будет больше в зависимости
	от шага винтов на осях и настроек шаговиков.
	пройденное расстояние вычисляется как корень кв((шаг винта X в квадрате + шаг винта Z в квадрате)
	= sqrt(screwX*screwX + screwZ*screwZ)
	for 1mm pitch leadscrew on Z and 1,27mm pitch on X we have
	sqrt(1+1,613)=1,61644672 coefficient for delay correction for Z and X axis step 
	and 1,002198971 coefficient if we step by X only(can be ignored in this case).
	for Z axis only coefficient is 1.
	1,61644672 = 27119476 in Q824
	so when we have both axis step we need to multiply passed value to this function by this 27119476 value.
	both direction step flag can be catched in TIM3 interrupt.
	
	warning: Also as we have 30khz precalculated ramp profile highest delay now is 75ARR,
	so if we multiply it by 1,61644672 we'll be in 8bit limit(<255) so it's ok. for greated delays(157 and up) different package format 
	should be used(923, 1022...)
	*/	
	//	s->z_period = value;
	
	uint32_t arr;
	
	if(TIM3->CCER > 256){ //both steppers pulse
		arr = fixedpt_toint(value) - 1;	
// todo check it later		arr = fixedpt_toint(fixedpt_mul(value,27119476)) - 1;
	} else {
		arr = fixedpt_toint(value) - 1;	
	}
	s->syncbase->ARR = arr;
//	s->syncbase->EGR |= TIM_EGR_UG;
}

//---------------------------------------------------------------------------------------------
void do_fsm_move_start(state_t* s){
	bool f_tacho = t4sr[TIM_SR_CC3IF_Pos];
	if(s->sync && !f_tacho){
		s->syncbase = TIM4;
		LL_TIM_EnableIT_CC3(s->syncbase);
		LL_TIM_EnableUpdateEvent(s->syncbase);
		s->function = do_fsm_move_start;// return here from interrupt when TACHO event
		// enable and wait tacho event on spindle encoder
		LL_TIM_CC_EnableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
		//		LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3); 				//trigger by spindle encoder timer TIM4(sync mode)
//		LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
		return;
	}

	if(f_tacho || !s->sync) { // if tacho event or we going to start back feed to initial position with async clock
		s->function = do_fsm_ramp_up;
		if(s->sync && f_tacho) {

//			LL_TIM_DisableCounter(s->syncbase);
			LL_TIM_DisableIT_CC3(s->syncbase);
			LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);	// disable TACHO events on channel 3
//			s->syncbase = TIM4; 									// sync with spindle

			LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3); 				//trigger by spindle encoder timer TIM4(sync mode)
			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
			TIM3->ARR = min_pulse;
			LL_TIM_GenerateEvent_UPDATE(TIM3);
//			debug();
			// disable TACHO events, we dont need'em until next start:
//LL_TIM_SetUpdateSource
//			s->syncbase->CNT = 0;
			set_ARR(s,(uint32_t)255<<24);

			// enable update inerrupt:
			LL_TIM_EnableIT_UPDATE(s->syncbase); //enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
//			LL_TIM_EnableCounter(s->syncbase);
//			*/
//			LL_TIM_DisableUpdateEvent(TIM4);
			LL_TIM_GenerateEvent_UPDATE(s->syncbase); /* Force update generation */
		} else {
			s->syncbase = TIM2; 									// sync with internal clock source(virtual spindle, "async" to main spindle)

			LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1); 				//trigger by spindle encoder timer TIM4(sync mode)
			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
			TIM3->ARR = min_pulse;
			LL_TIM_GenerateEvent_UPDATE(TIM3);
			s->syncbase->CNT = 0;
			set_ARR(s,10<<24);
			LL_TIM_EnableCounter(s->syncbase);
		}
		LL_TIM_EnableUpdateEvent(s->syncbase);
	}	
}


void do_fsm_move_start2(state_t* s){
	bool f_tacho = t4sr[TIM_SR_CC3IF_Pos];
	if(s->sync && !f_tacho){
		s->syncbase = TIM4;
		LL_TIM_EnableIT_CC3(s->syncbase);
		LL_TIM_EnableUpdateEvent(s->syncbase);
		s->function = do_fsm_move_start;// return here from interrupt when TACHO event
		// enable and wait tacho event on spindle encoder
		LL_TIM_CC_EnableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
		//		LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3); 				//trigger by spindle encoder timer TIM4(sync mode)
//		LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
		return;
	}

	if(f_tacho || !s->sync) { // if tacho event or we going to start back feed to initial position with async clock
		s->function = do_fsm_move2;
		if(s->sync && f_tacho) {
			LL_TIM_DisableIT_CC3(s->syncbase);
			LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);	// disable TACHO events on channel 3
			LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3); 				//trigger by spindle encoder timer TIM4(sync mode)
			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
			TIM3->ARR = min_pulse;
			LL_TIM_GenerateEvent_UPDATE(TIM3);
			set_ARR(s,(uint32_t)255<<24);
			// enable update inerrupt:
			LL_TIM_EnableIT_UPDATE(s->syncbase); //enable_encoder_ticks(); // enable thread specific interrupt controlled by Q824set
			LL_TIM_GenerateEvent_UPDATE(s->syncbase); /* Force update generation */
		} else {
			
			load_next_task(); // load first task from queue
			s->Q824set = s->current_task.F; // load feed value
//			s->steps_to_end = s->current_task.dz > s->current_task.dx ? s->current_task.dz : s->current_task.dx;
//			dzdx_init(int dx, int dz, s);
			s->err = (s->current_task.dx > s->current_task.dz ? s->current_task.dx : -s->current_task.dz) >> 1;
//			s->current_task.
			
			// connect async timer:
			s->syncbase = TIM2; 									// sync with internal clock source(virtual spindle, "async" to main spindle)
			LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1); 				//trigger by spindle encoder timer TIM4(sync mode)
			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
			TIM3->ARR = min_pulse;
			LL_TIM_GenerateEvent_UPDATE(TIM3);
			s->syncbase->CNT = 0;
			set_ARR(s,10<<24);
			LL_TIM_EnableCounter(s->syncbase);
		}
		LL_TIM_EnableUpdateEvent(s->syncbase);
	}	
}


void do_fsm_ramp_up(state_t* s){
//	debug();
	s->current_pos++;
	fixedptu  set_with_fract = fixedpt_add(ramp_profile[s->ramp_step], s->fract_part); // calculate new step delay with fract from previous step
	uint16_t rs2 = s->ramp_step << 1;
	if(s->Q824set > set_with_fract || rs2 >= s->end_pos) { 	// reach desired speed (or end of ramp map? || s->ramp_step == sync_ramp_profile_len)
		if(rs2 >= s->end_pos){ 
			s->ramp_step -=2;
			set_with_fract = ramp_profile[s->ramp_step];
			set_ARR(s,set_with_fract);
			s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
			s->function = do_fsm_ramp_down;
		} else {
			set_ARR(s,set_with_fract);
			s->fract_part = fixedpt_fracpart(s->Q824set); 								// save fract part for future use on next step
			s->function = do_fsm_move;
		}
	} else {
		s->ramp_step++;
		set_ARR(s,set_with_fract);
		s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	}
}



void load_next_task(){
	cb_pop_front(&task_cb, &state.current_task);
}
void do_fsm_ramp_up2(state_t* s){
//	debug();
	s->steps_to_end--;
	fixedptu  set_with_fract = fixedpt_add(ramp_profile[s->ramp_step], s->fract_part); // calculate new step delay with fract from previous step
	uint16_t rs2 = s->ramp_step << 1;
	if(s->Q824set > set_with_fract || rs2 >= s->steps_to_end) { 	// reach desired speed (or end of ramp map? || s->ramp_step == sync_ramp_profile_len)
		if(rs2 >= s->steps_to_end){ 
			s->ramp_step -=2;
			set_with_fract = ramp_profile[s->ramp_step];
			set_ARR(s,set_with_fract);
			s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
			s->function = do_fsm_ramp_down2;
		} else {
			set_ARR(s,set_with_fract);
			s->fract_part = fixedpt_fracpart(s->Q824set); 								// save fract part for future use on next step
			s->function = do_fsm_move2;
		}
	} else {
		s->ramp_step++;
		set_ARR(s,set_with_fract);
		s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	}
}

void do_fsm_move2(state_t* s){
	fixedptu set_with_fract = fixedpt_add(s->Q824set, s->fract_part); // calculate new step delay with fract from previous step
	set_ARR(s,set_with_fract);
	s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	s->current_task.steps_to_end--;
//  if( --s->steps_to_end == (s->ramp_step - 1) ) { // when end_pos is zero, end_pos-ramp_step= 4294967296 - ramp_step, so it will be much more lager then current_pos
//		s->function = do_fsm_ramp_down2;
//	}
}
void do_fsm_ramp_down2(state_t* s){
//	debug();
	s->steps_to_end--;
	fixedptu set_with_fract;
	if(s->ramp_step == 0){
		set_with_fract = fixedpt_add(s->Q824set, s->fract_part); // calculate new step delay with fract from previous step
	} else {
		set_with_fract = fixedpt_add(ramp_profile[--s->ramp_step], s->fract_part); // calculate new step delay with fract from previous step
		s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	}
	set_ARR(s,set_with_fract);
	if (s->ramp_step == 0) {
//		if(s->end_pos != s->current_pos) {
//			s->end_pos = s->current_pos;
//		}
		s->function = do_fsm_move_end2;
	}	
}
void do_fsm_move_end2(state_t* s){
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);

	if (s->sync) {
		disable_encoder_ticks(); 										//reset interrupt for encoder ticks, only tacho todo async mode not compatible now
		LL_TIM_CC_DisableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
	} else {
		LL_TIM_DisableCounter(s->syncbase); // pause async timer
	}
	LL_TIM_DisableUpdateEvent(s->syncbase);
	s->z_period = 0;
	LL_mDelay(2);
  MOTOR_Z_Disable();
  MOTOR_X_Disable();
	menu_changed = 1; 													//update menu
	s->function = do_fsm_wait_sclick;
	g_lock = false;
}



void do_fsm_move(state_t* s){
	fixedptu set_with_fract = fixedpt_add(s->Q824set, s->fract_part); // calculate new step delay with fract from previous step
	set_ARR(s,set_with_fract);
	s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
  if( ++s->current_pos == (s->end_pos - s->ramp_step - 1) ) { // when end_pos is zero, end_pos-ramp_step= 4294967296 - ramp_step, so it will be much more lager then current_pos
		s->function = do_fsm_ramp_down;
	}
}

/**
  * @brief  ramp down stepper
  * @param  state structure
  */
void do_fsm_ramp_down(state_t* s){
//	debug();
	s->current_pos++;
	fixedptu set_with_fract;
	if(s->ramp_step == 0){
		set_with_fract = fixedpt_add(s->Q824set, s->fract_part); // calculate new step delay with fract from previous step
	} else {
		set_with_fract = fixedpt_add(ramp_profile[--s->ramp_step], s->fract_part); // calculate new step delay with fract from previous step
		s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	}
	set_ARR(s,set_with_fract);
	if (s->ramp_step == 0) {
		if(s->end_pos != s->current_pos) {
			s->end_pos = s->current_pos;
		}
		s->function = do_fsm_move_end;
	}	
}

void do_fsm_move_end(state_t* s){
//	debug();
//	s->async_z = 0;
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);

	if (s->sync) {
		disable_encoder_ticks(); 										//reset interrupt for encoder ticks, only tacho todo async mode not compatible now
		LL_TIM_CC_DisableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
	} else {
		LL_TIM_DisableCounter(s->syncbase); // pause async timer
	}
//	s->syncbase = 0; // reset syncbase to stop calling it from timer interrupt
	LL_TIM_DisableUpdateEvent(s->syncbase);

	//	TIM3->ARR = 0;
//	LL_TIM_GenerateEvent_UPDATE(TIM3);
//	LL_TIM_DisableCounter(TIM3); // pause async timer

	//	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
//	TIM3->SR = 0;
	
	s->z_period = 0;
//	debug();
	LL_mDelay(2);
  MOTOR_Z_Disable();
  MOTOR_X_Disable();
//	feed_direction = !feed_direction; 					//autochange feed direction
	menu_changed = 1; 													//update menu
	s->function = do_fsm_wait_sclick;
	g_lock = false;
}



// init bresenham algorithm variables for generate stepper motors pulses
__STATIC_INLINE void dzdx_init(int dx, int dz, state_t* s) {
	s->current_task.dx = dx; //dx>0?dx:-dx; //	s->dx = abs(dx); 
	s->current_task.dz = dz; //dz>0?dz:-dz;//  s->dz = abs(dz);
  s->err = (s->current_task.dx > s->current_task.dz ? s->current_task.dx : -s->current_task.dz) >> 1;
}

/**
* @brief  G00: Coordinated Straight Motion Rapid Rate
  * @retval void.
  */
void G00(int dx, int dz){
	g_lock = true;
//	G01(dx,dz);
// move to position with max speed	
}


/**
* @brief  G01: Coordinated Straight Motion Feed Rate
  * @retval void.
  */
void G01(int dx, int dz, int feed){
	g_lock = true;
	// linear interpolated move to position with defined feed
	dzdx_init(dx, dz, &state);
	state.sync = false;

/*
TODO
нужно переделать/переосмыслить механизм определения и использования
позиций шаговика, а то какая-то каша получается
*/
	
	if(state.steps_to_end == 0) {
		state.steps_to_end += dz>dx?dz:dx;
	//	if(state.end_pos > 0){
	//		state.end_pos &= 0xFFFFFFFF - step_divider + 1;// to make sure that we'll not stop between full steps
	//	}
		do_fsm_move_start(&state);
	} else{
		state.steps_to_end += dz>dx?dz:dx;
	}
}


/**
* @brief  G33: Spindle Synchronized Motion
K - distance per revolution
X
Z

Technical Info(from LinuxCNC page)
At the beginning of each G33 pass, LinuxCNC uses the spindle speed and the machine acceleration 
limits to calculate how long it will take Z to accelerate after the index pulse, and determines 
how many degrees the spindle will rotate during that time. It then adds that angle to the index 
position and computes the Z position using the corrected spindle angle. 
That means that Z will reach the correct position just as it finishes accelerating to the proper speed, 
and can immediately begin cutting a good thread.
* @retval void.
  */
void G33(int dx, int dz, int K){
	g_lock = true;
	// move to position with spindle sync. Used for threading.
	// command is the same(?) as the G95 with followed G01 and sync start by tacho pulse from spindle
	dzdx_init(dx, dz, &state);
/*
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
*/
//	state.sync = false; 
	state.sync = true;
//	if(state.sync){
	state.main_feed_direction = feed_direction;
//	}

	state.current_pos = 0;
	state.end_pos = dz>=0?dz:-dz; //abs(dz);
	if(state.end_pos > 0){
		state.end_pos &= 0xFFFFFFFF - step_divider + 1;
//		s->end_pos |= step_divider; // to make sure that we'll not stop between full steps

//	} else {
//		state.sync = true;
	}
	do_fsm_move_start(&state);
}





/**
* @brief  G76: Threading cycle
  * @retval void.
P- - The thread pitch in distance per revolution.
Z- - The final position of threads. At the end of the cycle the tool will be at this Z position.
  */
void G76(int p, int z){
	// move to position with spindle sync. Used for threading.
	// command is the same(?) as the G95 with followed G01 and sync! start by tacho pulse from spindle
}





fixedptu f;
G_pipeline current_code;
G_pipeline prev_code;


void arc_dx_callback(){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	t3ccer[TIM_CCER_CC1E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
	int dz = SquareRoot(s->current_task.rr - s->current_task.dx * s->current_task.dx);
	if(dz != s->current_task.dz){
		t3ccer[TIM_CCER_CC3E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
		s->current_task.dz = dz;
	}
	s->current_task.dx += s->current_task.inc_dec;
}

void arc_dz_callback(){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	t3ccer[TIM_CCER_CC3E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
	int dx = SquareRoot(s->current_task.rr - s->current_task.dz * s->current_task.dz);
	if(dx != s->current_task.dx){
	t3ccer[TIM_CCER_CC1E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
		s->current_task.dx = dx;
	}
	s->current_task.dz += s->current_task.inc_dec;
}

void dxdz_callback(){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int e2 = s->err;
	if (e2 > -s->current_task.dx)	{ // step X axis
		s->err -= s->current_task.dz; 
		t3ccer[TIM_CCER_CC1E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
	}
	if (e2 < s->current_task.dz)	{ // step Z axis
		s->err += s->current_task.dx;
		t3ccer[TIM_CCER_CC3E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
	}
}


/*
0x4B000000,
0x4137A6F4,
0x2D745D17,
0x1C50D509,
0x17D84530,
0x13EF9BE2,
0x111EC621,
0x0F21BDA2,
*/
uint8_t get_ramp_steps(fixedptu from_speed, fixedptu to_speed){
	uint32_t ramp_delay, from, to;
	if(from_speed > to_speed){//ramp up
		ramp_delay = ramp_profile[0];
		while(ramp_delay > from_speed){
			from++;
			ramp_delay = ramp_profile[from];
		}
		for(int a = from; a <sizeof(ramp_profile);a++){
			ramp_delay = ramp_profile[a];
			if(ramp_delay <= to_speed)
				return a - from;
		}
	} else { //ramp down
	}
}











/* unused
void do_fsm_main_cut_back_prolong(state_t* s)   // reverse movement: main part with prolong activated. todo split it with 46 mode?
{
	MOTOR_Z_SetPulse();
	--s->current_pos;
	if(s->current_pos == s->ramp_step) { // we reach end of main path and have long_pressed key, so just add additional thread full turn to shift initial start point
		s->prolong_fract += s->prolong_addSteps; // fract part from prev step
		uint32_t prolong_fixpart = s->prolong_fract >> 24;
		s->current_pos += prolong_fixpart; // add fixed part
		s->end_pos += prolong_fixpart;
		s->prolong_fract &= FIXEDPT_FMASK; // leave fract part to accumulate with next dividing cycle
		// when long_press end, get back to 46 mode to proceed
	}
}
*/
