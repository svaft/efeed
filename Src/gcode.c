#include "gcode.h"


fixedpt command;
G_pipeline_t init_gp={0,0,0,0,0};

//G_task_t gt_precalc[task_precalc_size];
G_task_t gt[task_size];
//G_pipeline_t gp[gp_size];
substep_t substep_delay[substep_size]; // todo check how its work with small substep buffer size 
substep_job_t substep_job[substep_job_size];


//профиль ускорения при 300000pps/ps расчитан в файле rapmup.xls
uint8_t ramp_pos = 0;
uint32_t steps2end_equator = 0;

substep_t* cb_push_back_empty_ref(void){
	cb_push_back_empty(&substep_cb);
	return substep_cb.top;
}

uint32_t current_step = 0;
int break1 = 0;


void jog_pulse(int delay){
	state_hw.jog_pulse = true;
	state_hw.substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));
	LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_OUTPUT);

	TIM1->CCR1	= delay + 1;
	TIM1->ARR 	= delay + 46;//min_pulse + 1;
	TIM1->SR = 0;
	if(LL_TIM_IsEnabledCounter(TIM1))
		Error_Handler2(10); // some error?
	LL_TIM_EnableCounter(TIM1);
}
/**
* @brief  Extract new task from queue and start to process it
* @retval void.
  */
void load_next_task(state_t* s){
	if(s->task_lock == false && task_cb.count > 0) {
		G_task_t *next_task = cb_get_front_ref(&task_cb);
		if(next_task && next_task->unlocked == true) {
			s->last_loaded_task_ref  = next_task;
			current_step++;
//			if(current_step == 46)
//				break1 = 1;
//		if(next_task && next_task->unlocked == true) {
//	debug();
			s->task_lock = true;
//			cb_pop_front(&task_cb, s->current_task_ref);
			s->current_task_ref = next_task;//cb_get_front_ref(&task_cb);
			cb_pop_front_ref(&task_cb);
			if(s->current_task_ref->init_callback_ref){
				s->current_task_ref->init_callback_ref(s); // task specific init
//				if(s->task_lock == false)
//					cb_pop_front_ref(&task_cb);
			}
//			ramp_pos = 0; //todo костыль. надо смотреть по вектору движения, скорости и относительно этого корректировать ускорение,
			//например при смене направления нужно остановиться и с нуля ускориться, при движении в том же направлении можно продолжить на той же скорости			
//			s->Q824set = s->current_task_ref->F; // load feed value
			if(s->current_task_ref->stepper && LL_TIM_IsEnabledCounter(s->syncbase) == false) {
				do_fsm_move_start2(s);
			}
		} else{ // no next task
		}
	} 
//	else if (task_cb.count == 0 && s->task_lock == false){
//		s->current_task_ref = 0;
//	}
}


void G90init_callback_precalculate(state_t* s){
	s->G90G91 = G90mode;
	s->current_task_ref->unlocked = true;
}
void G90(state_t* s){
	s->G90G91 = G90mode;
	s->task_lock = false; // all processing is done here so unlock task to next
}
void G91init_callback_precalculate(state_t* s){
	s->G90G91 = G91mode;
	s->current_task_ref->unlocked = true;
}
void G91(state_t* s){
	s->G90G91 = G91mode;
	s->task_lock = false; // all processing is done here so unlock task to next
}



void G94init_callback_precalculate(state_t* s){
	s->G94G95 = G94code;
	s->current_task_ref->unlocked = true;
}

int sta = 0;
void switch_to_async(state_t* s){
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR0);

// unattach sync master TIM4 from slave	
  LL_TIM_DisableMasterSlaveMode(TIM4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
	LL_TIM_DisableCounter(TIM4); // pause sync timer
	sta++;
	LL_TIM_DisableCounter(TIM2); // pause async timer
	// connect sync timer:
	s->syncbase = TIM2; 									// sync with internal clock source(virtual spindle, "async" to main spindle)

  LL_TIM_EnableMasterSlaveMode(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);

// calibrate timer delay
//	LL_TIM_DisableUpdateEvent(TIM2);
	// connect async timer:
	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1); 				//trigger by asnyc timer TIM2(async mode)
	LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
}

void G94(state_t* s){
	if(s->G94G95 == G94code){ //если мы уже в асинхронном режиме переконфигурацию таймеров не проводим
		s->task_lock = false;
		return;
	}
	
	s->G94G95 = G94code;
	if(s->G94G00tmp == true){
		s->G94G00tmp = false;
	} else{
		switch_to_async(s);
	}
	s->task_lock = false; // all processing is done here so unlock task to next
}

bool msm;
void switch_to_sync(state_t* s){
	// отключаем TIM3 как ведомый таймер от мастера
	LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR0);
//  LL_TIM_DisableMasterSlaveMode(TIM3);
//	LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH3); // отключаем канал(ы?) для исключения генерации лишнего пульса при LL_TIM_GenerateEvent_UPDATE
	// отключаем мастер-таймер TIM2 от генерации сигналов ведомому 
  LL_TIM_DisableMasterSlaveMode(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);

//	LL_mDelay(2);
	// reconfigure async timer:
	LL_TIM_DisableCounter(TIM2); // pause async timer

//	TIM2->PSC = 0; // reset prescaler and set tim2 to max speed to use it as delay measure
//	TIM2->ARR = 0xFFFF;
	// to set prescaler register we need to generate update event, so disable IT first to prevent call of IT routine:
	LL_TIM_DisableIT_UPDATE(TIM2); 
	// and then generate UPDATE event:
	LL_TIM_GenerateEvent_UPDATE(TIM2); 

//	LL_GPIO_SetOutputPin(MOTOR_X_ENABLE_GPIO_Port,MOTOR_X_ENABLE_Pin);
	LL_TIM_DisableCounter(TIM4); // pause sync timer
//	LL_TIM_DisableUpdateEvent(TIM4);
  LL_TIM_EnableMasterSlaveMode(TIM4);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_UPDATE);


	// connect sync timer:
	s->syncbase = TIM4; 									// sync with main spindle encoder

	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3); 				//trigger by snyc timer TIM4(spindle sync mode)
	LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);

//	MOTOR_X_Enable();
//	LL_GPIO_ResetOutputPin(MOTOR_X_ENABLE_GPIO_Port,MOTOR_X_ENABLE_Pin);
}
void G95(state_t* s){
//	MOTOR_X_Enable();
	if(s->G94G95 == G95code){ //если мы уже в синхронном режиме переконфигурацию таймеров не проводим
		s->task_lock = false;
		return;
	}
	s->G94G95 = G95code;
	switch_to_sync(s);
	s->task_lock = false; // all processing is done here so unlock task to next
}

void G95init_callback_precalculate(state_t* s){
	s->G94G95 = G95code;
	s->current_task_ref->unlocked = true;
}



#define leddbg 100
int ms =0;
void do_fsm_move_start2(state_t* s){
//	load_next_task(s); // load first task from queue
//	LL_TIM_DisableARRPreload(s->syncbase); // prepare timer start after EnableCounter plus one timer tick to owerflow
//	s->syncbase->ARR = 1;
//	s->syncbase->CNT = 1; // set ARR=CNT to start pulse generation on next count increment after EnableCounter.
	// disclamer: why not to generate	update event by setting in EGR UG bit? with hardware logic analyzer all works fine,
	// but in simulator this first pulse not generater propertly by UG
//	LL_GPIO_SetOutputPin(MOTOR_X_ENABLE_GPIO_Port,MOTOR_X_ENABLE_Pin);
	ms++;
	ramp_pos = 0;
	steps2end_equator = s->current_task_ref->steps_to_end>>1;
	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);




//	#define _USEENCODER
	#ifdef _USEENCODER // wait tacho only when HW encoder is used, in software emu mode this is not necessary, we start immediately
// reload value in stored in ARR from preload to shadow register without update event 	
	if(s->syncbase == TIM4 && s->sync){ // if sync wait for tacho event and continue. While waiting disable stepper motors to correct position manually if needed
		LL_TIM_DisableIT_UPDATE(s->syncbase);
		LL_TIM_DisableUpdateEvent(s->syncbase);
		s->syncbase->SR = 0;
		LL_TIM_CC_EnableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);
		LL_TIM_EnableCounter(s->syncbase);
		while(!LL_TIM_IsActiveFlag_CC3(s->syncbase)); //wait tacho to sync move
		LL_TIM_CC_DisableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);

//wait spindle to rotate specific angle if needed:
		if(s->delay > 0) {
			s->syncbase->CNT = 0;
			LL_TIM_DisableARRPreload(s->syncbase);
			s->syncbase->ARR = s->delay;
			LL_TIM_EnableARRPreload(s->syncbase);
			s->syncbase->SR = 0;
			LL_TIM_EnableUpdateEvent(s->syncbase);
			while(!LL_TIM_IsActiveFlag_UPDATE(s->syncbase));
			LL_TIM_DisableCounter(s->syncbase);
			
		}
	}

	LL_TIM_DisableARRPreload(s->syncbase);
	if(s->syncbase->ARR < rampup[0] ){
		s->syncbase->ARR = rampup[ramp_pos++];
	} else{
		s->syncbase->ARR = s->syncbase->ARR;
	}
	LL_TIM_EnableARRPreload(s->syncbase);

	LL_TIM_EnableUpdateEvent(s->syncbase);
	s->syncbase->CNT = 0;
	s->syncbase->SR = 0;
	LL_TIM_EnableIT_UPDATE(s->syncbase);
	LL_TIM_EnableCounter(s->syncbase);


#else 
// initial load ARR
	LL_TIM_DisableARRPreload(s->syncbase);
	if(s->syncbase->ARR < rampup[0] ){
		s->syncbase->ARR = rampup[ramp_pos++];
	} else{
		s->syncbase->ARR = s->syncbase->ARR;
	}
	LL_TIM_EnableARRPreload(s->syncbase);

	LL_TIM_EnableUpdateEvent(s->syncbase);
	s->syncbase->CNT = 0;
	s->syncbase->SR = 0;
	LL_TIM_EnableIT_UPDATE(s->syncbase);
	LL_TIM_EnableCounter(s->syncbase);


	#endif

//	LL_GPIO_ResetOutputPin(MOTOR_X_ENABLE_GPIO_Port,MOTOR_X_ENABLE_Pin);
}
int32_t move_cnt = 0;


int break1;
/**
* @brief  do_fsm_move2
* @retval void.
  */
void do_fsm_move2(state_t* s){
	//todo нужен рефакторинг метода
	s->rised = true;
	substep_t *sb = substep_cb.tail; //get ref to current substep

	if(s->substep_axis == SUBSTEP_AXIS_X){
		s->current_task_ref->z_direction == zdir_forward ? s->global_Z_pos++ : s->global_Z_pos--;
		if(sb->skip ==0) s->current_task_ref->x_direction == xdir_backward ? s->global_X_pos++ : s->global_X_pos--;
	}	else {
		s->current_task_ref->x_direction == xdir_backward ? s->global_X_pos++ : s->global_X_pos--;
		if(sb->skip ==0) s->current_task_ref->z_direction == zdir_forward ? s->global_Z_pos++ : s->global_Z_pos--;
	}
	if(sb->skip == 0){ // if substep have no skip steps in it, calculate next delay and start substep timer to generate substep pulse
		int32_t delay = sb->delay*s->prescaler * (s->syncbase->ARR+1) >> subdelay_precision; // todo delay recalculate move to tim2 or tim4 wherer arr is changing?
		delay >>=4;
		if(delay>65500)
			Error_Handler(); // owerflow error, enlarge tim1 prescaler, timer1 is too fast

		//jog_pulse(delay);	
		TIM1->CCR1	= delay + 1;
		TIM1->ARR 	= delay + 46;//min_pulse + 1;
		TIM1->SR = 0;
//		if(LL_TIM_IsEnabledCounter(TIM1))
	//		Error_Handler(); // some error?
		LL_TIM_EnableCounter(TIM1);
		cb_pop_front_ref(&substep_cb); 		// timer to substep pulse started, move substep ref to next value in circular buffer to get in on next iteration
	} else { 														// substep contain packed value of skipped steps, continue to skip steps until skip = 0
		sb->skip--;
		if(sb->skip == 0)
			cb_pop_front_ref(&substep_cb); 	// substep reach zero, move substep ref to next value in circular buffer to get in on next iteration
	}

	fixedptu set_with_fract = fixedpt_add(s->Q824set, s->fract_part); // calculate new step delay with fract from previous step
	s->syncbase->ARR = fixedpt_toint(set_with_fract) - 1;							// load step delay to ARR register
	if(s->current_task_ref->skiprampdown!=true && ramp_pos >= s->current_task_ref->steps_to_end){
		s->syncbase->ARR = rampup[--ramp_pos];
	} else {
		if(s->syncbase->ARR < rampup[ramp_pos]){
			s->syncbase->ARR = rampup[ramp_pos++];
		} else{
			s->fract_part = fixedpt_fracpart( set_with_fract ); 					// save fract part for future use on next step
		}
	}
//	s->current_task_ref->steps_to_end--; // migrated to callback
}

void do_fsm_move33(state_t* s){
	s->current_task_ref->z_direction == zdir_forward ? s->global_Z_pos++ : s->global_Z_pos--;
	move_cnt++;
//	if(move_cnt>460 && move_cnt<470) 
//		break1 = 1;
	substep_t *sb = substep_cb.tail; //get ref to current substep
	if(sb->skip == 0){ // if substep have no skip steps in it, calculate next delay and start substep timer to generate substep pulse
		int32_t delay = sb->delay*s->prescaler * (s->syncbase->ARR+1) >> subdelay_precision; // todo delay recalculate move to tim2 or tim4 wherer arr is changing?
		delay >>=4;
		if(delay>65500)
			Error_Handler(); // owerflow error, enlarge tim1 prescaler, timer1 is too fast
		
		//jog_pulse(delay);		
		TIM1->CCR1	= delay + 1;
		TIM1->ARR 	= delay + 46;//min_pulse + 1;
		TIM1->SR = 0;
//		if(LL_TIM_IsEnabledCounter(TIM1))
		//	Error_Handler(); // some error?
		LL_TIM_EnableCounter(TIM1);
		cb_pop_front_ref(&substep_cb); 		// timer to substep pulse started, move substep ref to next value in circular buffer to get in on next iteration
	} else { 														// substep contain packed value of skipped steps, continue to skip steps until skip = 0
		sb->skip--;
		if(sb->skip == 0)
			cb_pop_front_ref(&substep_cb); 	// substep reach zero, move substep ref to next value in circular buffer to get in on next iteration
	}

	fixedptu set_with_fract = fixedpt_add(s->Q824set, s->fract_part); // calculate new step delay with fract from previous step
	s->syncbase->ARR = fixedpt_toint(set_with_fract) - 1;							// load step delay to ARR register

	s->fract_part = fixedpt_fracpart( set_with_fract ); 					// save fract part for future use on next step

	if(ramp_pos >= s->current_task_ref->steps_to_end){
	//	s->syncbase->ARR = rampup[--ramp_pos];
	} else {
//		if(s->syncbase->ARR < rampup[ramp_pos]){
//			s->syncbase->ARR = rampup[ramp_pos++];
//		} else{
//			s->fract_part = fixedpt_fracpart( set_with_fract ); 					// save fract part for future use on next step
//		}
	}
		
//	s->current_task_ref->steps_to_end--; // migrated to callback
}




/**
* @brief  do_fsm_move_end2
* @retval void.
  */
void do_fsm_move_end2(state_t* s){
//	debug();
//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
	ramp_pos = 0; // reset ramp map 
	cb_reset(&substep_cb);
	init_gp.Z = fixedpt_fromint2210(s->global_Z_pos);
	init_gp.X = fixedpt_fromint2210(s->global_X_pos);

	s->current_task_ref = 0;
//	substep_cb.count = substep_cb.count2 = 0;
//	substep_cb.top = substep_cb.head = substep_cb.tail2 = substep_cb.tail;

	if (s->sync) {
		disable_encoder_ticks(); 										//reset interrupt for encoder ticks, only tacho todo async mode not compatible now
		LL_TIM_CC_DisableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
		LL_TIM_DisableCounter(s->syncbase); // pause async timer
	} else {
		LL_TIM_DisableCounter(s->syncbase); // pause async timer
	}
	LL_TIM_DisableIT_UPDATE(s->syncbase);

	LL_TIM_DisableUpdateEvent(s->syncbase);
	EOM();
//	sendDefaultResponseDMA('E',&state_hw.global_X_pos);
//	sendResponse((uint32_t)"end\r\n",5);
}


//__STATIC_INLINE 
//G_task * add_empty_task(){
//	cb_push_back_empty(&task_cb);
//	return task_cb.top;
//}

//__STATIC_INLINE 
G_task_t* get_last_task( void ){
	return (G_task_t *)task_cb.top;
}



void command_parser(char *line){
	uint8_t char_counter = 0;  
	char letter;
	G_task_t *g_task;
//	char *end;
  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    letter = line[char_counter++];
		switch(letter){
			case 'G':
				command = str_f_to_steps2210(line, &char_counter);
				switch(command){
					//todo now its only one G command per line supported
//#define G4_2210	4*steps_per_unit_Z_2210*1024
					case 7*steps_per_unit_Z_2210: //G7 Lathe Diameter Mode
						break;
					case 8*steps_per_unit_Z_2210: //G8 Lathe Radius Mode
						break;
					case 20*steps_per_unit_Z_2210: //G20 - to use inches for length units
						break;
					case 21*steps_per_unit_Z_2210: //G21 - to use millimeters for length units
						break;
					case 4*steps_per_unit_Z_2210: //G4 dwell, pause P seconds
						G04parse(line+char_counter);
						break;
					case 90*steps_per_unit_Z_2210: //G90  absolute distance mode
						state_precalc.G90G91 = G90mode;
						g_task = add_empty_task();
						g_task->init_callback_ref = G90;
						g_task->precalculate_init_callback_ref = G90init_callback_precalculate;
						break;
					case 91*steps_per_unit_Z_2210: //G91  incremental distance mode
						state_precalc.G90G91 = G91mode;
						g_task = add_empty_task();
						g_task->init_callback_ref = G91;
						g_task->precalculate_init_callback_ref = G91init_callback_precalculate;
						break;
					case 94*steps_per_unit_Z_2210: //G94 Units per Minute Mode
//						s->G94G95 = 0;
						g_task = add_empty_task();
						g_task->init_callback_ref = G94;
						g_task->precalculate_init_callback_ref = G94init_callback_precalculate;
						break;
					case 95*steps_per_unit_Z_2210: //G95 - is Units per Revolution Mode
//						s->G94G95 = 1;
						g_task = add_empty_task();
						g_task->init_callback_ref = G95;
						g_task->precalculate_init_callback_ref = G95init_callback_precalculate;
//						g_task->init_callback_ref = calibrate_init_callback;
//						s->sync = 1;
						break;
					case 0://G0 command packed into 2210_400 format
						G01parse(line+char_counter, G00code);
						return;	
					case 1*steps_per_unit_Z_2210://G1 command packed into 2210_400 format
						G01parse(line+char_counter, G01code);
						return;	
					case 2*steps_per_unit_Z_2210: //G2 CW ARC
						G03parse(line+char_counter,CCW); //
						break;
					case 3*steps_per_unit_Z_2210: //G3 CCW ARC
						G03parse(line+char_counter, CW); //!! G2 and G3 codes inverted for classic lathe where 
							//cutting tool under Z axis
						return;	
					case 33*steps_per_unit_Z_2210: //G33
						G33parse(line+char_counter - 1);
						return;	
					case 76*steps_per_unit_Z_2210: //G76 threading cycle
						G33parse(line+char_counter - 1);
						return;	
					case 98*steps_per_unit_Z_2210: //G98 threading cycle
						G98G99parse(line+char_counter - 1, G98code);
						return;	
					case 99*steps_per_unit_Z_2210: //G98 threading cycle
						G98G99parse(line+char_counter - 1, G99code);
						return;	
				}
				break;
			case 'X':
			case 'Z':
				switch(command){
					case 0*steps_per_unit_Z_2210://G0 command packed into 2210_400 format
						G01parse(line+char_counter - 1,G00code);
						return;	
					case 1*steps_per_unit_Z_2210://G1 command packed into 2210_400 format
						G01parse(line+char_counter - 1,G01code);
						return;	
					case 3*steps_per_unit_Z_2210: //G3
						G03parse(line+char_counter - 1,0);
						return;	
					case 33*steps_per_unit_Z_2210: //G33
						G33parse(line+char_counter - 1);
						return;	
				}
				break;
			case 'F':
//				gpos->F = str_f_to_2210(line, &char_counter);
//				if (gpos->F < 36000) //
//					gpos->F = 36000; // the slowest feed that cat fit into limit of 8.24 format
				break;
// repeat same command				
						
		}
	}

}

G_pipeline_t* G_parse(char *line, G_pipeline_t *gpos){
  uint8_t char_counter = 0;  
//	cb_init_by_top(&gp_cb,&init_gp);

//	int x_old = gpos->X, z_old = gpos->Z;
	char letter;
//	char *end;

  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter++];
		switch(letter){
			case 'X':
				gpos->Xr = str_f_to_steps2210(line, &char_counter) >> 1;
				gpos->X = fixedpt_xmul2210(gpos->Xr,z_to_x_factor2210);
//				gpos->X >>= 1; //Fusion360 generate X in diameter mode, so divide by 2
				break;
			case 'Z':
				gpos->Z = str_f_to_steps2210(line, &char_counter);
				break;
			case 'I':
				gpos->I = str_f_to_steps2210(line, &char_counter);
				break;
			case 'K':
				gpos->K = str_f_to_2210(line, &char_counter); //str_f_to_steps2210(line, &char_counter);
				break;
			case 'M':
				gpos->M = str_f_to_2210(line, &char_counter); //str_f_to_steps2210(line, &char_counter);
				break;
			case 'F':
				gpos->F = str_f_to_2210(line, &char_counter);
//				if (gpos->F < 36000) //
//					gpos->F = 36000; // the slowest feed that cat fit into limit of 8.24 format			
				break;
			case 'J':
				gpos->J = str_f_to_2210(line, &char_counter);
				break;
			case 'R':
				gpos->R = str_f_to_2210(line, &char_counter);
				break;
			case 'Q':
				gpos->Q = str_f_to_2210(line, &char_counter);
				break;
			case 'H':
				gpos->H = str_f_to_2210(line, &char_counter);
				break;
			case 'E':
				gpos->E = str_f_to_2210(line, &char_counter);
				break;
			case 'L':	
				gpos->L = str_f_to_2210(line, &char_counter);
				break;
			case 'P':
				gpos->P = str_f_to_2210(line, &char_counter);
				break;
		}
	}

//	cb_push_back(&gp_cb, &init_gp);
	return &init_gp; //gp_cb.top;
}

void G98G99parse(char *line, uint8_t G98G99){
//todo
}


void calibrate_init_callback(state_t *s){ 
	// todo not sure if it's working from timer interrupt where load_task is started...
	if(s->syncbase == TIM2){
		s->prescaler = s->syncbase->PSC + 1;
		return;
	}
	LL_TIM_DisableCounter(s->syncbase); // pause current timer
	// reset calibrated value
	s->prescaler = 0;
	s->function = calibrate_callback;
	LL_TIM_SetAutoReload(TIM1,0xFFFF);
	TIM1->CNT = 0;

	s->syncbase->CNT = 1;
	LL_TIM_EnableIT_UPDATE(s->syncbase);
	LL_TIM_DisableARRPreload(s->syncbase);
	LL_TIM_SetAutoReload(s->syncbase,10);
	LL_TIM_EnableCounter(s->syncbase); // start current timer
//	while(s->async_z==0);
}

void calibrate_callback(state_t *s){
	if(TIM1->CNT == 0){
		LL_TIM_EnableCounter(TIM1);
	} else {
		LL_TIM_DisableCounter(TIM1);
		s->prescaler = (TIM1->CNT - 110)/s->syncbase->ARR;
		LL_TIM_DisableCounter(s->syncbase); // pause current timer
		LL_TIM_EnableARRPreload(s->syncbase);
	}
}




const uint8_t rampup[] = {
	75,
	65,
	50,
	33,
	28,
	24,
	20,
	18,
	16,
	15,
	14,
	13,
	13,
	12,
	11,
	11,
	11,
	10,
	10,
	10,
	9,
	9,
	9,
	9,
	8,
	8,
	8,
	8,
	8,
	8,
	7,
	7,
	7,
	7,
	7,
	7,
	7,
	7,
	7,
	6,
	6,
	6,
	6,
	6,
	5,
	5,
	5,
	5,
	4,
	4,
	4,
	3,
	2,
	1,
};
