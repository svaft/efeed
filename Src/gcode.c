#include "gcode.h"
#include "fsm.h"

fixedpt command;
G_pipeline init_gp={0,0,0,0,0};

G_task gt[30];
G_pipeline gp[10];


void load_next_task(){
	cb_pop_front(&task_cb, &state.current_task);
	if(state.current_task.init_callback_ref){
		state.current_task.init_callback_ref(); // task specific init
	}
	state.Q824set = state.current_task.F; // load feed value
}

void G94(state_t* s){
//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);

	LL_TIM_DisableCounter(TIM2); // pause async timer
	LL_TIM_DisableUpdateEvent(TIM2);
	// connect async timer:
	s->syncbase = TIM2; 									// sync with internal clock source(virtual spindle, "async" to main spindle)
//	LL_TIM_DisableARRPreload(s->syncbase); // prepare timer start after EnableCounter plus one timer tick to owerflow

	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR1); 				//trigger by asnyc timer TIM2(async mode)
	LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
//	TIM3->ARR = min_pulse;
//	LL_TIM_DisableIT_UPDATE(TIM3);
//	LL_TIM_GenerateEvent_UPDATE(TIM3); // load arr without calling interrupt. maybe disable arr preload here too?
}

void do_fsm_move_start2(state_t* s){
	load_next_task(); // load first task from queue
	
//	LL_TIM_DisableARRPreload(s->syncbase); // prepare timer start after EnableCounter plus one timer tick to owerflow
	s->syncbase->ARR = 1;
	s->syncbase->CNT = 1; // set ARR=CNT to start pulse generation on next count increment after EnableCounter.
	// disclamer: why not to generate	update event by setting in EGR UG bit? with hardware logic analyzer all works fine,
	// but in simulator this first pulse not generater propertly by UG
	
	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);
	LL_TIM_EnableUpdateEvent(s->syncbase);
	LL_TIM_EnableCounter(s->syncbase);

	LL_TIM_DisableIT_UPDATE(TIM2);
	LL_TIM_GenerateEvent_UPDATE(s->syncbase);
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
}

void do_fsm_move2(state_t* s){
	fixedptu set_with_fract = fixedpt_add(s->Q824set, s->fract_part); // calculate new step delay with fract from previous step
	s->syncbase->ARR = fixedpt_toint(set_with_fract) - 1;
	s->fract_part = fixedpt_fracpart( set_with_fract ); // save fract part for future use on next step
	s->current_task.steps_to_end--;
}


void do_fsm_move_end2(state_t* s){
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);

	if (s->sync) {
		disable_encoder_ticks(); 										//reset interrupt for encoder ticks, only tacho todo async mode not compatible now
		LL_TIM_CC_DisableChannel(s->syncbase, LL_TIM_CHANNEL_CH3);	// configure TACHO events on channel 3
	} else {
		LL_TIM_DisableCounter(s->syncbase); // pause async timer
	}
	LL_TIM_DisableIT_UPDATE(s->syncbase);

	LL_TIM_DisableUpdateEvent(s->syncbase);
}


//__STATIC_INLINE 
//G_task * add_empty_task(){
//	cb_push_back_empty(&task_cb);
//	return task_cb.top;
//}

//__STATIC_INLINE 
G_task* get_last_task( void ){
	return (G_task *)task_cb.top;
}



void command_parser(char *line){
  uint8_t char_counter = 0;  
	char letter;
//	char *end;
  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    letter = line[char_counter++];
		switch(letter){
			case 'G':
				command = str_f_to_steps2210(line, &char_counter);
				switch(command){
					//todo now its only one G command per line supported
					case 4*400*1024: //G4 dwell, pause P seconds
						G04parse(line+char_counter);
						break;
					case 36864000: //G90  absolute distance mode
						
						break;
					case 38502400: //G94 Units per Minute Mode
						state.G94G95 = 0;
						break;
					case 38912000: //G95 - is Units per Revolution Mode
						state.G94G95 = 1;
//						state.sync = 1;
						break;
					case 409600://G1 command packed into 2210_400 format
						G01parse(line+char_counter);
						return;	
					case 819200: //G2 CW ARC
						G03parse(line+char_counter,CCW); //
						break;
					case 1228800: //G3 CCW ARC
						G03parse(line+char_counter, CW); //!! G2 and G3 codes inverted for classic lathe where 
							//cutting tool under Z axis
						return;	
					case 13516800: //G33
						return;	
				}
				break;
			case 'X':
			case 'Z':
				switch(command){
					case 409600://G1 command packed into 2210_400 format
						G01parse(line+char_counter - 1);
						return;	
					case 1228800: //G3
						G03parse(line+char_counter - 1,0);
						return;	
					case 13516800: //G33
						return;	
				}
				break;
			case 'F':
				init_gp.F = str_f_to_2210(line, &char_counter);

				break;
// repeat same command				
						
		}
	}

}

G_pipeline* G_parse(char *line){
  uint8_t char_counter = 0;  
//	cb_init_by_top(&gp_cb,&init_gp);

//	int x_old = init_gp.X, z_old = init_gp.Z;
	char letter;
//	char *end;

  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter++];
		switch(letter){
			case 'X':
				init_gp.X = str_f_to_steps2210(line, &char_counter);
				init_gp.X >>= 1; //Fusion360 generate X in diameter mode, so divide by 2
				break;
			case 'Z':
				init_gp.Z = str_f_to_steps2210(line, &char_counter);
				break;
			case 'I':
				init_gp.I = str_f_to_steps2210(line, &char_counter);
				break;
			case 'K':
				init_gp.K = str_f_to_steps2210(line, &char_counter);
				break;
			case 'F':
				init_gp.F = str_f_to_2210(line, &char_counter);
				break;
			case 'P':
				init_gp.P = str_f_to_2210(line, &char_counter);
				break;
		}
	}

//	cb_push_back(&gp_cb, &init_gp);
	return &init_gp; //gp_cb.top;
}







