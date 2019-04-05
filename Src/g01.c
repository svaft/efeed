#include "gcode.h"
#include "fsm.h"
#include "g01.h"



// called from load_task
void G01init_callback_precalculate(state_t* s){
	if(s->current_task.dz > s->current_task.dx){
		s->current_task.steps_to_end = s->current_task.dz;
		s->substep_axis = SUBSTEP_AXIS_X;
		s->err = -s->current_task.dz >> 1;
	} else{
		s->current_task.steps_to_end = s->current_task.dx;
		s->err = s->current_task.dx >> 1;
		s->substep_axis = SUBSTEP_AXIS_Z;
	}
	dxdz_callback_precalculate(s);
//	s->current_task.steps_to_end = 1; 
}



void dxdz_callback_precalculate(state_t* s){
	s->precalculate_end = false;

	substep_t *sb = substep_cb.top;
	int e2 = s->err;
	int32_t delay = -1;

	if (e2 >= -s->current_task.dx)	{ // step X axis
		s->err -= s->current_task.dz;
		if(s->substep_axis == SUBSTEP_AXIS_X){

//			delay = s->prescaler * (s->syncbase->ARR+1)*(abs(e2))/s->current_task.dx;
			delay = (1<<subdelay_precision)*(abs(e2))/s->current_task.dx;
		}
	}
	if (e2 <= s->current_task.dz)	{ // step Z axis
		s->err += s->current_task.dx;
		if(s->substep_axis == SUBSTEP_AXIS_Z){
//			delay = s->prescaler * (s->syncbase->ARR+1)*(abs(e2))/s->current_task.dz;
			delay = (1<<subdelay_precision)*(abs(e2))/s->current_task.dz;
		}
	}

	if(delay >=0){
		cb_push_back_empty_ref()->delay = delay;
	} else {
		if(substep_cb.count == 0 || sb->skip == 0){
			cb_push_back_empty(&substep_cb);
		} else {
			if(sb->skip == 255){
				cb_push_back_empty(&substep_cb);
			}
		}
		sb = substep_cb.top;
		sb->skip++;
	}
	s->current_task.steps_to_end--;
	if(s->current_task.steps_to_end == 0)
		s->precalculate_end = true;
}

// called from load_task
void G01init_callback(state_t* s){
//	1. set state.function
//	2. set ARR
//	3. set channels
	s->function = do_fsm_move2;
	s->syncbase->ARR = fixedpt_toint(s->current_task.F) - 1;
	TIM3->CCER = 0;
	XDIR = s->current_task.x_direction;
	ZDIR = s->current_task.z_direction;
	if(s->current_task.dz > s->current_task.dx){
		s->current_task.steps_to_end = s->current_task.dz;
		s->substep_axis = SUBSTEP_AXIS_X;
		s->err = -s->current_task.dz >> 1;
		MOTOR_Z_AllowPulse(); 
	} else{
		s->current_task.steps_to_end = s->current_task.dx;
		s->err = s->current_task.dx >> 1;
		s->substep_axis = SUBSTEP_AXIS_Z;
		MOTOR_X_AllowPulse(); 
	}

//	dxdz_callback(s);
//	s->current_task.steps_to_end = 1; 
}




// called from TIM3 on end of the stepper pulse to set output channel configuration for next pulse
//int lx=0, ly=0;
//uint32_t st = 0, st1 = 0;
void dxdz_callback(state_t* s){
	s->current_task.steps_to_end--;
}


void G01parse(char *line){ //~60-70us
	int x0 = init_gp.X & ~1uL<<10; //get from prev gcode
	int z0 = init_gp.Z & ~1uL<<10;
	G_pipeline_t *gref = G_parse(line);
	if(state_hw.init == false){
		init_gp.X = gref->X;
		init_gp.Z = gref->Z;
		state_hw.init = true;
		return;
	}
//	gref->Z = -32*1024;
//	gref->X = 6*1024;
	
	int dx,dz, xdir,zdir;
	if(gref->Z > z0){ // go from left to right
		dz = gref->Z - z0;
		zdir = zdir_forward;
	} else { // go back from right to left
		dz = z0 - gref->Z;
		zdir = zdir_backward;
	}
	if(gref->X > x0){ // go forward
		dx = gref->X - x0;
		xdir = xdir_forward;
	} else { // go back
		dx = x0 - gref->X;
		xdir = xdir_backward;
	}
	G_task_t *gt_new_task = add_empty_task();
	gt_new_task->stepper = true;
	gt_new_task->callback_ref = dxdz_callback;
	gt_new_task->dx =  fixedpt_toint2210(dx);
	gt_new_task->dz =  fixedpt_toint2210(dz);
		
//	gt_new_task->steps_to_end = gt_new_task->dz > gt_new_task->dx ? gt_new_task->dz : gt_new_task->dx;
	gt_new_task->x_direction = xdir;
	gt_new_task->z_direction = zdir;

//		bool G94G95; // 0 - unit per min, 1 - unit per rev
	if(state_hw.G94G95 == 1){ 	// unit(mm) per rev
		gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
	} else { 											// unit(mm) per min
		gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
	}
	gt_new_task->init_callback_ref = G01init_callback;
	gt_new_task->precalculate_init_callback_ref =  G01init_callback_precalculate;
	gt_new_task->precalculate_callback_ref = dxdz_callback_precalculate;
//	gref->code = 1;
}




/*
void dxdz_callback(state_t* s){
	debug();
	if(s->substep_mask){
		// sub-step done, restore channel config
//		debug1();
		// inverse substep channel activity:
		if(s->substep_mask == MOTOR_Z_CHANNEL){ 
			MOTOR_X_OnlyPulse();
		} else {
			MOTOR_Z_OnlyPulse();
		}
		s->substep_mask = 0;
		return;
	}

	TIM3->CCER = 0;
	int e2 = s->err;
	if (e2 >= -s->current_task.dx)	{ // step X axis
		if(s->err == 0 || s->substep_axis == SUBSTEP_AXIS_Z){
//			lx++;
			MOTOR_X_AllowPulse(); 
		} else {
			s->substep_mask = MOTOR_X_CHANNEL;
			MOTOR_X_BlockPulse(); // block pulse on next timer2 tick but set it by substep timer1
			uint32_t delay = s->prescaler * s->syncbase->ARR;
			// explanations: prescaler value for timer connected to encoder is unknown because its depend on rotating speed of the spindle,
			// so we trying to detect it with calibrate_callback and use it here. For an async(predefined) timer	we can use TI2->PSC. 		
			delay = delay*(abs(e2))/s->current_task.dx;// + min_pulse;
			LL_TIM_SetAutoReload(TIM1,delay);
//			debug1();
			LL_TIM_EnableCounter(TIM1);
		}
		s->err -= s->current_task.dz;
	}
	if (e2 <= s->current_task.dz)	{ // step Z axis
		if(s->err == 0 || s->substep_axis == SUBSTEP_AXIS_X){
//			ly++;
			MOTOR_Z_AllowPulse();
		} else {
			s->substep_mask = MOTOR_Z_CHANNEL;
			MOTOR_Z_BlockPulse(); // block pulse on next timer2 tick but set it by substep timer1
			uint32_t delay = s->prescaler * s->syncbase->ARR;
			// explanations: prescaler for timer connecter to encoder is unknown because its depend on rotating speed of the encoder,
			// so we trying to detect it with calibrate_callback and use it here. For async(predefined) timer	we can use TI2->PSC for it. 		
			delay = delay*(abs(e2))/s->current_task.dz;// + min_pulse;
			LL_TIM_SetAutoReload(TIM1,delay);
//			debug1();
			LL_TIM_EnableCounter(TIM1);
		}
		s->err += s->current_task.dx;
	}

//	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
//		s->current_task.steps_to_end = 0; // end of arc
//		return;
//	}

	s->current_task.steps_to_end--;
}
*/