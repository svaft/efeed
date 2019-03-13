#include "gcode.h"
#include "fsm.h"
#include "g01.h"


// called from load_task
void G01init_callback(void){
	state_t *s = &state;
//1. set state.function
//	2. set ARR
//	3. set channels
	state.function = do_fsm_move2;
	s->syncbase->ARR = fixedpt_toint(s->current_task.F) - 1;
	s->err = (s->current_task.dx > s->current_task.dz ? s->current_task.dx : -s->current_task.dz) >> 1;
	dxdz_callback();
//	s->current_task.steps_to_end = 1; 

}

// called from TIM3 on end of the stepper pulse to set output channel configuration for next pulse
void dxdz_callback(){
	state_t *s = &state;
//			debug();
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

//	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
//		s->current_task.steps_to_end = 0; // end of arc
//		return;
//	}

	s->current_task.steps_to_end--;
}


void G01parse(char *line){ //~60-70us
	int x0 = init_gp.X & ~1uL<<10; //get from prev gcode
	int z0 = init_gp.Z & ~1uL<<10;
	G_pipeline *gref = G_parse(line);

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
	G_task *gt_new_task = add_empty_task();
	gt_new_task->callback_ref = dxdz_callback;
	gt_new_task->dx =  fixedpt_toint2210(dx);
	gt_new_task->dz =  fixedpt_toint2210(dz);
	gt_new_task->steps_to_end = gt_new_task->dz > gt_new_task->dx ? gt_new_task->dz : gt_new_task->dx;
	gt_new_task->x_direction = xdir;
	gt_new_task->z_direction = zdir;

//		bool G94G95; // 0 - unit per min, 1 - unit per rev
	if(state.G94G95 == 1){ 	// unit(mm) per rev
		gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
	} else { 											// unit(mm) per min
		gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
	}
	gt_new_task->init_callback_ref = G01init_callback;
//	gref->code = 1;
}
