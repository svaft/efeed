#include "gcode.h"

#include "g01.h"
#include "math.h"



void G01init_callback_precalculate(state_t* s){
	if(s->current_task_ref->dz > s->current_task_ref->dx){
		s->current_task_ref->steps_to_end = s->current_task_ref->dz;
		s->substep_axis = SUBSTEP_AXIS_X;
		s->err = -s->current_task_ref->dz >> 1;
	} else{
		s->current_task_ref->steps_to_end = s->current_task_ref->dx;
		s->err = s->current_task_ref->dx >> 1;
		s->substep_axis = SUBSTEP_AXIS_Z;
	}
	dxdz_callback_precalculate(s);
//	s->current_task_ref->steps_to_end = 1; 
}

//int pcc = 0;
int break5972 = 0;
void dxdz_callback_precalculate(state_t* s){
//	pcc++;
//	s->precalculate_end = false;

	substep_t *sb = substep_cb.top;
	int e2 = s->err;
	int32_t delay = -1;
	if(substep_cb.count2 == 5972) //some bug is here
		break5972 = 1;
	if (e2 >= -s->current_task_ref->dx)	{ // step X axis
		s->err -= s->current_task_ref->dz;
		if(s->substep_axis == SUBSTEP_AXIS_X){
			delay = ((1<<subdelay_precision)-1)*(abs(e2))/s->current_task_ref->dx;
		}
	}
	if (e2 <= s->current_task_ref->dz)	{ // step Z axis
		s->err += s->current_task_ref->dx;
		if(s->substep_axis == SUBSTEP_AXIS_Z){
			delay = ((1<<subdelay_precision)-1)*(abs(e2))/s->current_task_ref->dz;
		}
	}

	if(delay >= 0){
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
	s->current_task_ref->steps_to_end--;
	if(s->current_task_ref->steps_to_end == 0){
		s->current_task_ref->unlocked = true;

//		s->precalculate_end = true; // todo remove?
	}
}



void G00init_callback(state_t* s){
	if(s->G94G95 == G95code && s->G94G00tmp == false){
		s->G94G00tmp = true;
		switch_to_async(s);
	}
	G00G01init_callback(s);
}

void G01init_callback(state_t* s){
	if(s->G94G00tmp == true){
		s->G94G00tmp = false;
		switch_to_sync(s);
	}
	G00G01init_callback(s);
}

void G33init_callback(state_t* s){
	if(s->G94G00tmp == true){
		s->G94G00tmp = false;
		switch_to_sync(s);
	}
	s->gcode = 33;

	G00G01init_callback(s);
	s->function = do_fsm_move33;
}



// called from load_task
void G00G01init_callback(state_t* s){
//	1. set state.function
//	2. set ARR
//	3. set channels
	s->function = do_fsm_move2;
	s->syncbase->ARR = fixedpt_toint(s->current_task_ref->F) - 1;
//	if(s->syncbase->ARR < 75 ) // todo костыль, нулевое значение в таблице ускорения
//		s->syncbase->ARR = 75;

	s->Q824set = s->current_task_ref->F;

	s->prescaler = s->syncbase->PSC;
//	TIM3->CCER = 0;
	XDIR = s->current_task_ref->x_direction;
	ZDIR = s->current_task_ref->z_direction;
	if(s->current_task_ref->dz > s->current_task_ref->dx){
		s->current_task_ref->steps_to_end = s->current_task_ref->dz;
//#define XSTP	*((volatile uint32_t *) ((PERIPH_BB_BASE + (uint32_t)(  (uint8_t *)MOTOR_X_STEP_GPIO_Port+0xC 	- PERIPH_BASE)*32 + ( MOTOR_X_STEP_Pin_num*4 ))))
//#define BITBAND_PERI2(a,b) ((PERIPH_BB_BASE + (a-PERIPH_BASE)*32 + (b*4)))		
		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));

//		BITBAND_PERI2(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin_num); //(unsigned int *)((PERIPH_BB_BASE + (uint32_t)(  (uint8_t *)MOTOR_X_STEP_GPIO_Port+0xC 	- PERIPH_BASE)*32 + ( MOTOR_X_STEP_Pin_num*4 )));
		s->substep_pulse_on = 1;
		s->substep_pulse_off = 0;

		s->substep_axis = SUBSTEP_AXIS_X;
		s->err = -s->current_task_ref->dz >> 1;
		MOTOR_Z_AllowPulse();
		LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
	} else{
		s->current_task_ref->steps_to_end = s->current_task_ref->dx;
		s->err = s->current_task_ref->dx >> 1;
		s->substep_axis = SUBSTEP_AXIS_Z;

		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_Z_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_Z_STEP_Pin_num*4)));

//		s->substep_pin = &ZSTP;
		s->substep_pulse_on = 1;
		s->substep_pulse_off = 0;

		MOTOR_X_AllowPulse(); 
		LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
	}
}




// called from TIM3 on end of the stepper pulse to set output channel configuration for next pulse
//int lx=0, ly=0;
//uint32_t st = 0, st1 = 0;
void dxdz_callback(state_t* s){
	s->current_task_ref->steps_to_end--;
}



void G33parse(char *line){
	int x0 =  init_gp.X  & ~1uL<<(FIXEDPT_FBITS2210-1); //округляем предыдущее значение до целого числа шагов
	int x0r = init_gp.Xr & ~1uL<<(FIXEDPT_FBITS2210-1); //save pos from prev gcode
	int z0 =  init_gp.Z  & ~1uL<<(FIXEDPT_FBITS2210-1);
	state_t *s = &state_precalc;

	G_pipeline_t *gref = G_parse(line);
	if(s->init == false){
		init_gp.X = gref->X;
		init_gp.Z = gref->Z;
		s->init = true;
		return;
	}

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

//	uint64_t il = (int64_t)(gref->Xr-x0r)*(gref->Xr-x0r)+(int64_t)dz*dz;
	G_task_t *gt_new_task = 0;

	gt_new_task = add_empty_task();

	gt_new_task->F = str_f824mm_rev_to_delay824(gref->K); //todo inch support
	if(dx > dz){
//		float f2 = gref->K<<14;
//		float f3 = (rev_to_delay_f / f2)*2794369.09f; //float divide is faster then long?

//		gt_new_task->F = f3;
	}
	gt_new_task->stepper = true;
	gt_new_task->callback_ref = dxdz_callback;
	gt_new_task->dx =  fixedpt_toint2210(dx);
	gt_new_task->dz =  fixedpt_toint2210(dz);

//	gt_new_task->steps_to_end = gt_new_task->dz > gt_new_task->dx ? gt_new_task->dz : gt_new_task->dx;
	gt_new_task->x_direction = xdir;
	gt_new_task->z_direction = zdir;

	gt_new_task->init_callback_ref = G33init_callback;
	gt_new_task->precalculate_init_callback_ref =  G01init_callback_precalculate;
	gt_new_task->precalculate_callback_ref = dxdz_callback_precalculate;
}

void G01parse(char *line, bool G00G01){ //~60-70us
	int x0 = init_gp.X & ~1uL<<(FIXEDPT_FBITS2210-1); //get from prev gcode
	int x0r = init_gp.Xr & ~1uL<<(FIXEDPT_FBITS2210-1); //save pos from prev gcode
	int z0 = init_gp.Z & ~1uL<<(FIXEDPT_FBITS2210-1);
	state_t *s = &state_precalc;

	G_pipeline_t *gref = G_parse(line);
	if(s->init == false){
		init_gp.X = gref->X;
		init_gp.Z = gref->Z;
		init_gp.Xr = gref->Xr;
		s->init = true;
		return;
	}
	
	G01parsed(x0, x0r, z0, gref->F, gref->X, gref->Z,  gref->Xr, G00G01);
//	gref->code = 1;
}

void G01parsed(int x0, int x0r, int z0, int F, int X, int Z,  int Xr, bool G00G01){
	state_t *s = &state_precalc;
	int dx,dz, xdir,zdir;
	if(Z > z0){ // go from left to right
		dz = Z - z0;
		zdir = zdir_forward;
	} else { // go back from right to left
		dz = z0 - Z;
		zdir = zdir_backward;
	}
	if(X > x0){ // go forward
		dx = X - x0;
		xdir = xdir_forward;
	} else { // go back
		dx = x0 - X;
		xdir = xdir_backward;
	}

	G_task_t *prev_task = get_last_task();
	G_task_t *gt_new_task = 0;
	gt_new_task = add_empty_task();


//		bool G94G95; // 0 - unit per min, 1 - unit per rev
 	if(s->G94G95 == G95code && G00G01 == G01code){ 	// unit(mm) per rev
		gt_new_task->F = str_f824mm_rev_to_delay824(F); //todo inch support
	} else { 											// unit(mm) per min
		if(G00G01 == G00code){
			gt_new_task->F = 8<<16; //4285pps
		} else {
			// вычисляем длину линии как корень квадратный от суммы квадратов катетов. на этом шаге вычисляем сумму квадратов катетов:
			uint64_t il = (int64_t)(Xr-x0r)*(Xr-x0r)+(int64_t)dz*dz;
			gt_new_task->len_f = sqrtf(il); // SquareRoot64(il);
			uint32_t len = gt_new_task->len_f;
			uint32_t ff = (async_steps_factor * (len>>10) / (dz > dx ? fixedpt_toint2210(dz) : fixedpt_toint2210(dx)))<<10; //todo to float?
			float f1 = ff;
			float f2 = F;
			float f3 = f1 / f2;
			fixedptu f = f3;
			gt_new_task->F = f << 16; // translate to 16.16 format used for delays
		}
	}

	gt_new_task->stepper = true;
	gt_new_task->callback_ref = dxdz_callback;
	gt_new_task->dx =  fixedpt_toint2210(dx);
	gt_new_task->dz =  fixedpt_toint2210(dz);

//	gt_new_task->steps_to_end = gt_new_task->dz > gt_new_task->dx ? gt_new_task->dz : gt_new_task->dx;
	gt_new_task->x_direction = xdir;
	gt_new_task->z_direction = zdir;

	if(prev_task->stepper == true && prev_task->z_direction == zdir){
		prev_task->skiprampdown = true;
	}
	if(G00G01 == G00code)	
		gt_new_task->init_callback_ref = G00init_callback;
	else
		gt_new_task->init_callback_ref = G01init_callback;
	gt_new_task->precalculate_init_callback_ref =  G01init_callback_precalculate;
	gt_new_task->precalculate_callback_ref = dxdz_callback_precalculate;
}
