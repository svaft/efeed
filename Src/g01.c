#include "gcode.h"

#include "g01.h"
#include "math.h"


uint32_t steps_to_end_shadow =0;
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
	steps_to_end_shadow = s->current_task_ref->steps_to_end;

	if(s->current_task_ref->steps_to_end > 100000)
		Error_Handler2(3);
	dxdz_callback_precalculate(s);
//	s->current_task_ref->steps_to_end = 1; 
}

//int pcc = 0;
int break5972 = 0;
void dxdz_callback_precalculate(state_t* s){
//	pcc++;
//	s->precalculate_end = false;
	if(s->current_task_ref->steps_to_end == 0)
		Error_Handler2(5);

	if(s->current_task_ref->steps_to_end > 100000)
		Error_Handler2(4);

	substep_t *sb = substep_cb.top;
	int e2 = s->err;
	int32_t delay = -1;
//	if(substep_cb.count2 == 5972) //some bug is here
//		break5972 = 1;
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
//	s->current_task_ref->steps_to_end--;
	steps_to_end_shadow--;
	if(steps_to_end_shadow == 0){
		s->current_task_ref->unlocked = true;
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
	s->gcode = 01;
	G00G01init_callback(s);
}

void G33init_callback(state_t* s){
	if(s->G94G00tmp == true){
		s->G94G00tmp = false;
		switch_to_sync(s);
	}
	s->gcode = 33;
	s->sync = true;
	if(s->current_task_ref->multistart_thread > 1){
		int degree = 3600/s->current_task_ref->multistart_thread; // degree to delay
		s->delay += degree;
		if(s->delay >= 3600)
			s->delay = 0;
	} else {
		s->delay = 0;
	}

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

	s->Q824set = s->current_task_ref->F;
	s->prescaler = s->syncbase->PSC;
//	TIM3->CCER = 0;
	s->initial_task_X_pos = s->global_X_pos; // save current position to return here if we decide to repeat last move
	s->initial_task_Z_pos = s->global_Z_pos;

	XDIR = s->current_task_ref->x_direction;
	ZDIR = s->current_task_ref->z_direction;
	if(s->current_task_ref->dz > s->current_task_ref->dx){
		s->current_task_ref->steps_to_end = s->current_task_ref->dz;
		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));
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
	s->rised = false;
}


int grefM = 0;
void G33parse(char *line){
	state_t *s = &state_precalc;
	G_pipeline_t gref = {0};
//	s->init = true;
	if(s->G90G91 == G90mode){
		G_parse(line, &init_gp);
		scheduleG00G01move(init_gp.X, init_gp.Z, init_gp.K, 33);
	} else {
		G_parse(line, &gref);
		grefM = fixedpt_toint2210(gref.M);
		scheduleG00G01move(gref.X, gref.Z, gref.K, 33);
	}
	return;	
}

void G01parse(char *line, bool G00G01){ //~60-70us
//	int x0  = init_gp.X 	& ~1uL<<(FIXEDPT_FBITS2210-1); //get from prev gcode
//	int x0r = init_gp.Xr 	& ~1uL<<(FIXEDPT_FBITS2210-1); //save pos from prev gcode
//	int z0  = init_gp.Z 	& ~1uL<<(FIXEDPT_FBITS2210-1);
	state_t *s = &state_precalc;
	G_pipeline_t gref = {0};
//	gref.X = gref.Xr = gref.Z = gref.F = 0;
//	s->init = true;
	if(s->G90G91 == G90mode){
		G_parse(line, &init_gp);
		scheduleG00G01move(init_gp.X, init_gp.Z, init_gp.F, G00G01);
	} else {
		G_parse(line, &gref);
		scheduleG00G01move(gref.X, gref.Z, gref.F, G00G01);
	}
	return;
}

void scheduleG00G01move(int X, int Z, int F, uint8_t G00G01G33){
/* make G00 or G01 move from current pos to X,Z position with provided feed F. If feed = -1 move with G00, 
	if zero - move with previous feed speed
*/
	int z0 = init_gp.Z, x0 = init_gp.X;
	state_t *s = &state_precalc;
	int dx,dz, xdir,zdir;
	if(s->G90G91 == G91mode){ // incremental mode
		init_gp.Z += Z;
		init_gp.X += X;
		if(Z >= 0){ // go from left to right
			dz = Z;
			zdir = zdir_forward;
		} else { // go back from right to left
			dz = -Z;
			zdir = zdir_backward;
		}
		if(X >= 0){ // go backward(away from stock)
			dx = X;
			xdir = xdir_backward;
		} else { // go forward (to center of stock)
			dx = - X;
			xdir = xdir_forward;
		}
	} else { //absolute mode
		init_gp.Z = Z;
		init_gp.X = X;

		if(Z > s->task_destination_Z_pos){ // go from left to right
			dz = Z - z0;
			zdir = zdir_forward;
		} else { // go back from right to left
			dz = z0 - Z;
			zdir = zdir_backward;
		}
		if(X > x0){ // go backward(away from stock)
			dx = X - x0;
			xdir = xdir_backward;
		} else { // go forward(to center of stock)
			dx = x0 - X;
			xdir = xdir_forward;
		}
	}
	if(dz == 0 && dx == 0)
		return;
	G_task_t *prev_task = get_last_task();
	G_task_t *gt_new_task = 0;
	gt_new_task = add_empty_task();


//		G94G95; // 0 - unit per min, 1 - unit per rev
	if( (s->G94G95 == G95code && G00G01G33 == G01code ) || G00G01G33 == G33code){ 	// unit(mm) per rev
		gt_new_task->F = str_f2210mm_rev_to_delay1616(F); //todo inch support
		gt_new_task->multistart_thread = grefM;
		// пересчет подачи по длине линии как в коде ниже для G94
	} else { 											// unit(mm) per min
		if(G00G01G33 == G00code){
			if(dx == 0)
				gt_new_task->F = 10<<16; //4285pps
			else
				gt_new_task->F = 20<<16;
		} else {
			// вычисляем длину линии как корень квадратный от суммы квадратов катетов. на этом шаге вычисляем сумму квадратов катетов:
			uint64_t il = (int64_t)dz*dz+(int64_t)dx*dx;//(int64_t)(Xr-x0r)*(Xr-x0r)+(int64_t)dz*dz;
			gt_new_task->len_f = sqrtf(il); // получили длину отрезка в условных единицах
			uint32_t len = gt_new_task->len_f; //
			/* далее производим пересчет скорости подачи в проекции на основную ось, 
			по которой идет отсчет шагов по алгоритму Брезенхэма. 
			Так, для прямой линии её проекция будет равна длине линии, а для 
			линии по углом 45 градусов ее проекция будет в 1.41меньше, соответственно для сохранения постоянной
			скорости подачи исходная величина F должна быть скорректирована	*/
			uint32_t ff = (async_steps_factor * (len>>10) / (dz > dx ? fixedpt_toint2210(dz) : fixedpt_toint2210(dx)))<<10; //todo to float?
			float f1 = ff;
			float f2 = F;
			float f3 = f1 * 65536.0f / f2;
//			fixedptu f = f3;
			gt_new_task->F = f3; //f << 16; // translate to 16.16 format used for delays
		}
	}

	gt_new_task->stepper = true;
	gt_new_task->callback_ref = dxdz_callback;
	gt_new_task->dx =  fixedpt_toint2210(dx);
	gt_new_task->dz =  fixedpt_toint2210(dz);
	gt_new_task->x_direction = xdir;
	gt_new_task->z_direction = zdir;

	if(prev_task->stepper == true && prev_task->z_direction == zdir){
		prev_task->skiprampdown = true;
	}

	switch(G00G01G33){
		case G00code:
			gt_new_task->init_callback_ref = G00init_callback;
			break;
		case G01code:
			gt_new_task->init_callback_ref = G01init_callback;
			break;
		case G33code:
			gt_new_task->init_callback_ref = G33init_callback;
			break;
	} 
	gt_new_task->precalculate_init_callback_ref = G01init_callback_precalculate;
	gt_new_task->precalculate_callback_ref 			= dxdz_callback_precalculate;
}

/*
void G01parsed(int x0, int x0r, int z0, int F, int X, int Z,  int Xr, bool G00G01){
	state_t *s = &state_precalc;
	int dx,dz, xdir,zdir;
	if(s->G90G91 == G91mode){
		if(Z >= 0){ // go from left to right
			dz = Z;
			zdir = zdir_forward;
		} else { // go back from right to left
			dz = -Z;
			zdir = zdir_backward;
		}
		if(X >= 0){ // go backward(away from stock)
			dx = X;
			xdir = xdir_backward;
		} else { // go forward(to center of stock)
			dx = - X;
			xdir = xdir_forward;
		}
	} else {
		if(Z > z0){ // go from left to right
			dz = Z - z0;
			zdir = zdir_forward;
		} else { // go back from right to left
			dz = z0 - Z;
			zdir = zdir_backward;
		}
		if(X > x0){ // go backward(away from stock)
			dx = X - x0;
			xdir = xdir_backward;
		} else { // go forward(to center of stock)
			dx = x0 - X;
			xdir = xdir_forward;
		}
	}
	G_task_t *prev_task = get_last_task();
	G_task_t *gt_new_task = 0;
	gt_new_task = add_empty_task();


//		bool G94G95; // 0 - unit per min, 1 - unit per rev
 	if(s->G94G95 == G95code && G00G01 == G01code){ 	// unit(mm) per rev
		gt_new_task->F = str_f2210mm_rev_to_delay1616(F); //todo inch support
		// пересчет подачи по длине линии как в коде ниже для G94
	} else { 											// unit(mm) per min
		if(G00G01 == G00code){
			if(dx == 0)
				gt_new_task->F = 10<<16; //4285pps
			else
				gt_new_task->F = 20<<16;
		} else {
			// вычисляем длину линии как корень квадратный от суммы квадратов катетов. на этом шаге вычисляем сумму квадратов катетов:
			uint64_t il = (int64_t)(Xr-x0r)*(Xr-x0r)+(int64_t)dz*dz;
			gt_new_task->len_f = sqrtf(il); // получили длину отрезка в услонвых единицах
			uint32_t len = gt_new_task->len_f; //
			// далее производим пересчет скорости подачи в проекции на основную ось, 
			//по которой идет отсчет шагов по алгоритму Брезенхэма. 
			//Так, для прямой длина линччии её проекция будет равна длине линии, а для 
			//линии по углом 45 градусов ее проекция будет в 1.41меньше, соответственно для сохранения постоянной
			//скорости подачи исходная величина F должна быть скорректирована
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
*/


