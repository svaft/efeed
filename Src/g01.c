#include "gcode.h"
#include "g01.h"
#include "math.h"


extern uint8_t ramp_posZ;
extern uint8_t ramp_posX;

/**
* @brief  G00init_callback
* @retval void.
  */
void G00init_callback(state_t* s){
	if(s->flags.G94G95 == G95code && s->G94G00tmp == false){
		s->G94G00tmp = true;
		switch_to_async(s);
	}
	G00G01init_callback3(s);
}

/**
* @brief  G01init_callback
* @retval void.
  */
void G01init_callback(state_t* s){
	if(s->G94G00tmp == true){
		s->G94G00tmp = false;
		switch_to_sync(s);
		
		s->sync = true;// todo
		s->delay = 0;
	}
	s->gcode = 01;
	G00G01init_callback3(s);
}
/**
* @brief  G33init_callback
* @retval void.
  */
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

	G00G01init_callback3(s);
//	s->function = do_fsm_move33;
}

/**
* @brief  G00G01init_callback3
* @retval void.
  */
void G00G01init_callback3(state_t* s){
	s->initial_task_X_pos = s->global_X_pos; // save current position to return here if we decide to repeat last move
	s->initial_task_Z_pos = s->global_Z_pos;

	XDIR = s->current_task_ref->flags.x_direction;
	ZDIR = s->current_task_ref->flags.z_direction;
//	TIM1->PSC = 0; //fixedpt_toint(s->current_task_ref->Fx) - 1;
//	TIM3->PSC = 0; //fixedpt_toint(s->current_task_ref->Fz) - 1;
	s->x1616set = s->current_task_ref->Fx;
	s->x1616fract_part = fixedpt_fracpart(s->x1616set);
	s->z1616set = s->current_task_ref->Fz;
	s->z1616fract_part = fixedpt_fracpart(s->z1616set);
	
	LL_TIM_DisableARRPreload(TIM3);
	if(s->current_task_ref->Fx>0){
		TIM1->ARR = fixedpt_toint(s->current_task_ref->Fx) - 1;
		TIM1->SR = 0;
//		TIM1->CNT = 0;
		LL_TIM_EnableIT_UPDATE(TIM1);
		LL_TIM_EnableCounter(TIM1);
	}
	if(s->current_task_ref->Fz>0){
		if(ramp_posZ == 0){
			TIM3->ARR = fixedpt_toint(s->current_task_ref->Fz) - 1;
			TIM3->SR = 0;
	//		TIM3->CNT = 0;
		}
			LL_TIM_EnableIT_UPDATE(TIM3);
			LL_TIM_EnableCounter(TIM3);
	}
}


/**
* @brief  dz_callback by TIM3
* @retval void.
  */
void dz_callback(state_t* s){
	fixedptu set_with_fract = fixedpt_add(s->z1616set, s->x1616fract_part); // calculate new step delay with fract from previous step
	TIM3->ARR = fixedpt_toint(set_with_fract) - 1;							// load step delay to ARR register
	s->z1616fract_part = fixedpt_fracpart( set_with_fract );
	s->current_task_ref->flags.z_direction == zdir_forward ? s->global_Z_pos++ : s->global_Z_pos--;

	if(s->current_task_ref->flags.skiprampdownZ!=true && ramp_posZ >= s->current_task_ref->steps_to_endZ){
		TIM3->ARR = s->rampTab[--ramp_posZ];
	} else {
		if(TIM3->ARR < s->rampTab[ramp_posZ]){
			TIM3->ARR = s->rampTab[ramp_posZ++];
		}
	}

	if(--s->current_task_ref->steps_to_endZ == 0){
		LL_TIM_DisableCounter(TIM3);
		state_hw.current_task_ref->callback_refZ = 0;
	}		//		state_hw.current_task_ref = 0;
	s->rised = false;
}

/**
* @brief  dx_callback by TIM1
* @retval void.
  */
void dx_callback(state_t* s){
	fixedptu set_with_fract = fixedpt_add(s->x1616set, s->x1616fract_part); // calculate new step delay with fract from previous step
	TIM1->ARR = fixedpt_toint(set_with_fract) - 1;							// load step delay to ARR register
	s->x1616fract_part = fixedpt_fracpart( set_with_fract ); 	
	s->current_task_ref->flags.x_direction == xdir_backward ? s->global_X_pos++ : s->global_X_pos--;

	if(s->current_task_ref->flags.skiprampdownX!=true && ramp_posX >= s->current_task_ref->steps_to_endX){
		TIM1->ARR = s->rampTab[--ramp_posX];
	} else {
		if(TIM1->ARR < s->rampTab[ramp_posX]){
			TIM1->ARR = s->rampTab[ramp_posX++];
		}
	}

	s->rised = false;
	if(--s->current_task_ref->steps_to_endX == 0){
		LL_TIM_DisableCounter(TIM1);
		state_hw.current_task_ref->callback_refX = 0;
	}		//		state_hw.current_task_ref = 0;
}


int grefM = 0;

/**
* @brief  G01parse
* @retval void.
  */
void G01parse(char *line, uint8_t gcode){ //~60-70us
	state_t *s = &state_precalc;
	G_pipeline_t gref = {0};
	if(s->flags.G90G91 == G90mode){
		G_parse(line, &init_gp);
		if(gcode == 0 || gcode == 1){
			scheduleG00G01move3(init_gp.X, init_gp.Z, init_gp.F, init_gp.Xr,  gcode);
		}
		if(gcode == 33)
			scheduleG00G01move3(init_gp.X, init_gp.Z, init_gp.K,0, 33);
	} else {
		G_parse(line, &gref);
		if(gcode == 0 || gcode == 1){
			scheduleG00G01move3(gref.X, gref.Z, gref.F, gref.Xr, gcode);
		}
		if(gcode == 33){
			grefM = fixedpt_toint2210(gref.M);
			scheduleG00G01move3(gref.X, gref.Z, gref.K,0, 33);
		}
	}
}


/**
* @brief  scheduleG00G01move3
* @retval void.
  */
void scheduleG00G01move3(int X, int Z, int F, int Xr, uint8_t G00G01G33){
/* make G00 or G01 move from current pos to X,Z position with provided feed F. If feed = -1 move with G00, 
	if zero - move with previous feed speed
*/
	int z0 = init_gp.Z, x0 = init_gp.X;
	state_t *s = &state_precalc;
	int dx,dz, xdir,zdir;
	if(s->flags.G90G91 == G91mode){ // incremental mode
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
	
	if(prev_task->flags.stepper == true) {
		if(prev_task->flags.z_direction == zdir){
			prev_task->flags.skiprampdownZ = true;
		}
		if(prev_task->flags.x_direction == xdir){
			prev_task->flags.skiprampdownX = true;
		}
	}

	G_task_t *gt_new_task = 0;
	if(task_cb.count == 2){
		volatile int q = 1; 
	}
	gt_new_task = add_empty_task();
	if(task_cb.count == 3){
		volatile int q = 1; 
	}
//		G94G95; // 0 - unit per min, 1 - unit per rev
	if( (s->flags.G94G95 == G95code && G00G01G33 == G01code ) || G00G01G33 == G33code){ 	// unit(mm) per rev
		gt_new_task->Fz = str_f2210mm_rev_to_delay1616(F); //todo inch support
		gt_new_task->Fx = 0;

		gt_new_task->multistart_thread = grefM;
	} else { 											// unit(mm) per min
		
		if(G00G01G33 == G00code) {
			F = 800*1024;// 600mm/min
			// Z: 10mm/min =0737280000, x mm/min = 7372800000/x
			// X: 10мм/мин =1179648000
			gt_new_task->Fz = 7549747200000 / F;
			gt_new_task->Fx =  12079595520000 / F;

//			uint32_t Fint = F / 1024; 
//			gt_new_task->Fx = 737280000 /Fint; // precalculated for 600mm/min
//			gt_new_task->Fz = 12288000; // precalculated for 600mm/min
//			gt_new_task->F = 10<<16;
		}// else 
		{
			// вычисляем длину линии как корень квадратный от суммы квадратов катетов. на этом шаге вычисляем сумму квадратов катетов:
			int64_t dxscaled = dx * 16 / 10;
			uint64_t il = (int64_t)dz*dz+(int64_t)dxscaled*dxscaled;//(int64_t)(Xr-x0r)*(Xr-x0r)+(int64_t)dz*dz;
			float len_f = sqrtf(il); // получили длину отрезка в условных единицах
			
			const float timerFreq =(float)(72000000/(TIM2->ARR+1)); //300.000 ticks per second for PSC=239
			const float pathLenConst = (float)(64.*1024.* 60.0 * 1024.0*timerFreq / (z_steps_unit / z_screw_pitch)); // 7.549.747.200.000
			float pathLen = len_f * pathLenConst / (float)F;

			gt_new_task->Fz = dz > 0 ? pathLen / dz : 0;
			gt_new_task->Fx = dx > 0 ? pathLen / dx : 0;
		}
	}

	gt_new_task->flags.stepper = true;
	gt_new_task->callback_refZ = dz_callback;
	gt_new_task->callback_refX = dx_callback;
	gt_new_task->steps_to_endX = gt_new_task->dx =  fixedpt_toint2210(dx);
	gt_new_task->steps_to_endZ = gt_new_task->dz =  fixedpt_toint2210(dz);
	// no need to precalculate, so skip callbacks and unlock task
	gt_new_task->flags.unlocked = true; 

	gt_new_task->flags.x_direction = xdir;
	gt_new_task->flags.z_direction = zdir;


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
}
