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

	XDIR = s->current_task_ref->x_direction;
	ZDIR = s->current_task_ref->z_direction;
	TIM1->PSC = 0; //fixedpt_toint(s->current_task_ref->Fx) - 1;
	TIM3->PSC = 0; //fixedpt_toint(s->current_task_ref->Fz) - 1;
	s->x1616set = s->current_task_ref->Fx;
	s->x1616fract_part = fixedpt_fracpart(s->x1616set);
	s->z1616set = s->current_task_ref->Fz;
	s->z1616fract_part = fixedpt_fracpart(s->z1616set);
	
	LL_TIM_DisableARRPreload(TIM3);
	if(s->current_task_ref->Fx>0){
		TIM1->ARR = fixedpt_toint(s->current_task_ref->Fx) - 1;
		TIM1->SR = 0;
		LL_TIM_EnableIT_UPDATE(TIM1);
		LL_TIM_EnableCounter(TIM1);
	}
	if(s->current_task_ref->Fz>0){
		TIM3->ARR = fixedpt_toint(s->current_task_ref->Fz) - 1;
		TIM3->SR = 0;
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
	s->current_task_ref->z_direction == zdir_forward ? s->global_Z_pos++ : s->global_Z_pos--;

	if(s->current_task_ref->skiprampdownZ!=true && ramp_posZ >= s->current_task_ref->steps_to_endZ){
		TIM3->ARR = rampup2[--ramp_posZ];
	} else {
		if(TIM3->ARR < rampup2[ramp_posZ]){
			TIM3->ARR = rampup2[ramp_posZ++];
		} else{
			s->z1616fract_part = fixedpt_fracpart( set_with_fract ); 	
		}
	}

	if(--s->current_task_ref->steps_to_endZ == 0){
		LL_TIM_DisableCounter(TIM3);
//		LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH2);
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
	s->current_task_ref->x_direction == xdir_backward ? s->global_X_pos++ : s->global_X_pos--;

	if(s->current_task_ref->skiprampdownX!=true && ramp_posX >= s->current_task_ref->steps_to_endX){
		TIM1->ARR = rampup2[--ramp_posX];
	} else {
		if(TIM1->ARR < rampup2[ramp_posX]){
			TIM1->ARR = rampup2[ramp_posX++];
		} else{
			s->x1616fract_part = fixedpt_fracpart( set_with_fract ); 	
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
* @brief  G33parse
* @retval void.
  */
void G33parse(char *line){
	state_t *s = &state_precalc;
	G_pipeline_t gref = {0};
//	s->init = true;
	if(s->flags.G90G91 == G90mode){
		G_parse(line, &init_gp);
		scheduleG00G01move3(init_gp.X, init_gp.Z, init_gp.K,0, 33);
	} else {
		G_parse(line, &gref);
		grefM = fixedpt_toint2210(gref.M);
		scheduleG00G01move3(gref.X, gref.Z, gref.K,0, 33);
	}
	return;	
}

/**
* @brief  G01parse
* @retval void.
  */
void G01parse(char *line, bool G00G01){ //~60-70us
//	int x0  = init_gp.X 	& ~1uL<<(FIXEDPT_FBITS2210-1); //get from prev gcode
//	int x0r = init_gp.Xr 	& ~1uL<<(FIXEDPT_FBITS2210-1); //save pos from prev gcode
//	int z0  = init_gp.Z 	& ~1uL<<(FIXEDPT_FBITS2210-1);
	state_t *s = &state_precalc;
	G_pipeline_t gref = {0};
//	gref.X = gref.Xr = gref.Z = gref.F = 0;
//	s->init = true;
	if(s->flags.G90G91 == G90mode){
		G_parse(line, &init_gp);
		scheduleG00G01move3(init_gp.X, init_gp.Z, init_gp.F, init_gp.Xr,  G00G01);
	} else {
		G_parse(line, &gref);
		scheduleG00G01move3(gref.X, gref.Z, gref.F, gref.Xr, G00G01);
	}
	return;
}


extern __IO uint32_t ttt;

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
	G_task_t *gt_new_task = 0;
	gt_new_task = add_empty_task();


//		G94G95; // 0 - unit per min, 1 - unit per rev
	if( (s->flags.G94G95 == G95code && G00G01G33 == G01code ) || G00G01G33 == G33code){ 	// unit(mm) per rev
		gt_new_task->Fz = str_f2210mm_rev_to_delay1616(F); //todo inch support
		gt_new_task->Fx = 0;

		gt_new_task->multistart_thread = grefM;
	} else { 											// unit(mm) per min
		
		if(G00G01G33 == G00code){
			F = 600*1024;// 600mm/min
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
			
//			uint32_t tttc = 0;
//			
//			int count = 100000;
//			tttc = ttt;
//			for(int a = 0; a < count;a++){
////					LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin_num);
//				WRITE_REG(MOTOR_Z_STEP_GPIO_Port->BSRR, (MOTOR_Z_STEP_Pin_num >> GPIO_PIN_MASK_POS) & 0x0000FFFFU);
//				
////		*((unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_Z_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_Z_STEP_Pin_num*4)))) = 1; //state_hw.substep_pulse_on;
//			}
//			tttc = ttt - tttc;

//			
//			tttc = ttt;
//			
//			uint32_t bbref = (PERIPH_BB_BASE + ((uint32_t)&(MOTOR_Z_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_Z_STEP_Pin_num*4));
//			for(int a = 0; a < count;a++){
////					LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin_num);
//				*((unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_Z_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_Z_STEP_Pin_num*4)))) = 1; //state_hw.substep_pulse_on;
//			}
//			tttc = ttt - tttc;

//			
			
			// вычисляем длину линии как корень квадратный от суммы квадратов катетов. на этом шаге вычисляем сумму квадратов катетов:
			int64_t dxscaled = dx * 16 / 10;
			uint64_t il = (int64_t)dz*dz+(int64_t)dxscaled*dxscaled;//(int64_t)(Xr-x0r)*(Xr-x0r)+(int64_t)dz*dz;
			gt_new_task->len_f = sqrtf(il); // получили длину отрезка в условных единицах
			
			const float timerFreq =(float)(72000000/(TIM2->ARR+1)); //300.000 ticks per second for PSC=239
			const float pathLenConst = (float)(64.*1024.* 60.0 * 1024.0*timerFreq / (z_steps_unit / z_screw_pitch)); // 7.549.747.200.000
			float pathLen = gt_new_task->len_f * pathLenConst / (float)F;

			gt_new_task->Fz = dz > 0 ? pathLen / dz : 0;
			gt_new_task->Fx = dx > 0 ? pathLen / dx : 0;
		}
	}

	gt_new_task->stepper = true;
	gt_new_task->callback_refZ = dz_callback;
	gt_new_task->callback_refX = dx_callback;
	gt_new_task->steps_to_endX = gt_new_task->dx =  fixedpt_toint2210(dx);
	gt_new_task->steps_to_endZ = gt_new_task->dz =  fixedpt_toint2210(dz);

	// no need to precalculate, so skip callbacks and unlock task
//	gt_new_task->precalculate_init_callback_ref = G01init_callback_precalculate3;
//	gt_new_task->precalculate_callback_ref 			= dxdz_callback_precalculate3;
	gt_new_task->unlocked = true; 

	gt_new_task->x_direction = xdir;
	gt_new_task->z_direction = zdir;

	if(prev_task->stepper == true) {
		if(prev_task->z_direction == zdir){
			prev_task->skiprampdownZ = true;
		}
		if(prev_task->x_direction == xdir){
			prev_task->skiprampdownX = true;
		}
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
}






///**
//* @brief  dxdz_callback_precalculate
//* @retval void.
//  */
//void dxdz_callback_precalculate3(state_t* s){
//		s->current_task_ref->unlocked = true;
//}

/**
* @brief  G01init_callback_precalculate
* @retval void.
  */
//void G01init_callback_precalculate3(state_t* s){
//	s->current_task_ref->steps_to_endZ = s->current_task_ref->dz;
//	s->current_task_ref->steps_to_endX = s->current_task_ref->dx;
//	dxdz_callback_precalculate3(s);
////	s->current_task_ref->steps_to_end = 1; 
//}






//uint32_t steps_to_end_shadow =0;
///**
//* @brief  G01init_callback_precalculate
//* @retval void.
//  */
//void G01init_callback_precalculate(state_t* s){
//	if(s->current_task_ref->dz > s->current_task_ref->dx){
//		s->current_task_ref->steps_to_end = s->current_task_ref->dz;
//		s->substep_axis = SUBSTEP_AXIS_X;
//		s->err = -s->current_task_ref->dz >> 1;
//	} else{
//		s->current_task_ref->steps_to_end = s->current_task_ref->dx;
//		s->err = s->current_task_ref->dx >> 1;
//		s->substep_axis = SUBSTEP_AXIS_Z;
//	}
//	steps_to_end_shadow = s->current_task_ref->steps_to_end;

//	if(s->current_task_ref->steps_to_end > 100000)
//		Error_Handler2(3);
//	dxdz_callback_precalculate(s);
////	s->current_task_ref->steps_to_end = 1; 
//}

//int pcc = 0;
//int break5972 = 0;
///**
//* @brief  dxdz_callback_precalculate
//* @retval void.
//  */
//void dxdz_callback_precalculate(state_t* s){
////	pcc++;
////	s->precalculate_end = false;
//	if(s->current_task_ref->steps_to_end == 0)
//		Error_Handler2(5);

//	if(s->current_task_ref->steps_to_end > 100000)
//		Error_Handler2(4);

//	substep_t *sb = substep_cb.top;
//	int e2 = s->err;
//	int32_t delay = -1;
////	if(substep_cb.count2 == 5972) //some bug is here
////		break5972 = 1;
//	if (e2 >= -s->current_task_ref->dx)	{ // step X axis
//		s->err -= s->current_task_ref->dz;
//		if(s->substep_axis == SUBSTEP_AXIS_X){
//			delay = ((1<<subdelay_precision)-1)*(abs(e2))/s->current_task_ref->dx;
//		}
//	}
//	if (e2 <= s->current_task_ref->dz)	{ // step Z axis
//		s->err += s->current_task_ref->dx;
//		if(s->substep_axis == SUBSTEP_AXIS_Z){
//			delay = ((1<<subdelay_precision)-1)*(abs(e2))/s->current_task_ref->dz;
//		}
//	}

//	if(delay >= 0){
//		cb_push_back_empty_ref()->delay = delay;
//	} else {
//		if(substep_cb.count == 0 || sb->skip == 0){
//			cb_push_back_empty(&substep_cb);
//		} else {
//			if(sb->skip == 255){
//				cb_push_back_empty(&substep_cb);
//			}
//		}
//		sb = substep_cb.top;
//		sb->skip++;
//	}
////	s->current_task_ref->steps_to_end--;
//	steps_to_end_shadow--;
//	if(steps_to_end_shadow == 0){
//		s->current_task_ref->unlocked = true;
//	}
//}
/**
* @brief  G00G01init_callback called from load_task
* @retval void.
  */
//void G00G01init_callback(state_t* s){
////	1. set state.function
////	2. set ARR
////	3. set channels
//	s->function = do_fsm_move2;
//	s->syncbase->ARR = fixedpt_toint(s->current_task_ref->F) - 1;

//	s->Q824set = s->current_task_ref->F;
//	s->prescaler = s->syncbase->PSC;
////	TIM3->CCER = 0;
//	s->initial_task_X_pos = s->global_X_pos; // save current position to return here if we decide to repeat last move
//	s->initial_task_Z_pos = s->global_Z_pos;

//	XDIR = s->current_task_ref->x_direction;
//	ZDIR = s->current_task_ref->z_direction;
//	if(s->current_task_ref->dz > s->current_task_ref->dx){
//		s->current_task_ref->steps_to_end = s->current_task_ref->dz;
//		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));
//		s->substep_pulse_on = 1;
//		s->substep_pulse_off = 0;

//		s->substep_axis = SUBSTEP_AXIS_X;
//		s->err = -s->current_task_ref->dz >> 1;
//		MOTOR_Z_AllowPulse();
//		LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_OUTPUT);
//		LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
//	} else{
//		s->current_task_ref->steps_to_end = s->current_task_ref->dx;
//		s->err = s->current_task_ref->dx >> 1;
//		s->substep_axis = SUBSTEP_AXIS_Z;

//		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_Z_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_Z_STEP_Pin_num*4)));
//		s->substep_pulse_on = 1;
//		s->substep_pulse_off = 0;

//		MOTOR_X_AllowPulse(); 
//		LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
//		LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
//	}
//}
