#include "gcode.h"
#include "fsm.h"
#include "g03.h"

int count = 0;
int count_total = 0;
//uint8_t bufx[2000];
//uint8_t bufz[2000];

int subdelay_x=0, subdelay_z=0, subdelay_zfast = 0;
//int x=0, z=0;
int acnt = 0, ecnt=0, exx=0, ezz=0;
float e2e2, edx,edz;
float ff, ffx;
bool brk=0;


void arc_q1_callback(void){
	state_t *s = &state;

	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int64_t e2 = s->arc_err<<1;

	MOTOR_X_AllowPulse();
	s->current_task.x++;
	s->arc_err += s->arc_dx += s->arc_bb; 

	if (e2 > s->arc_dx) {
		return;
	}

	if (e2 > s->arc_dz) { // z step 
		s->substep_mask = MOTOR_Z_CHANNEL;
//		s->substep_axis = SUBSTEP_AXIS_Z;
// quick subdelay detection, depend on given precision, for x16 about ~1us, for x32 about ~2,5us		
		#define subdelay_precision 4
		int32_t delay = s->prescaler * s->syncbase->ARR; // todo delay recalculate move to tim2 or tim4 wherer arr is changing?

		int64_t edz_delta = s->arc_dz >> subdelay_precision;
		int64_t edz_tmp = s->arc_dz;// -(edz_delta<<1);
		while(edz_tmp < e2){
			edz_tmp -= edz_delta;
			delay -= 9600 >> subdelay_precision; // 300 = 9600>>5
		}

		s->current_task.z--;
		s->arc_err += s->arc_dz += s->arc_aa; 

		if(TIM1->CR1 == 0){
			if(delay <= 0) {
				if(delay < 0){
					brk = true;
				}	

				TIM1->CCR1 = 0;
				TIM1->ARR = min_pulse;
				LL_TIM_DisableIT_CC1(TIM1);
				LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_0); 
			} else {
				TIM1->CCR1 = delay+1;
				TIM1->ARR = TIM1->CCR1+min_pulse;
				LL_TIM_EnableIT_CC1(TIM1);
			}

			LL_GPIO_SetOutputPin(MOTOR_Z_ENABLE_GPIO_Port, MOTOR_Z_ENABLE_Pin);
//		LL_TIM_SetAutoReload(TIM1,delay+1);
//			debug1();
			LL_TIM_EnableCounter(TIM1);
		}
	} // z step

//*/

	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
		s->current_task.steps_to_end = 0; // end of arc
		return;
	}

	if(s->current_task.z == 0){ // end of quadrant
		s->current_task.steps_to_end = 0;
	}

	
}




void arc_q1_callback_mod(void){
	state_t *s = &state;

//	if(s->current_task.x == 1709){
//		brk = true;
//	}
	
	
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


	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int64_t e2 = s->arc_err<<1;

/*
	e2e2 = e2;
	edx = s->arc_dx;
	edz = s->arc_dz;
	ecnt = acnt;
	exx = s->current_task.x;
	ezz = s->current_task.z;
	subdelay_z = 0;
	subdelay_x = 0;
	subdelay_zfast = 0;
	acnt++;
*/

	#define subdelay_precision 6
	uint32_t delay = s->prescaler * s->syncbase->ARR; // todo delay recalculate move to tim2 or tim4 wherer arr is changing?

	if (e2 < s->arc_dx) {
		if(s->substep_axis == SUBSTEP_AXIS_X) {
			int64_t edz_delta = s->arc_dx >> subdelay_precision;
			int64_t edz_tmp = s->arc_dx -(edz_delta<<1);
			while(edz_tmp > e2){
				edz_tmp -= edz_delta;
				delay -= 9600 >> subdelay_precision; // 300 = 9600>>5
			}
//			subdelay_x = delay*e2/s->arc_dx;
		}
		MOTOR_X_AllowPulse();
		s->current_task.x++;
		s->arc_err += s->arc_dx += s->arc_bb; 
	} else {
		// x is subaxis now
		s->substep_axis = SUBSTEP_AXIS_X;
	} // x step
	if (e2 > s->arc_dz) { // z step 
		if(s->substep_axis == SUBSTEP_AXIS_Z){
			debug7();
//			subdelay_z = delay*e2/s->arc_dz; // ~44us
//			if(subdelay_z > 9000 || subdelay_z < 100)
//				subdelay_z = 0;
			
// quick subdelay detection, depend on given precision, for x16 about ~1us, for x32 about ~2,5us		
			int64_t edz_delta = s->arc_dz >> subdelay_precision;
			int64_t edz_tmp = s->arc_dz;// -(edz_delta<<1);
			while(edz_tmp < e2){
				edz_tmp -= edz_delta;
				delay -= 9600 >> subdelay_precision; // 300 = 9600>>5
			}
//			else 
//				debug7();

			
			//			subdelay_zfast = delay; // ~44us
//			subdelay_z = delay; // ~44us
			
			//*/
		}
		s->current_task.z--;
		MOTOR_Z_AllowPulse();
		s->arc_err += s->arc_dz += s->arc_aa; 
	} else { // z axis is subaxis now
		s->substep_axis = SUBSTEP_AXIS_Z;
	} // z step

	if(delay >= 0 && delay < 9600) {
		if(delay<160){
			brk = 1;
		}

//	if(subdelay_z > 0 || subdelay_x > 0){
		if(s->substep_axis == SUBSTEP_AXIS_Z){
			MOTOR_Z_BlockPulse();
			s->substep_mask = MOTOR_Z_CHANNEL;
		}
		else {
			MOTOR_X_BlockPulse();
			s->substep_mask = MOTOR_X_CHANNEL;
		}

		LL_TIM_SetAutoReload(TIM1,delay+1);
//			debug1();
		LL_TIM_EnableCounter(TIM1);
//		debug7();
//	}
	}
//*/

	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
		s->current_task.steps_to_end = 0; // end of arc
		return;
	}

	if(s->current_task.z == 0){ // end of quadrant
		s->current_task.steps_to_end = 0;
	}

	
}


void arc_q1_callback_old(void){
	state_t *s = &state;

	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int64_t e2 = s->arc_err<<1;

	if (e2 < s->arc_dx) {
		MOTOR_X_AllowPulse();
		s->current_task.x++;
		s->arc_err += s->arc_dx += s->arc_bb; 
	}// x step
	if (e2 > s->arc_dz) { // z step 
		s->current_task.z--;
		MOTOR_Z_AllowPulse();
		s->arc_err += s->arc_dz += s->arc_aa; 
	}// z step

	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
		s->current_task.steps_to_end = 0; // end of arc
		return;
	}

	if(s->current_task.z == 0){ // end of quadrant
		s->current_task.steps_to_end = 0;
	}

	
}



void arc_q4_callback(void){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int64_t e2 = s->arc_err<<1;
//	do {
//		setPixel(xm, ym-y);
		e2 = s->arc_err<<1;
		if (e2 > s->arc_dx) { 
			s->current_task.x--; 
			MOTOR_X_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
			s->arc_err += s->arc_dx += s->arc_bb; 
		} // x step
		if (e2 < s->arc_dz) { 
			s->current_task.z--;
			MOTOR_Z_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
			s->arc_err += s->arc_dz += s->arc_aa; 
		} // z step
//	} while (x>0);
	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
		s->current_task.steps_to_end = 0; // end of arc
		return;
	}
	if(s->current_task.x == 0){ // end of quadrant
		s->current_task.steps_to_end = 0;
	}
}




uint8_t get_quadrant(int x0, int z0){
	if (x0 >= 0){
		if(z0>=0){
			return 1;
		} else{
			return 4;
		}
	} else {
		if(z0>0){
			return 2;
		} else{
			return 3;
		}
	}
}

int64_t dddz=0;
// on new task loading callback
void G03init_callback(void){
	// set initial delay for pulse
	state.syncbase->ARR = fixedpt_toint(state.current_task.F) - 1;
	state.Q824set = state.current_task.F;
	// set callback iterator for feed speed:
	state.function = do_fsm_move2; 

	//precalculate variables:
	state.arc_aa = (uint64_t)state.current_task.a * state.current_task.a<<1;
	state.arc_bb = (uint64_t)state.current_task.b * state.current_task.b<<1;

	state.current_task.z  = fixedpt_toint2210(state.current_task.z);
	state.current_task.x  = fixedpt_toint2210(state.current_task.x);
	state.current_task.z1 = fixedpt_toint2210(state.current_task.z1);
	state.current_task.x1 = fixedpt_toint2210(state.current_task.x1);

	uint8_t q_from 	= get_quadrant(state.current_task.x, state.current_task.z);
//	uint8_t q_to 		= get_quadrant(state.current_task.x1, state.current_task.z1);

	int cwccw = 1;
	if(cwccw>0){ //cw
		switch(q_from){
			case 1:
				state.arc_dx =  (2*state.current_task.x+1)*state.arc_bb>>1;
				state.arc_dz = -(2*state.current_task.z-1)*state.arc_aa>>1;
				break;
			case 4:
				state.arc_dx = -(2*state.current_task.x-1)*state.arc_bb>>1;
				state.arc_dz = -(2*state.current_task.z-1)*state.arc_aa>>1;
	//			gt_new_task->x_direction = xdir_backward;
				break;
			default: while(1); // impossible case trap
		}
		state.arc_err = state.arc_dx+state.arc_dz;
	} else { //ccw
		//todo check and correct ccw arc generation
		switch(q_from){
			case 2:
				// Q2
				state.arc_dx =  (2*state.current_task.x+1)*state.arc_bb>>1;
				state.arc_dz =  (2*state.current_task.z+1)*state.arc_aa>>1;
				break;
			case 3:
				state.arc_dx = -(2*state.current_task.x-1)*state.arc_bb>>1;
				state.arc_dz =  (2*state.current_task.z+1)*state.arc_aa>>1;
				break;
			default: while(1); // impossible case trap
		}
		state.arc_err = state.arc_dx+state.arc_dz;
	}
	state.substep_axis = -1;
	// use this variable as flag, set it in callback when arc is done:
	state.current_task.steps_to_end = 1; 
	// init and prepare corresponding output channels to generate first step pulse:
	state.current_task.callback_ref();
}

/**
* @brief  G33 parse to tasks. New version with ellipse
* @retval void.
  */
	int64_t ik;
void G03parse(char *line, int8_t cwccw){
/*
                  ^ Z
                  |
                  |
            ******|******
         ***      |      ***
       **         |         **
      *      2    |   1       *
     *            |            *
 -------------------------------------->
     *            |            *       X
      *      3    |   4       *
       **         |         **
         ***      |      ***
            ******|******
                  |
*/
	int x0 = init_gp.X & ~1uL<<10; //save pos from prev gcode
	int z0 = init_gp.Z & ~1uL<<10;
	G_pipeline_t *gref = G_parse(line);

	int x0z = -gref->I; //x0+xdelta;
	int z0z = -gref->K; //z0+zdelta;
	int x1z = gref->X - x0 - gref->I;
	int z1z = gref->Z - z0 - gref->K;


	ik = (int64_t)gref->I*gref->I + (int64_t)gref->K*gref->K;
	ik  = SquareRoot64(ik); // ik - radius of circle in 2210 format.

//	int ii 	= gref->I >> 10, kk = gref->K >> 10; //back from 2210 to steps
//	uint32_t rr = SquareRoot(ii*ii + kk*kk); // find arc radius

/*
Due to the fact that the configuration of the stepper motor for the X and Z axes may not be equal 
in the real world, our circle(in steps per/mm) will not be a circle but an ellipse.
for example, in my config I have 1 mm lead screw 400 steps / mm in Z 
but since my taig lathe with imperial screw on cross feed
I have 1.27 * 200 steps / mm with a decrease in the pulley 61/16 = 200 * 61/16 / 1.27 = 600.3937 steps per mm.
this is about 1.5 more than the Z axis.
so in the case of transferring the physical circle into a stepped ellipse, 
we need to multiply the radius of the X axis (steps by / mm) by 1.5.
*/

// xf is X radius corrected by ~1,50097 factor:
	fixedptu xf, zf;
	xf = fixedpt_toint2210(fixedpt_xmul2210(ik,z_to_x_factor2210));
	zf = fixedpt_toint2210(ik);

	G_task_t *gt_new_task;
	gt_new_task 		= add_empty_task();

//feed:
	if(state.G94G95 == 1){ 	// unit(mm) per rev
		gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
	} else { 											// unit(mm) per min
		gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
	}

	gt_new_task->init_callback_ref = G03init_callback;
	gt_new_task->a = xf;
	gt_new_task->b = zf;

	gt_new_task->x = x0z;
	gt_new_task->z = z0z;

	gt_new_task->x1 = x1z;
	gt_new_task->z1 = z1z;
	
	
//	gt_new_task->aa = xf*xf<<1;
//	gt_new_task->bb = zf*zf<<1;

	uint8_t q_from 	= get_quadrant(x0z, z0z);
	uint8_t q_to 		= get_quadrant(x1z, z1z);
	if(cwccw>0){ //cw
		switch(q_from){
			case 1:
				gt_new_task->callback_ref = arc_q1_callback;
//				gt_new_task->dx =  (2*x0z+1)*gt_new_task->bb>>1;
//				gt_new_task->dz = -(2*z0z-1)*gt_new_task->aa>>1;
	//		x+z-
				break;
			case 4:
				gt_new_task->callback_ref = arc_q4_callback;
//				gt_new_task->dx = -(2*x0z-1)*gt_new_task->bb>>1;
//				gt_new_task->dz = -(2*z0z-1)*gt_new_task->aa>>1;
	//			gt_new_task->x_direction = xdir_backward;
				break;
			default: while(1); // impossible case trap
		}
//		gt_new_task->err = gt_new_task->dx+gt_new_task->dz;

		if(q_to != q_from){ 
			// two quadrants used, add next quadrant as separated task with changed motor direction flag:
			gt_new_task 		= add_empty_task();
		//feed:
			if(state.G94G95 == 1){ 	// unit(mm) per rev
				gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
			} else { 											// unit(mm) per min
				gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
			}
			
			gt_new_task->init_callback_ref = G03init_callback;

//			gt_new_task->aa = xf*xf<<1;
//			gt_new_task->bb = zf*zf<<1;

			if(q_to == 4){
				gt_new_task->a = xf;
				gt_new_task->b = zf;

				gt_new_task->x = xf;
				gt_new_task->z = 0;

				gt_new_task->x1 = x1z;
				gt_new_task->z1 = z1z;
				
				gt_new_task->callback_ref = arc_q4_callback;
//				gt_new_task->dx = -(2*xf-1)*gt_new_task->bb>>1;
//				gt_new_task->dz = -(2*0-1)*gt_new_task->aa>>1;
			} else while(1); // impossible case trap
		}
	} 
	else { //ccw
		//todo check and correct ccw arc generation
		switch(q_from){
			case 2:
				// Q2
				gt_new_task->callback_ref = arc_q2_callback;
//				gt_new_task->dx =  (2*x0z+1)*gt_new_task->bb>>1;
//				gt_new_task->dz =  (2*z0z+1)*gt_new_task->aa>>1;
				break;
			case 3:
//				gt_new_task->dx = -(2*x0z-1)*gt_new_task->bb>>1;
//				gt_new_task->dz =  (2*z0z+1)*gt_new_task->aa>>1;
				gt_new_task->callback_ref = arc_q3_callback;
				break;
			default: while(1); // impossible case trap
		}
//		gt_new_task->err = gt_new_task->dx+gt_new_task->dz;

		if(q_to != q_from){ 
			// two quadrants used, add next quadrant as separated task with changed motor direction flag:
			gt_new_task 		= add_empty_task();
		//feed:
			if(state.G94G95 == 1){ 	// unit(mm) per rev
				gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
			} else { 											// unit(mm) per min
				gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
			}
			
			gt_new_task->init_callback_ref = G03init_callback;

//			gt_new_task->aa = xf*xf<<1;
//			gt_new_task->bb = zf*zf<<1;

			if(q_to == 3){
					gt_new_task->a = xf;
					gt_new_task->b = zf;

					gt_new_task->x = -xf;
					gt_new_task->z = 0;

					gt_new_task->x1 = x1z;
					gt_new_task->z1 = z1z;
//				gt_new_task->dx = -(2*xf-1)*gt_new_task->bb>>1;
//				gt_new_task->dz =  (2*0+1)*gt_new_task->aa>>1;
				gt_new_task->callback_ref = arc_q3_callback;
			} else while(1); // impossible case trap
		}
	}

//	plotOptimizedEllipse(0,0, xf, zf);
}

/*
//int64_t x = 0, z = 0;
void plotOptimizedEllipse(int x0, int z0, int x1, int z1, int a, int b){

	x = -a;
	z = 0; // II. quadrant from bottom left to top right
	int64_t e2, dx; // error increment
	int64_t dz, err; // error of 1.step

	int64_t aa = a*a<<1;
	int64_t bb = b*b<<1;

	
	uint8_t q0 = get_quadrant(x0,z0);

	x = x0;
	z = z0;
	switch(q0){
		case 1:{
		///////////////-----------------Q1	
			//Q1
//			x = 0;
//			z = b;
		// init:		
			dx =  (2*x+1)*bb>>1;
			dz = -(2*z-1)*aa>>1;
			err = dx+dz;

		// callback:
			do {
				e2 = err<<1;
				if (e2 < dx) {
					x++; 
					err += dx += bb; 
				} // x step
				if (e2 > dz) { 
					z--;
					err += dz += aa; 
				} // z step
				if(x == x1 && z == z1)
						return;
			} while (z>0);
			//prepare to next quadrant:
			x = a;
			z = 0;
		}
		case 4:{
			dx = -(2*x-1)*bb>>1;
			dz = -(2*z-1)*aa>>1;
			err = dx+dz;
			do {
				e2 = err<<1;
				if (e2 > dx) { 
					x--; 
					err += dx += bb; 
				} // x step
				if (e2 < dz) { 
					z--;
					err += dz += aa; 
				} // z step
				if(x == x1) {
					if(z == z1){
						return;
					}
				}
			} while (x>0);
			x = 0;
			z = -b;
		}
		case 3:{
			dx = -(2*x-1)*bb>>1;
			dz =  (2*z+1)*aa>>1;
			err = dx+dz;
			do {
				e2 = err<<1;
				if (e2 < dx) { 
					x--; 
					err += dx += bb; 
				} // x step
				if (e2 > dz) { 
					z++;
					err += dz += aa; 
				} // z step
				if(x == x1 && z == z1)
						return;
			} while (z<0);
			x = -a;
			z = 0;
		}
		case 2:{
			// Q2
			dx = (2*x+1)*bb>>1;
			dz = (2*z+1)*aa>>1;
			err = dx+dz;
			do {
				e2 = err<<1;
				if (e2 >= dx) { 
					x++; 
					err += dx += bb; 
				} // x step
				if (e2 <= dz) { 
					z++;
					err += dz += aa; 
				} // z step
				if(x == x1 && z == z1)
						return;
			} while (x <= 0);
		}
	}
}
*/

// on task load callback
/*
void G03init_callback_old(void){
	state_t *s = &state;

	s->syncbase->ARR = fixedpt_toint(s->current_task.F) - 1;
	s->Q824set = s->current_task.F;
	state.function = do_fsm_move2;
	if(s->current_task.callback_ref == arc_dx_callback){
		s->current_task.dz = SquareRoot(s->current_task.rr - s->current_task.dx * s->current_task.dx);
		
	} else {
		s->current_task.dx = SquareRoot(s->current_task.rr - s->current_task.dz * s->current_task.dz);
	}
	s->current_task.callback_ref();
}

// on pulse end callback
void arc_dx_callback(){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	MOTOR_X_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
	int dz = SquareRoot(s->current_task.rr - s->current_task.dx * s->current_task.dx);
	if(dz != s->current_task.dz){
		MOTOR_Z_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
		s->current_task.dz = dz;
	}
	s->current_task.dx += s->current_task.inc_dec;
}

void arc_dz_callback(){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	MOTOR_Z_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
	int dx = SquareRoot(s->current_task.rr - s->current_task.dz * s->current_task.dz);
	if(dx != s->current_task.dx){
	MOTOR_X_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
		s->current_task.dx = dx;
	}
	s->current_task.dz += s->current_task.inc_dec;
}




uint8_t get_octant(int x0z,int z0z, int octant10){
	if(x0z>=0){								//0,1,2,3
		if(x0z < octant10){			//0,3
			if(z0z>0){ 						//oct0
				return 0;
			} else{ 							//oct3
				return 3;
			}
		} else { 								//1,2
			if(z0z>=0){ 						//oct1
				return 1;
			} else{ 							//oct2
				return 2;
			}
		}
	} else { //4,5,6,7
		if(x0z > -octant10){			//4,7
			if(z0z>0){ 						//oct7
				return 7;
			} else{ 							//oct4
				return 4;
			}
		} else { 								//5,6
			if(z0z>0){ 						//oct6
				return 6;
			} else{ 							//oct5
				return 5;
			}
		}
	}
}




void G03parse_old(char *line, int8_t cwccw){ //~130-150us
	int x0 = init_gp.X & ~1uL<<10; //get from prev gcode
	int z0 = init_gp.Z & ~1uL<<10;
//	int pos_count; // 1st octant count by X
	G_pipeline *gref = G_parse(line);
	gref->code = 3;
//#define SQ64
	#ifndef SQ64
	int ii 	= gref->I >> 10, kk = gref->K >> 10; //back from 2210 to steps
	uint32_t rr = SquareRoot(ii*ii + kk*kk); // find arc radius
	rr *= rr;
	int octant = SquareRoot(rr >>1) << 10; // find octant value
	#else
	// more preceise 64bit evaluation of R and octant, 13% slower then 32bit
	uint64_t ik0 = abs(gref->I);
	uint64_t ik = ik0*ik0;
	ik0 = abs(gref->K);
	ik += ik0*ik0;
	ik  = SquareRoot64(ik);
	ik *= ik;
	uint32_t octant = SquareRoot64(ik >>1); // find octant value
	uint32_t rr = ik >> 20;
	#endif

//	int zdelta = -(z0+gref->K); // center delta to shift it to zero 
//	int xdelta = -(x0+gref->I);

	int x0z = -gref->I; //x0+xdelta;
	int z0z = -gref->K; //z0+zdelta;
	int x1z = gref->X - x0 - gref->I;
	int z1z = gref->Z - z0 - gref->K;

	int oct0 = get_octant(x0z, z0z, octant);
	int oct1 = get_octant(x1z, z1z, octant);

//	pos_count = 0;
	G_task *gt_new_task;

//				 ^ Z  
//		 \ 7 | 0 /
//			\  |  /
//			 \ | /       
//			6 \|/ 1
//	 ------0------->X
//			5 /|\ 2
//			 / | \
//			/  |  \      
//		 / 4 | 3 \

	if(oct0 == oct1){
		gt_new_task 		= add_empty_task();
		gt_new_task->init_callback_ref = G03init_callback;
		gt_new_task->rr = rr;
		int current_oct = oct0;

		if(current_oct == 0 || current_oct == 3 || current_oct == 4 || current_oct == 7){
			gt_new_task->steps_to_end = abs(x1z - x0z);
			gt_new_task->dx = x0z;
			gt_new_task->callback_ref = arc_dx_callback;
		} else{
			gt_new_task->steps_to_end = abs(z0z - z1z);
			gt_new_task->dz = z0z;//pos_count;
			gt_new_task->callback_ref = arc_dz_callback;
		}

		if(cwccw>0){ //cw
			if(current_oct == 0 || current_oct == 2){ // 4,6
				gt_new_task->inc_dec = 1;
			} else { // 1,3
				gt_new_task->inc_dec = -1;
			}
		} else { //ccw
			if(current_oct == 7 || current_oct == 5){
				gt_new_task->inc_dec = 1;
			} else {
				gt_new_task->inc_dec = -1;
			}
		}
		
		// X direction:
		if(current_oct == 0 || current_oct == 1 || current_oct == 4 || current_oct == 5){
				gt_new_task->x_direction = xdir_forward;
		} else{
				gt_new_task->x_direction = xdir_backward;
		}
		gt_new_task->z_direction = zdir_forward;
		
		//feed:
		if(state.G94G95 == 1){ 	// unit(mm) per rev
			gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
		} else { 											// unit(mm) per min
			gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
		}
	} else {
		uint8_t current_oct = oct0;
		while(current_oct != oct1+cwccw){
			gt_new_task 		= add_empty_task();
			gt_new_task->init_callback_ref = G03init_callback;
			gt_new_task->rr = rr;
// X direction
			if(current_oct == 0 || current_oct == 1 || current_oct == 4 || current_oct == 5){
					gt_new_task->x_direction = xdir_forward;
			} else{
					gt_new_task->x_direction = xdir_backward;
			}
			gt_new_task->z_direction = zdir_forward;

			if(current_oct == oct0){
				switch(current_oct){
					case 0: case 7:
						gt_new_task->steps_to_end = abs(octant - x0z);
						gt_new_task->dx = x0z;//pos_count;
						gt_new_task->callback_ref = arc_dx_callback;
						break;
					case 1: case 6:
						gt_new_task->steps_to_end = abs(z0z);
						gt_new_task->dz = abs(z0z);//pos_count;
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 2: case 5:
						gt_new_task->steps_to_end = abs(octant - z0z);
						gt_new_task->dz = abs(z0z);
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 3: case 4:
						gt_new_task->steps_to_end = abs(x0z);
						gt_new_task->dx = abs(x0z); //pos_count;
						gt_new_task->callback_ref = arc_dx_callback;
						break;
				}
			} else if(current_oct == oct1){
				switch(current_oct){
					case 0: case 7:
						while(1);
					case 1: case 6:
						gt_new_task->steps_to_end = abs(octant - z1z);
						gt_new_task->dz = octant;
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 2: case 5:
						gt_new_task->steps_to_end = abs(z1z);
						gt_new_task->dz = abs(z1z);
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 3: case 4:
						gt_new_task->steps_to_end = abs(octant - x1z);
						gt_new_task->dx = octant;
						gt_new_task->callback_ref = arc_dx_callback;
						break;
				}
			} else { //middle octant
			if(cwccw>0){ //cw
				switch(current_oct){
					case 1:
						gt_new_task->dz = octant; break;
					case 2:
						gt_new_task->dz = 0; break;
					case 3:
						gt_new_task->dx = octant; break;
				}
			} else {
				switch(current_oct){
					case 6:
						gt_new_task->dz = octant; break;
					case 5:
						gt_new_task->dz = 0; break;
					case 4:
						gt_new_task->dx = octant; break;
				}
			}
				//				pos_count =octant;
				gt_new_task->steps_to_end = octant;
				if(current_oct == 0 || current_oct == 3 || current_oct == 4 || current_oct == 7){
//					gt_new_task->dx = octant;
					gt_new_task->callback_ref = arc_dx_callback;
				} else{
//					gt_new_task->dz = octant;
					gt_new_task->callback_ref = arc_dz_callback;
				}
			}
//inc_dec
			if(cwccw>0){ //cw
				if(current_oct == 0 || current_oct == 2){
					gt_new_task->inc_dec = 1;
				} else {
					gt_new_task->inc_dec = -1;
				}
			} else { //ccw
				if(current_oct == 7 || current_oct == 5){
					gt_new_task->inc_dec = 1;
				} else {
					gt_new_task->inc_dec = -1;
				}
			}
			//feed:
			if(state.G94G95 == 1){ 	// unit(mm) per rev
				gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
			} else { 											// unit(mm) per min
				gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
			}
			gt_new_task->dz = fixedpt_toint2210(gt_new_task->dz);
			gt_new_task->dx = fixedpt_toint2210(gt_new_task->dx);
			gt_new_task->steps_to_end = fixedpt_toint2210(gt_new_task->steps_to_end);
			current_oct +=cwccw;
			if(current_oct == 255)
				current_oct = 7;
			else if(current_oct == 8)
				current_oct = 0;
		}
	}					

}
*/

/*
  общее число шагов для дуги равно сумме шагов по осям в соответствии с текущей октантой
	для октант 0,3,4 и 7 основной осью является ось Х, сдвиг по оси Z вычисляется как sqrt(R*R-dx*dx)
	для октант 1,2,5 и 6 основной осью является ось Z, сдвиг по оси X вычисляется как sqrt(R*R-dz*dz)
	Если дуга в нескольких октантах, например начинается в 0-1-2,
	то число шагов равно:
		участок в 0м октанте:
		octant-x1
		участок в 1м октанте:
		octant
		участок в 2м октанте:
		abs(z2)
		итого (octant-x1 + octant + abs(x2))
	при переходе границы октанта меняется ось по которой шагаем, 
	в т.ч. при переходе границ осей меняем направления шагания для моторов(плюс корректировка backlash если надо)
	
	for cross octane arc:

				 ^ Z  
		 \ 7 | 0 /
			\  |  /
			 \ | /       
			6 \|/ 1
	 ------0------->X
			5 /|\ 2
			 / | \
			/  |  \      
		 / 4 | 3 \



	
*/	




/*
void plotEllipse(int x0, int z0, int a, int b){
	x = -a;
	z = 0;
	int64_t ex, ey, e2, err;

	e2 = (int64_t)b*b;


	// II quadrant
	err = (x0+1)*(x0+1)*b*b + (z0+1)*(z0+1)*a*a - a*a*b*b;
	// -373 403 -321632000	
	do {
		e2 = 2*err;
		ex = (x*2+1)*(int64_t)b*b;
		if (e2 >= ex){ //e_xy+e_x > 0
			err += (++x*2+1)*(int64_t)b*b;
//			BIT_BAND_SRAM(&bufx,count) = 1;
		}
		ey = (z*2+1)*(int64_t)a*a;
		if (e2 <= ey){ // e_xy+e_y < 0 
			err += (++z*2+1)*(int64_t)a*a;
//			BIT_BAND_SRAM(&bufz,count) = 1;
		}
		count++;
	} while (x <= 0);
	count_total = --count;
	x0 = 0;
	z0 = b;
	x = x0;
	z = z0;


	// I quadrant:
	err = (x0+1)*(x0+1)*b*b + (z0-1)*(z0-1)*a*a - a*a*b*b;
	do {
		e2 = 2*err;
		ex = (x*2+1)*(int64_t)b*b;
		if (e2 < ex){ //if (e2 - ex < 0){ // e_xy+e_x < 0 
			err += (++x*2+1)*(int64_t)b*b;
		}
		ey = (z*2-1)*(int64_t)a*a;
		if (e2 > -ey){ //if (e2 + ey > 0){ //e_xy+e_y > 0	
			err -= (--z*2-1)*(int64_t)a*a;
		}
	} while (z >0);


	// IV quadrant:
	x0 = a;
	z0 = 0;
	x = x0;
	z = z0;

	err = (x0-1)*(x0-1)*b*b + (z0-1)*(z0-1)*a*a - a*a*b*b;
	do {
		e2 = 2*err;
		ex = (x*2-1)*(int64_t)b*b;
		if (e2+ex>0){ //if (e2 - ex < 0){ // e_xy+e_x < 0 
			err -= (--x*2-1)*(int64_t)b*b;
		}
		ey = (z*2-1)*(int64_t)a*a;
		if (e2+ey<0){ //if (e2 + ey > 0){ //e_xy+e_y > 0	
			err -= (--z*2-1)*(int64_t)a*a;
		}
	} while (x>=0);

	
	// III quadrant:
	x0 = 0;
	z0 = -b;
	x = x0;
	z = z0;

	err = (x0-1)*(x0-1)*b*b + (z0+1)*(z0+1)*a*a - a*a*b*b;
	do {
		e2 = 2*err;
		ex = (x*2-1)*(int64_t)b*b;
		if (e2+ex<0){ //if (e2 - ex < 0){ // e_xy+e_x < 0 
			err -= (--x*2-1)*(int64_t)b*b;
		}
		ey = (z*2+1)*(int64_t)a*a;
		if (e2-ey>0){ //if (e2 + ey > 0){ //e_xy+e_y > 0	
			err += (++z*2+1)*(int64_t)a*a;
		}
	} while (z<=0);
}*/







void arc_q2_callback(void){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int64_t e2 = s->arc_err<<1;

	if (e2 >= s->arc_dx) { 
		MOTOR_X_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
//			x++; 
		s->arc_err += s->arc_dx += s->arc_bb; 
	} // x step
	if (e2 <= s->arc_dz) { 
//			z++;
		MOTOR_Z_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
		s->arc_err += s->arc_dz += s->arc_aa; 
	} // z step
//	} while (x <= 0);
}



void arc_q3_callback(void){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int64_t e2 = s->arc_err<<1;

	if (e2 >= s->arc_dx) { 
		MOTOR_X_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
		s->current_task.x++; 
		s->arc_err += s->arc_dx += s->arc_bb; 
	} // x step
	if (e2 <= s->arc_dz) { 
		s->current_task.z++;
		MOTOR_Z_AllowPulse(); //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
		s->arc_err += s->arc_dz += s->arc_aa; 
	} // z step

	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
		s->current_task.steps_to_end = 0; // end of arc
		return;
	}
	if(s->current_task.x == 0){ // end of quadrant
		while(1); // trap
		s->current_task.steps_to_end = 0;
	}
}
