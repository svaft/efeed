#include "gcode.h"
#include "fsm.h"
#include "g03.h"

#include "math.h"
int count = 0;
int count_total = 0;
//uint8_t bufx[2000];
//uint8_t bufz[2000];

int subdelay_x=0, subdelay_z=0, subdelay_zfast = 0;
//int x=0, z=0;
int acnt = 0, ecnt=0, exx=0, ezz=0, ldt = 0;
float e2e2, edx,edz;
float ff, ffx;
bool brk=0;

//substep_t substep[substep_size];

uint8_t prev_delay = 0;
int32_t prev_delay_sma = 0;
uint32_t avgSum = 0, directSum = 0;

#define arrNumbers_len 8
int arrNumbers[arrNumbers_len] = {0};
bool start_smooth_flag = false;
uint8_t pos = 0;
int32_t moving_sum = 0, ms2 = 0;

#define smooth_direction_front 0
#define smooth_direction_back  1
substep_t *sb_head = 0; 

/**
* @brief  find substep delay value for arc point
* @retval void.
  */
void substep_recalc(int64_t e2, int64_t dzdx, char smooth_direction){
	substep_t *sb = substep_cb.top;

	int16_t delay = (1<<subdelay_precision); //s->prescaler * (s->syncbase->ARR+1); // todo delay recalculate move to tim2 or tim4 wherer arr is changing?

///		e2 = labs(e2);
	bool step_back = false;
	if(e2>0){
		step_back = true;
		e2 = -e2;
	}
	int16_t delay_delta = delay >> subdelay_precision;
	int64_t edz_delta = dzdx >> subdelay_precision;
	// do binary search for delta:
	int64_t r = dzdx - (edz_delta <<(subdelay_precision-1)); // start from half
	delay -= delay_delta<<(subdelay_precision-1);

	for(int a =subdelay_precision-2;a>=0; a--){
		if(r > e2){
			r 		+=	edz_delta		<< a;
			delay	+=	delay_delta	<< a;
		}	else {
			r 		-=	edz_delta		<< a;
			delay	-=	delay_delta	<< a;
		}
	}
	// subdelay found, let's smooth it:
	// find delay from previous calculated step to current:
	int last_delay_total;
	if(step_back){
		last_delay_total = 255 - prev_delay + (sb->skip << subdelay_precision) - delay;
		prev_delay = 256 - delay;
	}
	else {
		last_delay_total = 255 - prev_delay + delay + (sb->skip << subdelay_precision);
		prev_delay = delay;
	}
	ldt = 0;
	ldt = last_delay_total;

//	substep_sma_ref_t *sma_head_ref = 0;
		substep_sma_ref_t sb_sma;
		substep_sma_ref_t sb_sma_head;
	// calculate moving sum:
	moving_sum += last_delay_total;//for step back last delay total calculation wrong
	if(sma_cb.count == arrNumbers_len) {
		cb_pop_front(&sma_cb, &sb_sma_head);
//		sma_head_ref = (substep_sma_ref_t *)cb_pop_front_ref(&sma_cb);
		moving_sum -= sb_sma_head.delay; // *(uint32_t *)cb_pop_front_ref(&sma_cb);
//		cb_pop_front_ref(&sma_substep_cb);
//		cb_push_back(&sma_substep_cb,sb);
	}
	sb_sma.delay = last_delay_total;
	sb_sma.ref = sb;
	sb_sma.count = substep_cb.count;
	cb_push_back(&sma_cb,&sb_sma);

  ms2 = moving_sum;
	int sma = moving_sum >> 3; /// arrNumbers_len;

	// dumb condition when to start applying smooth steps:
	if(sma <700)
//		start_smooth_flag = false;
		start_smooth_flag = true;

	if(smooth_direction == smooth_direction_back) { //shift smoothed step to front
		if(start_smooth_flag == true) {
			// calculate moving average:
			prev_delay_sma = (sma - (255 - prev_delay_sma));
			if(prev_delay_sma < 0)
				Error_Handler();
			int skip_sma_steps = prev_delay_sma >> subdelay_precision;
			if(step_back){
				if(sb->delay > 0){
				// ellipse equator?
					subdelay_x++;
				}
				step_back = false;
			}
			sb = substep_cb.top;
			if(sb->skip > 0) {
				sb->skip = skip_sma_steps;
				if(skip_sma_steps == 0)
					cb_step_back(&substep_cb);
			}
			else if(skip_sma_steps > 0){
				cb_push_back_empty_ref()->skip = skip_sma_steps;
			}
			prev_delay_sma = prev_delay_sma - (skip_sma_steps << subdelay_precision);
			delay = prev_delay_sma;
		} else {
			prev_delay_sma = prev_delay;
		}

		if(!step_back){
			cb_push_back_empty_ref()->delay = delay;
		} else { // delay in negative, so we need to step back and modify previous step
			delay = (1 << subdelay_precision) - delay;
			sb = substep_cb.top;
			if(sb->skip > 0){
				sb->skip--; // 1 step back
				if(sb->skip > 0){ // if we stil have some to skip add new substep, if not - just modify current substep
					sb = cb_push_back_empty_ref();
				}
				sb->delay = delay;
				cb_push_back_empty_ref()->skip = 1;
			} else {
				// in this case we need to do two substeps in one main step  :( 
	//			Error_Handler();
				cb_push_back_empty_ref()->delay = delay;
				cb_push_back_empty_ref()->skip = 1;
			}
		}
	} else {
		// for back smooth strategy at first we add delay at current time position, then recalculate previous steps to avoid extremal stepper accel\deccelerations 
		if(!step_back){
			cb_push_back_empty_ref()->delay = delay;
		} else { // delay in negative, so we need to step back and modify previous step
			delay = (1 << subdelay_precision) - delay;
			sb = substep_cb.top;
			if(sb->skip > 0){
				sb->skip--; // 1 step back
				if(sb->skip > 0){ // if we stil have some to skip add new substep, if not - just modify current substep
					sb = cb_push_back_empty_ref();
				}
				sb->delay = delay;
				cb_push_back_empty_ref()->skip = 1;
			} else {
				// in this case we need to do two substeps in one main step  :( 
	//			Error_Handler();
				cb_push_back_empty_ref()->delay = delay;
				cb_push_back_empty_ref()->skip = 1;
			}
		}
		if (start_smooth_flag == true) {
//			float fsma = sma * 0.7f; //98 ticks
			uint32_t isma = ((sma<<10)*800)>>20;//6 ticks
			if(last_delay_total < isma){
				//перематываем назад к указателю, записанному в sma_head.ref, и вставляем 8 значений sma итоговой суммой moveing_sum
			substep_t *sb2 = sb;
			substep_cb.top = (char *)sb_sma_head.ref;// + substep_cb.sz;
			sb = substep_cb.top;

			substep_cb.count2 -= (substep_cb.count - sb_sma_head.count);
			substep_cb.count = sb_sma_head.count;
			substep_cb.head = (char *)substep_cb.top + substep_cb.sz;

			int a = subdelay_precision, b, delta = moving_sum - sma*subdelay_precision;
				
			while(a > 0){
				b = sma - (255 - sb->delay);
				int c = b >> subdelay_precision;
				if (c > 0){
					cb_push_back_empty_ref()->skip = c;
					b -= (c<<subdelay_precision);
				}
				if (b <= 0)
					b = 1;
				cb_push_back_empty_ref()->delay = b;
				a--;
				sb = substep_cb.top;
				//refresh SMA buffer with new values:
				cb_pop_front(&sma_cb, &sb_sma_head);
				sb_sma.delay = sma;
				sb_sma.ref = sb;
				sb_sma.count = substep_cb.count;
				cb_push_back(&sma_cb,&sb_sma);
			}
			sb = substep_cb.top;
			sb->delay += delta;
			prev_delay = sb->delay;
			if (step_back)
				cb_push_back_empty_ref()->skip = 1;
			}
		}
	}
	
}


bool equator_init = false;
/**
* @brief  precalculate arc 1st quadrant with substeps
* @retval void.
  */
void arc_q1_callback_precalculate(state_t* s){
	int64_t e2 = s->arc_err<<1;
	substep_t *sb = substep_cb.top;
	if(s->current_task.x == 7089){
	brk =1;
	}
	if(s->arc_equator > 0){
// main axis is X, subaxis - Z	
		s->arc_equator--;		
		s->current_task.x++;
		s->arc_err += s->arc_dx += s->arc_bb;
	} else {
		if(equator_init == false){
			equator_init  = true;
			cb_push_back_empty_ref()->skip = 1;
		}
// main axis is Z, subaxis - X	
		s->arc_equator=0;
		if (e2 < s->arc_dx) {
			substep_recalc(-e2, -s->arc_dx, smooth_direction_front);
			s->current_task.x++;
			s->arc_err += s->arc_dx += s->arc_bb;
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
	}
	s->current_task.steps_to_end++;


//	if (e2 > s->arc_dx) {
//		substep_job_t *sbjob = substep_job_cb.top;
//		if (sbjob && sbjob->substep_axis == SUBSTEP_AXIS_X) {
//		}

//		Error_Handler(); // trap
//	}

	if (e2 > s->arc_dz) { // z step 
		if(s->arc_equator>0)
			substep_recalc(e2, s->arc_dz, smooth_direction_front);
		s->current_task.z--;
		s->arc_err += s->arc_dz += s->arc_aa;
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
//		last_skip++;
	} // z step

	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
		s->precalculating_task_ref->unlocked = true;
//		s->precalculate_end = true;
//		s->current_task.steps_to_end = 0; // end of arc
		return;
	}

	if(s->current_task.z == 0){ // end of quadrant
		s->precalculating_task_ref->unlocked = true;
//		s->precalculate_end = true;
//		s->current_task.steps_to_end = 0;
	}
}

void arc_q1_callback_light(state_t* s){
	if(s->arc_equator > 0){
		s->arc_equator--;
	} else if (s->arc_equator == 0) {
		s->arc_equator = -1; 
		LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
//		s->Q824set = s->current_task.F;
		MOTOR_X_Enable(); // for debug to see equator
//		MOTOR_X_BlockPulse(); 
		s->substep_axis = SUBSTEP_AXIS_X;
		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));
		s->substep_pulse_on = 1;
		s->substep_pulse_off = 0;

		MOTOR_Z_AllowPulse(); 
		LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
	}
	if(s->arc_total_steps-- == 0)
		s->current_task.steps_to_end = 0;
}


//int64_t dddz=0;
// on new task loading callback
void G03init_callback(state_t* s){
	// set initial delay for pulse

	s->arc_equator = fixedpt_xmul2210( s->current_task.b, z_to_x_ellipse_equator2210); //todo move to task
	s->arc_total_steps = fixedpt_xmul2210( s->current_task.b, ellipse_total_steps2210);
	
	s->Q824set = s->current_task.F;
//	s->Q824set = fixedpt_toint(f) - 1; //s->current_task.F;
	// set callback iterator for feed speed:
	s->function = do_fsm_move2; 
	s->prescaler = s->syncbase->PSC;
	s->syncbase->ARR = fixedpt_toint(s->current_task.F) - 1;
	
	//precalculate variables:
//	s->arc_aa = (uint64_t)s->current_task.a * s->current_task.a<<1;
//	s->arc_bb = (uint64_t)s->current_task.b * s->current_task.b<<1;

	uint8_t q_from 	= get_quadrant(s->current_task.x, s->current_task.z);
//	uint8_t q_to 		= get_quadrant(s->current_task.x1, s->current_task.z1);

	int cwccw = 1;
	if(cwccw>0){ //cw
		switch(q_from){
			case 1:
//				s->arc_dx =  (2*s->current_task.x+1)*s->arc_bb>>1;
//				s->arc_dz = -(2*s->current_task.z-1)*s->arc_aa>>1;
				
				if(s->current_task.x < s->arc_equator){
					s->arc_equator -= s->current_task.x;

					s->substep_axis = SUBSTEP_AXIS_Z;
					s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_Z_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_Z_STEP_Pin_num*4)));
					s->substep_pulse_on = 0;
					s->substep_pulse_off = 1;
					
					MOTOR_X_AllowPulse(); 
					LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
					LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
					LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
				} else { //arc starts beyond equator, set it to zero
					s->arc_equator = 0;
				}

				s->arc_total_steps -= (s->current_task.x + s->current_task.z1);
				break;
			case 4:
//				s->arc_dx = -(2*s->current_task.x-1)*s->arc_bb>>1;
//				s->arc_dz = -(2*s->current_task.z-1)*s->arc_aa>>1;
	//			gt_new_task->x_direction = xdir_backward;
				break;
			default: Error_Handler(); // impossible case trap
		}
		s->arc_err = s->arc_dx+s->arc_dz;
	} else { //ccw
		//todo check and correct ccw arc generation
		switch(q_from){
			case 2:
				// Q2
//				s->arc_dx =  (2*s->current_task.x+1)*s->arc_bb>>1;
//				s->arc_dz =  (2*s->current_task.z+1)*s->arc_aa>>1;
				break;
			case 3:
//				s->arc_dx = -(2*s->current_task.x-1)*s->arc_bb>>1;
//				s->arc_dz =  (2*s->current_task.z+1)*s->arc_aa>>1;
				break;
			default: Error_Handler(); // impossible case trap
		}
//		s->arc_err = s->arc_dx+s->arc_dz;
	}
//	s->substep_axis = -1;
	// use this variable as flag, set it in callback when arc is done:
	s->current_task.steps_to_end = 1; 
	// init and prepare corresponding output channels to generate first step pulse:
	s->current_task.callback_ref(s);
}



void G03init_callback_precalculate(state_t* s){
//	s->precalculate_end = false;
	//precalculate variables:
//	TIM1->ARR = 0xffff;
//	TIM1->PSC = 0;
//	TIM1->SR = 0;
//	LL_TIM_EnableCounter(TIM1);
	
	s->arc_equator = fixedpt_xmul2210( s->current_task.b, z_to_x_ellipse_equator2210);
	s->arc_total_steps = fixedpt_xmul2210( s->current_task.b, ellipse_total_steps2210);

	// recalculate feed rate from arc length:

//	uint32_t ff = (9000 * (s->current_task.len>>10) / s->arc_total_steps)<<20;
//	uint32_t ff = 1800000 / s->arc_total_steps;
//	ff = ff * s->current_task.len / 200; // 200 = z motor 400steps/2mm
//	s->current_task.F = fixedpt_xdiv2210(ff, s->current_task.F) << 14; // translate to 8.24 format used for delays
//150994944000.0f
//167713215111.444f = 30000hz*60sec/(400steps/2mmrev)*2^24*len_2_arc, see defines.ods
	float f1 = 167713215111.444f * s->current_task.len_f / s->arc_total_steps / s->current_task.F;
//	s->current_task.F = f1;
	s->precalculating_task_ref->F = f1;
//	s->current_task.F = SquareRoot64(s->current_task.F);
//	f1 = sqrtf(f1); //380;

	s->arc_aa = (uint64_t)s->current_task.a * s->current_task.a<<1;
	s->arc_bb = (uint64_t)s->current_task.b * s->current_task.b<<1;

	s->current_task.z  = fixedpt_toint2210(s->current_task.z);
	s->current_task.x  = fixedpt_toint2210(s->current_task.x);
	s->current_task.z1 = fixedpt_toint2210(s->current_task.z1);
	s->current_task.x1 = fixedpt_toint2210(s->current_task.x1);

	uint8_t q_from 	= get_quadrant(s->current_task.x, s->current_task.z);
//	uint8_t q_to 		= get_quadrant(s->current_task.x1, s->current_task.z1);

	int cwccw = 1;
	if(cwccw>0){ //cw
		switch(q_from){
			case 1:
				s->arc_dx =  (2*s->current_task.x+1)*s->arc_bb>>1;
				s->arc_dz = -(2*s->current_task.z-1)*s->arc_aa>>1;
			
				if(s->current_task.x < s->arc_equator){
					s->arc_equator -= s->current_task.x;
				} else { //arc starts beyond equator, set it to zero
					s->arc_equator = 0;
				}
				s->arc_total_steps -= (s->current_task.x + s->current_task.z1);
				break;
			case 4:
				s->arc_dx = -(2*s->current_task.x-1)*s->arc_bb>>1;
				s->arc_dz = -(2*s->current_task.z-1)*s->arc_aa>>1;
	//			gt_new_task->x_direction = xdir_backward;
				break;
			default: Error_Handler(); // impossible case trap
		}
		s->arc_err = s->arc_dx+s->arc_dz;
	} else { //ccw
		//todo check and correct ccw arc generation
		switch(q_from){
			case 2:
				// Q2
				s->arc_dx =  (2*s->current_task.x+1)*s->arc_bb>>1;
				s->arc_dz =  (2*s->current_task.z+1)*s->arc_aa>>1;
				break;
			case 3:
				s->arc_dx = -(2*s->current_task.x-1)*s->arc_bb>>1;
				s->arc_dz =  (2*s->current_task.z+1)*s->arc_aa>>1;
				break;
			default: Error_Handler(); // impossible case trap
		}
		s->arc_err = s->arc_dx+s->arc_dz;
	}
	s->substep_axis = -1;
	// use this variable as flag, set it in callback when arc is done:
	s->current_task.steps_to_end = 1; 
	// init and prepare corresponding output channels to generate first step pulse:
	s->current_task.precalculate_callback_ref(s);
}


/**
* @brief  G33 parse to tasks. New version with ellipse
* @retval void.
  */
void G03parse(char *line, int8_t cwccw){
/*
                           | -X
                           |
                     ******|******
                  ***      |      ***
                 **        |        **
------          *          |          *
     |         *     q3    |  q2       *
lathe|--       *           |           *
chuck|  |     *            |            *
     |  |---  *            |            *
     |  |     *            |            *
     |  | -----------------0------------x-------> spindle axis(Z)
     |  |     *            |    b       * x0z0      
     |  |---  *            |            *       
     |  |     *            |            *       
     |--       *     q4   a|  q1       *
     |         *           |           *
------          *          |          *
                 **        |        **<--equator
                  ***      |      ***___
                     ******x******   \_lathe tool
                           |x1z1
								           ˅ +X

-------------------------------->        
                          |              
                    *******              
                 ***                     
                **                       
               *                         
              *    3                     
              *                          
             *                           
-------------*                           

-------------------------------->
                          |   
                          |   
---------------           |
               ***        |
                 **       |
                   *      |
                2   *     |
                    *     |
                     *    |
                     *----*



*/
	state_t *s = &state_precalc;
	int x0  = init_gp.X  & ~1uL<<(FIXEDPT_FBITS2210-1); //save pos from prev gcode
	int x0r = init_gp.Xr & ~1uL<<(FIXEDPT_FBITS2210-1); //save pos from prev gcode
	int z0  = init_gp.Z  & ~1uL<<(FIXEDPT_FBITS2210-1);
	G_pipeline_t *gref = G_parse(line);

	uint64_t iklong = (int64_t)gref->I*gref->I + (int64_t)gref->K*gref->K;
	uint32_t ik = sqrtf((float)iklong);// ik - radius of circle in 2210 format.
//	ik  = SquareRoot64(ik); // ik - radius of circle in 2210 format.


	int x0z = -gref->I; //x0+xdelta;
	int z0z = -gref->K; //z0+zdelta;
	int x1z = gref->X - x0 - gref->I;
	int z1z = gref->Z - z0 - gref->K;
	int x1zr = gref->Xr - x0r - gref->I;


/*
	int x1zr = gref->Xr - (init_gp.Xr & ~1uL<<(FIXEDPT_FBITS2210-1)) - gref->I;

	int ikz0 = (ik + gref->K)/1024;
	int ikx0 = -gref->I/1024;
	

	int x1 = gref->Xr - x0r - gref->I;

	int ikx1 = (gref->Xr - x0r + ikx0)/1024;
	int ikz1 = (gref->Z -  z0  - ikz0)/1024;

	int lenint = SquareRoot((ikx1 - ikx0)*(ikx1 - ikx0) + (ikz1 - ikz0)*(ikz1 - ikz0));
*/
//	int ii 	= gref->I >> 10, kk = gref->K >> 10; //back from 2210 to steps
//	uint32_t rr = SquareRoot(ii*ii + kk*kk); // find arc radius

/*
Due to the fact that the configuration of the stepper motor for the X and Z axes may not be equal 
in the real world so the circle in mm(or inch) will not be a circle in steps per/mm but an ellipse.
for example, in my config if I have 1 mm lead screw 400 steps / mm in Z 
and my taig lathe with imperial screw on cross feed
I have 1.27 * 200 steps / mm with a decrease in the pulley pair 61/16 = 200 * 61/16 / 1.27 = 600.3937 steps per mm.
this is about 1.5 more than the Z axis.
so in the case of transferring the physical circle into a stepped ellipse, 
we need to multiply the radius of the X axis (steps by / mm) by 1.5.
for the x stepper config of 400step/rev and z screw pitch of 2mm z_to_x = 6. see defines.ods 
*/

// xf is X radius corrected by z_to_x_factor2210 factor:
	fixedptu xf, zf;
	xf = fixedpt_toint2210(fixedpt_xmul2210(ik,z_to_x_factor2210));
	zf = fixedpt_toint2210(ik);

	G_task_t *gt_new_task;
	gt_new_task = add_empty_task();
	gt_new_task->stepper = true;
	
/*	// вычисляем расстояние между точками x0z0-x1z1 на дуге. 
умножив расстояние на len_to_arc_factor2210 получим длину дуги.
по длине дуги сможем скорректировать скорость движения(F)
*/	
	uint64_t il = (int64_t)(x1zr-x0z)*(x1zr-x0z)+(int64_t)(z0z-z1z)*(z0z-z1z);
//	gt_new_task->len = fixedpt_xmul2210(len_to_arc_factor2210,sqrtf(il));
//	gt_new_task->len_f = 1.110720735f * sqrtf(il); //fixedpt_xmul2210(len_to_arc_factor2210,sqrtf(il));
	gt_new_task->len_f = sqrtf(il); //fixedpt_xmul2210(len_to_arc_factor2210,sqrtf(il));
//	gt_new_task->len = fixedpt_xmul2210(len_to_arc_factor2210,SquareRoot64(il));
//	uint32_t len = 0;


//feed:
	if(s->G94G95 == G95code){ 	// unit(mm) per rev
		gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
	} else { 											// unit(mm) per min
		gt_new_task->F = gref->F;//str_f824mm_min_to_delay824(gref->F);
	}


	gt_new_task->precalculate_init_callback_ref =  G03init_callback_precalculate;
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
				gt_new_task->callback_ref = arc_q1_callback_light;
				gt_new_task->precalculate_callback_ref = arc_q1_callback_precalculate;

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
			default: Error_Handler(); // impossible case trap
		}
//		gt_new_task->err = gt_new_task->dx+gt_new_task->dz;

		if(q_to != q_from){ 
			// two quadrants used, add next quadrant as separated task with changed motor direction flag:
			gt_new_task 		= add_empty_task();
			gt_new_task->stepper = true;
		//feed:
			if(s->G94G95 == 1){ 	// unit(mm) per rev
				gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
			} else { 											// unit(mm) per min
				gt_new_task->F = gref->F; //str_f824mm_min_to_delay824(gref->F);
			}
			
			gt_new_task->init_callback_ref = G03init_callback;
			gt_new_task->precalculate_init_callback_ref =  G03init_callback_precalculate;

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
			} else Error_Handler(); // impossible case trap
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
			default: Error_Handler(); // impossible case trap
		}
//		gt_new_task->err = gt_new_task->dx+gt_new_task->dz;

		if(q_to != q_from){ 
			// two quadrants used, add next quadrant as separated task with changed motor direction flag:
			gt_new_task 		= add_empty_task();
			gt_new_task->stepper = true;
		//feed:
			if(s->G94G95 == 1){ 	// unit(mm) per rev
				gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
			} else { 											// unit(mm) per min
				gt_new_task->F = gref->F; //str_f824mm_min_to_delay824(gref->F);
			}
			
			gt_new_task->init_callback_ref = G03init_callback;
			gt_new_task->precalculate_init_callback_ref =  G03init_callback_precalculate;

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
			} else Error_Handler(); // impossible case trap
		}
	}

//	plotOptimizedEllipse(0,0, xf, zf);
}





/*
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


*/




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
	s->function = do_fsm_move2;
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
		if(s->G94G95 == 1){ 	// unit(mm) per rev
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
						Error_Handler();
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
			if(s->G94G95 == 1){ 	// unit(mm) per rev
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







void arc_q2_callback(state_t* s){
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
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



void arc_q3_callback(state_t* s){
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
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
		Error_Handler(); // trap
		s->current_task.steps_to_end = 0;
	}
}


void arc_q4_callback(state_t* s){
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
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


/*
void arc_q1_callback(state_t* s){
//	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	int64_t e2 = s->arc_err<<1;
	
	if (e2 < s->arc_dx) {
		s->arc_err += s->arc_dx += s->arc_bb; 
		s->current_task.x++;
	} else {
		// x is subaxis now
//	if (e2 > s->arc_dx) {
//		return;
//	}
		s->substep_axis = SUBSTEP_AXIS_X;
//		LL_TIM_CC_DisableChannel(s->syncbase,MOTOR_X_CHANNEL);
		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_X_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_X_STEP_Pin_num*4)));
		s->substep_pulse_on = 1;
		s->substep_pulse_off = 0;

		MOTOR_Z_AllowPulse(); 
		LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
	}

	if (e2 > s->arc_dz) { // z step 
		s->current_task.z--;
		s->arc_err += s->arc_dz += s->arc_aa; 
	} else {
		s->substep_axis = SUBSTEP_AXIS_Z;

		s->substep_pin = (unsigned int *)((PERIPH_BB_BASE + ((uint32_t)&(MOTOR_Z_STEP_GPIO_Port->ODR) -PERIPH_BASE)*32 + (MOTOR_Z_STEP_Pin_num*4)));
		s->substep_pulse_on = 0;
		s->substep_pulse_off = 1;
		
		MOTOR_X_AllowPulse(); 
		LL_GPIO_SetOutputPin(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin);
		LL_GPIO_SetPinMode(MOTOR_Z_STEP_GPIO_Port,MOTOR_Z_STEP_Pin,LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinMode(MOTOR_X_STEP_GPIO_Port,MOTOR_X_STEP_Pin,LL_GPIO_MODE_ALTERNATE);
	}

	if(s->current_task.x == s->current_task.x1 && s->current_task.z == s->current_task.z1) {
		s->current_task.steps_to_end = 0; // end of arc
		return;
	}

	if(s->current_task.z == 0){ // end of quadrant
		s->current_task.steps_to_end = 0;
	}
}
*/
