#include "gcode.h"
#include "fsm.h"
#include "g03.h"

int count = 0;
int count_total = 0;
uint8_t bufx[2000];
uint8_t bufz[2000];

#define BIT_BAND_SRAM(RAM,BIT) (*(volatile uint32_t*)(SRAM_BB_BASE+32*((uint32_t)((void*)(RAM))-SRAM_BASE)+4*((uint32_t)(BIT))))
	

int64_t err2;
int32_t x32, z32, ex, ey, e2, err;
void plotEllipse(int64_t x0, int64_t z0, int64_t a, int64_t b){
	int64_t x = x0, z = z0; /* II. quadrant from bottom left to top right */
	e2 = (int64_t)b*b, 
//	err = x*(2*e2+x)+e2; /* error of 1.step */

	err = (x0+1)*(x0+1)*b*b + (z0+1)*(z0+1)*a*a - a*a*b*b;
	// -373 403 -321632000	
	z32 = z;
	x32 = x;
	do {
		e2 = 2*err;
		ex = (x*2+1)*(int64_t)b*b;
		if (e2 >= ex){ /* e_xy+e_x > 0 */
			err += (++x*2+1)*(int64_t)b*b;
			BIT_BAND_SRAM(&bufx,count) = 1;
			x32 = x;
		}
		ey = (z*2+1)*(int64_t)a*a;
		if (e2 <= ey){ /* e_xy+e_y < 0 */
			err += (++z*2+1)*(int64_t)a*a;
			BIT_BAND_SRAM(&bufz,count) = 1;
			z32 = z;
		}
		count++;
	} while (x <= 0);
	count_total = --count;
	x0 = 0;
	z0 = 4;
	x = x0;
	z = z0;

/*	do{ //bitband by precalculated quadrant, fast but memory depended
		if(BIT_BAND_SRAM(&bufx,count) == 1){
			x32++;
		}
		if(BIT_BAND_SRAM(&bufz,count) == 1){
			z32--;
		}
		count--;
	} while(count >= 0);
*/

/// II	
	do {
		int exy = (x+1)*(x+1)*b*b + (z-1)*(z-1)*a*a - a*a*b*b;
		int eyloc = (x+1)*(x+1)*b*b + (z)*(z)*a*a - a*a*b*b; // ey=(x+1)²b²+y²a²–a²b²
		int exloc = (x)*(x)*b*b + (z-1)*(z-1)*a*a - a*a*b*b;
		if(exloc+exy<0){
			x++;
		}
		if(eyloc+exy>0){
			z--;
		}
	} while (z >= 0);

// III	
	x = 7;
	z = 0;
	do {
		int exy = (x0-1)*(x-1)*b*b + (z-1)*(z-1)*a*a - a*a*b*b;
		int eyloc = (x-1)*(x-1)*b*b + (z)*(z)*a*a - a*a*b*b; // ey=(x+1)²b²+y²a²–a²b²
		int exloc = (x)*(x)*b*b + (z-1)*(z-1)*a*a - a*a*b*b;
		if(exloc+exy>0){
			x--;
		}
		if(eyloc+exy<0){
			z--;
		}
	} while (x >= 0);

// IV	
	x = 0;
	z = -4;
	do {
		int exy = (x0-1)*(x-1)*b*b + (z+1)*(z+1)*a*a - a*a*b*b;
		int eyloc = (x-1)*(x-1)*b*b + (z)*(z)*a*a - a*a*b*b; // ey=(x+1)²b²+y²a²–a²b²
		int exloc = (x)*(x)*b*b + (z+1)*(z+1)*a*a - a*a*b*b;
		if(exloc+exy<0){
			x--;
		}
		if(eyloc+exy>0){
			z++;
		}
	} while (x >= 0);
	
	
	
	/*


	err = (x0+1)*(x0+1)*b*b + (z0-1)*(z0-1)*a*a - a*a*b*b;
	do {
		e2 = 2*err;
		ex = (x*2+1)*(int64_t)b*b;
		if (e2 < ex){ // e_xy+e_x > 0 
			err += (++x*2+1)*(int64_t)b*b;
			x32 = x;
		}
		ey = (z*2+1)*(int64_t)a*a;
		if (e2 > ey){ //e_xy+e_y < 0	
			err -= (--z*2-1)*(int64_t)a*a;
			z32 = z;
		}
	} while (x <= 400);

	
//	while (z++ < b) { /* to early stop of flat ellipses a=1, */
//			z32 = z;
			//z32 = z;
		//setPixel(xm, ym+y); /* -> finish tip of ellipse */
		//setPixel(xm, ym-y);
//	}


}

//int64_t x = 0, z = 0;
void plotOptimizedEllipse(int x0, int z0, int a, int b){
	int64_t x = -a;
	int64_t z = 0; /* II. quadrant from bottom left to top right */
	int64_t e2 = b, dx = (1+2*x)*e2*e2; /* error increment */
	int64_t dz = x*x, err = dx+dz; /* error of 1.step */
	z32 = z;
	x32 = x;
	do {
//		setPixel(xm, ym-y);
		count++;
		e2 = 2*err;
		if (e2 >= dx) { 
			x++; 
			x32 = x;
			err += dx += 2*(int64_t)b*b; 
		} /* x step */
		if (e2 <= dz) { 
			z++;
		z32 = z;			
			err += dz += 2*(int64_t)a*a; 
		} /* z step */
	} while (x <= 0);
	while (z++ < b) { /* to early stop for flat ellipses with a=1, */
		z32 = z;			
//		setPixel(xm, ym+y); /* -> finish tip of ellipse */
//		setPixel(xm, ym-y);
	}
}
// on task load callback
void G03init_callback(void){
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
	t3ccer[TIM_CCER_CC1E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
	int dz = SquareRoot(s->current_task.rr - s->current_task.dx * s->current_task.dx);
	if(dz != s->current_task.dz){
		t3ccer[TIM_CCER_CC3E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
		s->current_task.dz = dz;
	}
	s->current_task.dx += s->current_task.inc_dec;
}

void arc_dz_callback(){
	state_t *s = &state;
	TIM3->CCER = 0;	//	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	t3ccer[TIM_CCER_CC3E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); 
	int dx = SquareRoot(s->current_task.rr - s->current_task.dz * s->current_task.dz);
	if(dx != s->current_task.dx){
	t3ccer[TIM_CCER_CC1E_Pos] = 1; //		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 
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




/**
* @brief  G33 parse to tasks
* @retval void.
  */
void G03parse(char *line, int8_t cwccw){ //~130-150us
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
/*
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
}
