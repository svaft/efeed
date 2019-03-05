#include "gcode.h"
#include "nuts_bolts.h"
#include "fsm.h"

fixedpt command;
G_pipeline init_gp={0,0,0,0,0};

G_task * add_empty_task(){
	cb_push_back_empty(&task_cb);
	return task_cb.top;
}

//__STATIC_INLINE 
void add_task(int dx, int dz, int feed, int inc_dec, void *ref, uint32_t rr, uint8_t x_dir, uint8_t z_dir ){
	G_task tmp_gt;
	tmp_gt.dx = dx;
	tmp_gt.dz = dz;
	tmp_gt.F = feed;
	tmp_gt.inc_dec = inc_dec;
	tmp_gt.callback_ref = ref;
	tmp_gt.rr = rr;
	tmp_gt.x_direction = x_dir;
	tmp_gt.z_direction = z_dir;
	cb_push_back(&task_cb, &tmp_gt);

}


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
					case 36864000: //G90  absolute distance mode
						
						break;
					case 38502400: //G94 Units per Minute Mode
						state.G94G95 = 0;
						break;
					case 38912000: //G95 - is Units per Revolution Mode
						state.G94G95 = 1;
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
				init_gp.F = str_f_to_824(line, &char_counter);
				break;
		}
	}

	cb_push_back(&gp_cb, &init_gp);
	return gp_cb.top;
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




void G01parse(char *line){
	int x0 = init_gp.X; //get from prev gcode
	int z0 = init_gp.Z;
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

	gt_new_task->x_direction = xdir;
	gt_new_task->z_direction = zdir;

//		bool G94G95; // 0 - unit per min, 1 - unit per rev
	if(state.G94G95 == 1){ 	// unit(mm) per rev
		gt_new_task->F = str_f824mm_rev_to_delay824(gref->F);
	} else { 											// unit(mm) per min
		gt_new_task->F = str_f824mm_min_to_delay824(gref->F);
	}

	gref->code = 1;
}

//int xc,zc,r;


void G03parse(char *line, int8_t cwccw){
	int x0 = init_gp.X; //get from prev gcode
	int z0 = init_gp.Z;
//	int pos_count; // 1st octant count by X
	G_pipeline *gref = G_parse(line);
	gref->code = 3;

	int ii 	= gref->I >> 10, kk = gref->K >> 10; //back from 2210 to steps
	int rr = SquareRoot(ii*ii + kk*kk); // find arc radius
	rr *= rr;
	int octant = SquareRoot(rr >>1) << 10; // find octant value
//	gref->R	= SquareRoot(ii*ii + kk*kk); // find arc radius
//	gref->R *= gref->R; // gref->R *= gref->R; // pow2
//	int octant = SquareRoot(gref->R >>1) << 10; // find octant value

 // more preceise 64bit evaluation of R and octant:
/*
	uint64_t ik0 = abs(gref->I);
	uint64_t ik = ik0*ik0;
	ik0 = abs(gref->K);
	ik += ik0*ik0;
	ik  = SquareRoot64(ik);
	ik *= ik;
	uint64_t octant64 = SquareRoot64(ik >>1); // find octant value
*/

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

	if(oct0 == oct1){
		gt_new_task 		= add_empty_task();
		gt_new_task->rr = rr;
		switch(oct0){
			case 0: case 3: case 7: case 4:
//				pos_count = abs(x1z - x0z);
				gt_new_task->dx = abs(x1z - x0z);
				gt_new_task->callback_ref = arc_dx_callback;
				break;
			case 1: case 2: case 5: case 6:
//				pos_count = abs(z0z - z1z);
				gt_new_task->dz = abs(z0z - z1z);//pos_count;
				gt_new_task->callback_ref = arc_dz_callback;
				break;
		}
		// X direction:
		switch(oct0){
			case 0: case 1: case 4: case 5:
				gt_new_task->x_direction = xdir_forward;
			case 2: case 3: case 6: case 7:
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
			gt_new_task->rr = rr;
// X direction
			switch(current_oct){
				case 0: case 1: case 4: case 5:
					gt_new_task->x_direction = xdir_forward; 				break;
				case 2: case 3: case 6: case 7:
					gt_new_task->x_direction = xdir_backward;				break;
			}
			gt_new_task->z_direction = zdir_forward;

			if(current_oct == oct0){
				switch(current_oct){
					case 0: case 7:
//						pos_count = abs(octant - x0z);
						gt_new_task->dx = abs(octant - x0z);//pos_count;
						gt_new_task->callback_ref = arc_dx_callback;
						break;
					case 1: case 6:
//						pos_count = abs(z0z);
						gt_new_task->dz = abs(z0z);//pos_count;
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 2: case 5:
//						pos_count = abs(octant - z0z);
						gt_new_task->dz = abs(octant - z0z);
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 3: case 4:
//						pos_count = abs(x0z);
						gt_new_task->dx = abs(x0z); //pos_count;
						gt_new_task->callback_ref = arc_dx_callback;
						break;
				}
			} else if(current_oct == oct1){
				switch(current_oct){
					case 0: case 7:
						while(1);
					case 1: case 6:
//						pos_count = abs(octant - z1z);
						gt_new_task->dz = abs(octant - z1z);
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 2: case 5:
//						pos_count = abs(z1z);
						gt_new_task->dz = abs(z1z);
						gt_new_task->callback_ref = arc_dz_callback;
						break;
					case 3: case 4:
//						pos_count = abs(octant - x1z);
						gt_new_task->dx = abs(octant - x1z);
						gt_new_task->callback_ref = arc_dx_callback;
						break;
				}
			} else {
//				pos_count =octant;
				switch(current_oct){
					case 0: case 3: case 7: case 4:
						gt_new_task->dx = octant;
						gt_new_task->callback_ref = arc_dx_callback;
						break;
					case 1: case 2: case 5: case 6:
						gt_new_task->dz = octant;
						gt_new_task->callback_ref = arc_dz_callback;
						break;
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
			current_oct +=cwccw;
			if(current_oct == 255)
				current_oct = 7;
			else if(current_oct == 8)
				current_oct = 0;
		}
	}					

/*
	if(cwccw == CW) { // clockwise arc
		if(pos_count == 0){ // same octant?
			switch(oct0){
				case 0: case 3: case 7: case 4:
					pos_count += abs(x1z - x0z);
					break;
				case 1: case 2: case 5: case 6:
					pos_count += abs(z0z - z1z);
					break;
			}
		}

		
		
		if(oct0 < oct1){ //CW, outer(right) part of the arc this kind: ")"
			for (int oct = oct0; oct <= oct1; oct++){
				if(oct == oct0){
					switch(oct){
						case 0:
							pos_count += (octant - x0z);
							break;
						case 1:
							pos_count += z0z;
							break;
						case 2:
							pos_count += (octant - z0z);
							break;
						case 3:
							pos_count += x0z;
							break;
					}
					continue;
				}
				if(oct == oct1){
					switch(oct){
						case 0:
							while(1);
						case 1:
							pos_count += (octant - z1z);
							break;
						case 2:
							pos_count += abs(z1z);
							break;
						case 3:
							pos_count += (octant - x1z);
							break;
					}
				} else pos_count +=octant;
			}	
		} else if(oct0 == oct1){ // if arc start and end in the same octant 
			switch(oct0){
				case 0:
				case 3:
				case 7:
				case 4:
					pos_count += abs(x1z - x0z);
					break;
				case 1:
				case 2:
				case 5:
				case 6:
					pos_count += abs(z0z - z1z);
					break;
			}
		} else { // not supporder on lathe?
			while(1);
		} 
	} else { // CCW
		if (oct1 < oct0){ 	//CCW, inner(left) part of the arc this kind: "("
			for (int oct = oct0; oct >= oct1; oct--){
				if(oct == oct0){
					switch(oct){
						case 7:
							pos_count += abs(octant - x0z);
							break;
						case 6:
							pos_count += abs(z0z);
							break;
						case 5:
							pos_count += abs(octant - z0z);
							break;
						case 4:
							pos_count += abs(x0z);
							break;
					}
					continue;
				}
				if(oct == oct1){
					switch(oct){
						case 7:
							while(1);
						case 6:
							pos_count += abs(octant - z1z);
							break;
						case 5:
							pos_count += abs(z1z);
							break;
						case 4:
							pos_count += abs(octant - x1z);
							break;
					}
				} else pos_count +=octant;
			}		
		} else if(oct0 == oct1){ // if arc start and end in the same octant 
			switch(oct0){
				case 0:
				case 3:
				case 7:
				case 4:
					pos_count += abs(x1z - x0z);
					break;
				case 1:
				case 2:
				case 5:
				case 6:
					pos_count += abs(z0z - z1z);
					break;
			}
		} else { // not supporder on lathe?
			while(1);
		} 
	}
*/	
/*
	if(oct0 < oct1){ //CW, outer(right) part of the arc this kind: ")"
		for (int oct = oct0; oct <= oct1; oct++){
			if(oct == oct0){
				switch(oct){
					case 0:
						pos_count += (octant - x0z);
						break;
					case 1:
						pos_count += z0z;
						break;
					case 2:
						pos_count += (octant - z0z);
						break;
					case 3:
						pos_count += x0z;
						break;
				}
				continue;
			}
			if(oct == oct1){
				switch(oct){
					case 0:
						while(1);
					case 1:
						pos_count += (octant - z1z);
						break;
					case 2:
						pos_count += abs(z1z);
						break;
					case 3:
						pos_count += (octant - x1z);
						break;
				}
			} else pos_count +=octant;
		}	
	} else if(oct0 == oct1){ // if arc start and end in the same octant 
		switch(oct0){
			case 0:
			case 3:
			case 7:
			case 4:
				pos_count += abs(x1z - x0z);
				break;
			case 1:
			case 2:
			case 5:
			case 6:
				pos_count += abs(z0z - z1z);
				break;
		}
	} else if (oct1 < oct0){ 	//CCW, inner(left) part of the arc this kind: "("
		for (int oct = oct0; oct >= oct1; oct--){
			if(oct == oct0){
				switch(oct){
					case 7:
						pos_count += abs(octant - x0z);
						break;
					case 6:
						pos_count += abs(z0z);
						break;
					case 5:
						pos_count += abs(octant - z0z);
						break;
					case 4:
						pos_count += abs(x0z);
						break;
				}
				continue;
			}
			if(oct == oct1){
				switch(oct){
					case 7:
						while(1);
					case 6:
						pos_count += abs(octant - z1z);
						break;
					case 5:
						pos_count += abs(z1z);
						break;
					case 4:
						pos_count += abs(octant - x1z);
						break;
				}
			} else pos_count +=octant;
		}		
	}
	*/
//	pos_count >>= 10;
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
				 |
				 |
		 \ 7 | 0 /
			\  |  /
			 \ | /       
			6 \|/ 1
	 ------0----------->X
			5 /|\ 2
			 / | \
			/  |  \      
		 / 4 | 3 \



	
*/	
}
