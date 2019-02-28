#include "gcode.h"
#include "nuts_bolts.h"
//int ii, kk;

void command_parser(char *line){
  uint8_t char_counter = 0;  
	char letter;
//	char *end;
	fixedpt command;
  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    letter = line[char_counter++];
		switch(letter){
			case 'G':
				command = str_f_to_steps2210(line, &char_counter);
				switch(command){
					//todo now its only one G command per line supported
					case 409600://G1 command packed into 2210_400 format
						G01parse(line+char_counter+1);
						return;	
					case 1228800: //G3
						G03parse(line+char_counter+1);
						return;	
					case 13516800: //G33
						return;	
				}
			
				break;
		}
	}

}

G_pipeline* G_parse(char *line){
  uint8_t char_counter = 0;  
	G_pipeline init_gp={0,0,0,0,0};
	cb_init_by_top(&gp_cb,&init_gp);

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
				init_gp.feed = str_f_to_steps2210(line, &char_counter);
				break;
		}
	}

	cb_push_back(&gp_cb, &init_gp);
	return gp_cb.top;
}


uint8_t get_octant(int x0z,int z0z, int octant10){
	if(x0z>0){								//0,1,2,3
		if(x0z < octant10){			//0,3
			if(z0z>0){ 						//oct0
				return 0;
			} else{ 							//oct3
				return 3;
			}
		} else { 								//1,2
			if(z0z>0){ 						//oct1
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
	G_pipeline *gref = G_parse(line);
	gref->code = 1;
}

//int xc,zc,r;
int oct0,oct1;
int pos_count; // 1st octant count by X

void G03parse(char *line){
	G_pipeline *gref_prev = gp_cb.top;
	int x0 = gref_prev->X;
	int z0 = gref_prev->Z;
	G_pipeline *gref = G_parse(line);
	gref->code = 3;

	int ii 		= gref->I >> 10, kk = gref->K >> 10; //back from 2210 to steps
	gref->R 	= SquareRoot(ii*ii + kk*kk); // find arc radius
	gref->R *= gref->R; // pow2
	int octant = SquareRoot(gref->R >>1) << 10; // find octant value

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

	int zdelta = -(z0+gref->K); // center delta to shift it to zero 
	int xdelta = -(x0+gref->I);

	int x0z = x0+xdelta;
	int z0z = z0+zdelta;
	int x1z = gref->X+xdelta;
	int z1z = gref->Z+zdelta;
	oct0 = get_octant(x0z, z0z, octant);
	oct1 = get_octant(x1z, z1z, octant);
	pos_count = 0;

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
	pos_count >>= 10;
/*
  общее число шагов для дуги равно сумме шагов по осям в соответствии с текущей октантой
	для октант 0,3,4 и 7 основной осью является ось Х, сдвиг по оси Z вычисляется как корень(R*R-dx*dx)
	для октант 1,2,5 и 6 основной осью является ось Z, сдвиг по оси X вычисляется как корень(R*R-dz*dz)
	Если дуга в нескольких октантах, например начинается в 0-1-2,
	то число шагов равно:
		участок в 0м октанте:
		octant-x1
		участок в 1м октанте:
		octant
		участок в 2м октанте:
		x2
		итого (octant-x1 + octant + x2)
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
