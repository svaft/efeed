#include "gcode.h"
#include "nuts_bolts.h"
//int ii, kk;

G_pipeline* G_parse(char *line){
  uint8_t char_counter = 0;  
	G_pipeline init_gp={0,0,0,0,0};
	cb_init_by_top(&gp_cb,&init_gp);

	int x_old = init_gp.X, z_old = init_gp.Z;
	char letter;
	char *end;

  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    
    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter++];
		switch(letter){
			case 'X':
				init_gp.X = str_f_to_steps2210(line, &char_counter);
				init_gp.X >>= 2; //Fusion360 generate X in diameter mode, so divide by 2
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


void G01parse(char *line){
	G_pipeline *gref = G_parse(line);
	gref->code = 1;
}

int xc,zc,r;
void G03parse(char *line){
	G_pipeline *gref = G_parse(line);
	gref->code = 3;
	int ii = gref->I >> 10,kk = gref->K >> 10; //back from 2210 to steps
	gref->R = SquareRoot(ii*ii + kk*kk);
//	xc = gref->X + gref->I;
//	zc = gref->Z + gref->K;
}
