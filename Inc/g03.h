/*
 * g03.h
 *
 */

#ifndef G03_H_
#define G03_H_

#include "main.h"
#include "nuts_bolts.h"

void plotOptimizedEllipse(int x0, int z0, int x1, int z1, int a, int b);
void plotEllipse(int x0, int z0, int a, int b);
void G03init_callback(state_t* s);
void G03init_callback_precalculate(state_t* s);

	//void arc_dx_callback(void);// arc movement callback
//void arc_dz_callback(void); // arc movement callback

uint8_t get_quadrant(int x0, int z0);

void arc_q1_callback_light(state_t* s);
void arc_q1_callback(state_t* s);
void arc_q2_callback(state_t* s);
void arc_q3_callback(state_t* s);
void arc_q4_callback(state_t* s);

#define CW 1
#define CCW -1
void G03parse(char *line, int8_t cwccw);

#endif /* G03_H_ */
