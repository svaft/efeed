/*
 * g03.h
 *
 */

#ifndef G03_H_
#define G03_H_

#include "main.h"
#include "nuts_bolts.h"


void G03init_callback(void);
void arc_dx_callback(void);// arc movement callback
void arc_dz_callback(void); // arc movement callback
#define CW 1
#define CCW -1
void G03parse(char *line, int8_t cwccw);

#endif /* G03_H_ */
