/*
 * g01.h
 *
 */

#ifndef G01_H_
#define G01_H_

#include "main.h"
#include "nuts_bolts.h"

void calibrate_callback(state_t *);
void calibrate_init_callback(void);


void G01init_callback(void);
void dxdz_callback(void); // line movement callback
void G01parse(char *line);

#endif /* G01_H_ */
