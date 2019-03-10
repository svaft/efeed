/*
 * g04.h
 *
 */

#ifndef G04_H_
#define G04_H_

#include "main.h"
#include "nuts_bolts.h"

void G04init_callback(void);
void dwell_callback(void);
void G04parse(char *line);
void do_fsm_dwell(state_t *s);

#endif /* G04_H_ */
