/*
 * g01.h
 *
 */

#ifndef G01_H_
#define G01_H_

#include "main.h"
#include "nuts_bolts.h"

void G00G01init_callback(state_t* s);
void G00init_callback(state_t* s);
void G01init_callback(state_t* s);
void G33init_callback(state_t* s);
void dxdz_callback(state_t* s); // line movement callback

void dxdz_callback_precalculate(state_t* s);
void G01init_callback_precalculate(state_t* s);

void G01parse(char *line, bool G00G01);

#endif /* G01_H_ */
