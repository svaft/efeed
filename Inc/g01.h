/*
 * g01.h
 *
 */

#ifndef G01_H_
#define G01_H_

#include "main.h"
#include "nuts_bolts.h"

void G00G01init_callback(state_t* s);
void G00G01init_callback3(state_t* s);

void G00init_callback(state_t* s);
void G01init_callback(state_t* s);
void G33init_callback(state_t* s);
void dx_callback(state_t* s); // line movement callback
void dz_callback(state_t* s); // line movement callback

void dxdz_callback_precalculate(state_t* s);

void G01init_callback_precalculate(state_t* s);

void G01parse(char *line, uint8_t gcode);
void G01parsed(int x0, int x0r, int z0, int F, int X, int Z, int Xr, bool G00G01);
void scheduleG00G01move(int X, int Z, int F, uint8_t G00G01G33);
void scheduleG00G01move3(int X, int Z, int F, int Xr, uint8_t G00G01G33);
#endif /* G01_H_ */
