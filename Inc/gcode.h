/*
 * gcode.h
 *
 */

#ifndef GCODE_H_
#define GCODE_H_

#include "main.h"

void G01parse(char *line);
void G03parse(char *line);
void G33parse(char *line);
void G00parse(char *line);


typedef struct G_pipeline{
	int X,Z,feed; //general variables
	int X0,Z0,XC,ZC,I,K,R; //for arc
	bool sync;
	uint8_t code;
} G_pipeline;


void process_G_pipeline(void);



#endif /* GCODE_H_ */
