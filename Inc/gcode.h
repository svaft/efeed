/*
 * gcode.h
 *
 */

#ifndef GCODE_H_
#define GCODE_H_

#include "main.h"

void G01parse(const char *str);
void G03parse(const char *str);
void G33parse(const char *str);
void G00parse(const char *str);


typedef struct G_pipeline{
	int X,Z,feed;
	bool sync;
	uint8_t code;
} G_pipeline;


void process_G_pipeline(void);



#endif /* GCODE_H_ */
