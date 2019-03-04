/*
 * gcode.h
 *
 */

#ifndef GCODE_H_
#define GCODE_H_

#include "main.h"

void command_parser(char *line);

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
extern G_pipeline gp[];

typedef struct G_task{
	int32_t dx, dz; // delta
	fixedptu feed; //Q824
	void *callback_ref; //callback ref to iterate line or arc
	uint8_t z_direction, x_direction;
	int rr, inc_dec;
} G_task;
extern G_task gt[];


void process_G_pipeline(void);

G_task * add_empty_task();
void add_task(int dx, int dz, int feed, int inc_dec, void *ref, uint32_t rr, uint8_t x_dir, uint8_t z_dir );
G_task* get_last_task( void );

#endif /* GCODE_H_ */
