/*
 * gcode.h
 *
 */

#ifndef GCODE_H_
#define GCODE_H_

#include "main.h"
#include "nuts_bolts.h"
void command_parser(char *line);

void G01parse(char *line);
#define CW 1
#define CCW -1
void G03parse(char *line, int8_t cwccw);
void G33parse(char *line);
void G00parse(char *line);

typedef void (*callback_func_t)(void);


typedef struct G_pipeline{
	int X,Z,F,P; //general registers
	int I,K; //for arc
	bool sync;
	uint8_t code;
} G_pipeline;
extern G_pipeline gp[];

typedef struct G_task{
	int32_t dx, dz; // delta
	uint32_t steps_to_end;
	fixedptu F; //Q824
	callback_func_t callback_ref; //callback ref to iterate line or arc
	callback_func_t init_callback_ref;
	uint8_t z_direction, x_direction;
	int rr, inc_dec;
} G_task;
extern G_task gt[];


void process_G_pipeline(void);
G_pipeline* G_parse(char *line);
__STATIC_INLINE 
G_task * add_empty_task(){
	cb_push_back_empty(&task_cb);
	return task_cb.top;
}

//G_task* add_empty_task(void);
void add_task(int dx, int dz, int feed, int inc_dec, void *ref, uint32_t rr, uint8_t x_dir, uint8_t z_dir );
G_task* get_last_task( void );

#endif /* GCODE_H_ */
