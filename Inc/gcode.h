/*
 * gcode.h
 *
 */

#ifndef GCODE_H_
#define GCODE_H_

#include "main.h"
#include "nuts_bolts.h"
#include "g01.h"
#include "g03.h"
#include "g04.h"


void command_parser(char *line);

void G33parse(char *line);
void G00parse(char *line);

extern G_task gt[];
extern G_pipeline gp[];
extern G_pipeline init_gp;

void do_fsm_move_start2(state_t* s);
void do_fsm_move2(state_t*);
void do_fsm_move_end2(state_t* );
void load_next_task(void);


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
