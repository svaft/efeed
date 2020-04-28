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

#define task_precalc_size 20
#define task_size 20
#define gp_size 10
#define substep_size 6000
#define subdelay_precision 8 // 7 - 5,58us, 8 - 6,41us, 
#define substep_job_size task_size*2


typedef struct substep{
	uint8_t delay;
	uint8_t skip;
} substep_t;

typedef struct substep_sma_ref{
	uint16_t delay;
	uint16_t count;
	substep_t *ref;
} substep_sma_ref_t;


void command_parser(char *line);

void G94init_callback_precalculate(state_t* s);
void G95init_callback_precalculate(state_t* s);
void switch_to_async(state_t* s);
void switch_to_sync(state_t* s);
void G94(state_t* s);
void G95(state_t* s);

void G33parse(char *line);
void G00parse(char *line);

extern G_task_t gt_precalc[];
extern G_task_t gt[];
extern G_pipeline_t gp[];
extern G_pipeline_t init_gp;
extern substep_job_t substep_job[];
extern substep_sma_ref_t smaNumbers[];
extern substep_t* smaSubstepRefs[];



void calibrate_callback(state_t *);
void calibrate_init_callback(state_t* s);


void do_fsm_move_start2(state_t* s);

void do_fsm_move33(state_t*);
void do_fsm_move2(state_t*);
void do_fsm_move_end2(state_t* );
void load_next_task(state_t* s);


extern substep_t substep_delay[];
substep_t* cb_push_back_empty_ref(void);



void process_G_pipeline(void);
G_pipeline_t* G_parse(char *line);
__STATIC_INLINE 
G_task_t * add_empty_task(){
	cb_push_back_empty(&task_cb);
	return task_cb.top;
}
__STATIC_INLINE 
substep_job_t * add_empty_substep_job(){
	cb_push_back_empty(&substep_job_cb);
	return substep_job_cb.top;
}


//G_task* add_empty_task(void);
void add_task(int dx, int dz, int feed, int inc_dec, void *ref, uint32_t rr, uint8_t x_dir, uint8_t z_dir );
G_task_t* get_last_task( void );

#endif /* GCODE_H_ */
