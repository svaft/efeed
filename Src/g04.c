#include "gcode.h"
#include "fsm.h"
#include "g04.h"

void G04parse(char *line){
	G_pipeline_t *gref = G_parse(line);
	G_task_t *gt_new_task = add_empty_task();
	gt_new_task->steps_to_end = (gref->P * 1000) >> 10;
	gt_new_task->callback_ref = dwell_callback;
	gt_new_task->init_callback_ref = G04init_callback;
}

void G04init_callback(state_t* s){
	s->function = do_fsm_dwell;
//  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED); // disable pulse generation
	LL_TIM_CC_DisableChannel(TIM3,LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
//	TIM3->CCER = 0; // disable pulse generation
	TIM2->ARR = 30;
}

void do_fsm_dwell(state_t *s){
	// callback from TIM2
}

void dwell_callback(state_t* s){
// callback from TIM3
	s->current_task.steps_to_end--;
	if(s->current_task.steps_to_end == 0){
		//restore connection here?
			LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
	}
}
