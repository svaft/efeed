#include "gcode.h"
#include "nuts_bolts.h"

void G01parse(const char *str){
	G_pipeline init_gp;
	cb_push_back(&gp_cb, &init_gp);
}
void G03parse(const char *str){
	G_pipeline init_gp;
	cb_push_back(&gp_cb, &init_gp);
}
