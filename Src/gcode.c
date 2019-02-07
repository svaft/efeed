/* This code is inspired by the "Grbl"
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 * Copyright (c) 2011 Sungeun K. Jeon
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "gcode.h"

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define MM_PER_INCH (25.4)

#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL_G4 1
#define NEXT_ACTION_GO_HOME_G28 2
#define NEXT_ACTION_RESET_XYZ_G92 3
#define NEXT_ACTION_STOP 4
#define NEXT_ACTION_SEEK_G0 5 // G0 
#define NEXT_ACTION_LINEAR_G1 6 // G1
#define NEXT_ACTION_EXTRUDER_STOP 7
#define NEXT_ACTION_EXTRUDER_ON 8
#define NEXT_ACTION_EXTRUDER_FAST_T 9
#define NEXT_ACTION_EXTRUDER_WAIT_T 10
#define NEXT_ACTION_CW_ARC 11
#define NEXT_ACTION_CCW_ARC 12

typedef struct {
	uint8_t status_code;
	uint8_t inches_mode;             /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
	uint8_t absolute_mode;           /* 0 = relative motion, 1 = absolute motion {G90, G91} */
	uint8_t spindle_on, extruder_on;
	double feed_rate, seek_rate;     /* Millimeters/second */
	double extruder_length, extruder_k;
	double position[3];              /* Where the interpreter considers the tool to be at this point in the code */
	int16_t s_value;           /* RPM/100 or temperature of the current extruder */
	uint8_t next_action;  /* The action that will be taken by the parsed line */
} parser_state_t;

parser_state_t gc;

#define FAIL(status) { gc.status_code = status; return(gc.status_code); }

double strtod_M(const char *str, char **endptr)
{
	double number = 0.0;
	double div = 0.0;
	bool negative = false;
	bool plus = false;
	bool skip = true;
	char c;
	while ((c = *str) != 0)
	{
		if (c == '+')
		{
			if (skip && !plus)
			{
				plus = true;
				skip = false;
			}
			else
				break;
		}
		else if (skip && !negative && c == '-')
		{
			if (skip && !negative)
			{
				negative = true;
				skip = false;
			}
			else
				break;
		}
		else if (c == '.')
		{
			if (div == 0.0)
				div = 1.0;
			else
				break;
		}
		else if (c >= '0' && c <= '9')
		{
			skip = false;
			if (div == 0.0)
				number = number * 10.0 + (double)(c - '0');
			else
			{
				div *= 10.0;
				number += ((double)(c - '0') / div);
			}
		}
		else if (!skip)
		{
			break;
		}
		str++;
	}

	if (negative) number = -number;
	if (endptr != NULL) *endptr = (char *)str;
	return number;
}

static int read_double(char *line, int *char_counter, double *double_ptr)
{
	char *start = line + *char_counter;
	char *end;

	*double_ptr = strtod_M(start, &end);
	if (end == start)
	{
		gc.status_code = GCSTATUS_BAD_NUMBER_FORMAT;
		return false;
	}
	*char_counter = (int)(end - line);
	return true;
}

static int next_statement(char *letter, double *double_ptr, char *line, int *char_counter)
{
	while (line[*char_counter] == ' ') (*char_counter)++;

	if (line[*char_counter] == 0 || line[*char_counter] == ';' ||
		line[*char_counter] == '\n' || line[*char_counter] == '\r') return false;
	*letter = line[*char_counter];
	if ((*letter < 'A') || (*letter > 'Z'))
	{
		gc.status_code = GCSTATUS_EXPECTED_COMMAND_LETTER;
		return false;
	}
	(*char_counter)++;
	return read_double(line, char_counter, double_ptr);
}

void gc_init(void)
{
	memset(&gc, 0, sizeof(gc));
	gc.feed_rate = SM_DEFAULT_FEED_RATE;
	gc.seek_rate = SM_DEFAULT_SEEK_RATE;
	gc.absolute_mode = true;
	// gc.startPosX = commonValues.startX;
	// gc.startPosY = commonValues.startY;
	// gc.startPosZ = commonValues.startZ;
	gc.extruder_k = 1;
	// commonValues.extruder_k;
	gc.next_action = NEXT_ACTION_DEFAULT;
}

static double to_millimeters(double value)
{
	return (gc.inches_mode ? (value * MM_PER_INCH) : value);
}


// Executes one line of 0-terminated G-Code. The line is assumed to contain only upper case
// characters and signed floating point values (no whitespace).
uint8_t gc_execute_line(char *line)
{
//	double feed_rate;
//	double extrudeLength;
	int char_counter = 0;
	char letter;
	double value;
//	int pause_value = 0;
//	uint8_t radius_mode = false;

	gc.status_code = GCSTATUS_OK;
	
	if (line[0] == ';'
		|| line[0] == '('
		|| line[0] == '%'
		)
		return GCSTATUS_OK;	// comments

	// Pass 1: Commands
	while (next_statement(&letter, &value, line, &char_counter))
	{
		int int_value = (int)value;
		switch (letter)
		{
		case 'N':
			break;
		case 'G':
			switch (int_value)
			{
			case 0: gc.next_action = NEXT_ACTION_SEEK_G0; break;
			case 1: gc.next_action = NEXT_ACTION_LINEAR_G1; break;
			case 2: gc.next_action = NEXT_ACTION_CW_ARC; break;
			case 3: gc.next_action = NEXT_ACTION_CCW_ARC; break;
			case 4: gc.next_action = NEXT_ACTION_DWELL_G4; break;
			case 20: gc.inches_mode = true; break;
			case 21: gc.inches_mode = false; break;
			case 30:
			case 28: gc.next_action = NEXT_ACTION_GO_HOME_G28; break;
			case 90: gc.absolute_mode = true; break;
			case 91: gc.absolute_mode = false; break;
			case 92: gc.next_action = NEXT_ACTION_RESET_XYZ_G92; break;
			case 64:
			case 40:
			case 17:	// G17 Выбор рабочей плоскости X-Y
			case 94:	// Feedrate per minute
			case 98:	// Feedrate per minute (group type A)
			case 97:	// Constant spindle speed M T Takes an S address integer, which is interpreted as rev/min (rpm). The default speed mode per system parameter if no mode is programmed. 
			case 49:	// Tool length offset compensation cancel
			case 80:	// Cancel canned cycle
				break;
			default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
			}
			break;
		case 'M':
			switch (int_value)
			{
			case 112: // Emergency Stop 
			case 0:
			case 1:
			case 2:
			case 30:
			case 60:
				gc.next_action = NEXT_ACTION_STOP;
				break;
			case 3: gc.spindle_on = 1; break;
			//	case 4: gc.spindle_direction = -1; break;
			case 5: gc.spindle_on = 0; break;
			case 23: // Thread gradual pullout ON
			case 24: // Thread gradual pullout OFF
			case 52: // Unload Last tool from spindle
			case 49: // Feedrate override NOT allowed
			case 48: // Feedrate override allowed
			case 8:  // Coolant on
			case 9:  // Coolant off
			case 105: // M105: Get Extruder Temperature Example: M105 Request the temperature of the current extruder and the build base in degrees Celsius. The temperatures are returned to the host computer. For example, the line sent to the host in response to this command looks like 
			case 106: // M106: Fan On Example: M106 S127 Turn on the cooling fan at half speed. Optional parameter 'S' declares the PWM value (0-255) 
			case 107: // Fan Off 
			case 108: // M108: Set Extruder Speed  Sets speed of extruder motor. (Deprecated in current firmware, see M113) 
			case 110: // Set Current Line Number 
			case 113: // Set Extruder PWM 
			case 140: // Bed Temperature (Fast) Example: M140 S55 Set the temperature of the build bed to 55oC 
			case 141: //Chamber Temperature (Fast) Example: M141 S30 Set the temperature of the chamber to 30oC
			case 142: // Holding Pressure Example: M142 S1 Set the holding pressure of the bed to 1 bar. 
			case 6:
				return gc.status_code;
			default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
			}
			break;
		}
		if (gc.status_code)
			return (gc.status_code);
	}
	if (gc.status_code)
		return(gc.status_code);

	char_counter = 0;

	// Pass 2: Parameters
	while (next_statement(&letter, &value, line, &char_counter))
	{
		double unit_millimeters_value = to_millimeters(value);
		switch (letter)
		{
//		case 'E': extrudeLength = value; break;
		case 'F':
			if (gc.next_action == NEXT_ACTION_SEEK_G0)
				gc.seek_rate = unit_millimeters_value;
			else
				gc.feed_rate = unit_millimeters_value; // millimeters pr min
			//	if (unit_millimeters_value > SM_MAX_FEEDRATE)
			//		FAIL(GCSTATUS_UNSOPORTED_FEEDRATE);
			break;
//		case 'P': pause_value = (int)value; break;
		case 'S': gc.s_value = (int16_t)value; break;
		case 'X':
		case 'Y':
		case 'Z':
			if (gc.absolute_mode)
				gc.position[letter - 'X'] = unit_millimeters_value;
			else
				gc.position[letter - 'X'] += unit_millimeters_value;
			break;
		case 'I':
		case 'J':
		case 'K':
//			offset[letter - 'I'] = unit_millimeters_value;
			break;
		case 'R':
//			radius = unit_millimeters_value;
//			radius_mode = true;
			break;
		case 'G':
		case 'N':
		case 'M':
			break;
		default:
			FAIL(GCSTATUS_UNSUPPORTED_PARAM);
		}
	}
	
	return(gc.status_code);
}
