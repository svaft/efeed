#include "nuts_bolts.h"

circular_buffer gp_cb;
circular_buffer task_cb;
circular_buffer task_precalc_cb;
circular_buffer substep_cb;
circular_buffer substep_job_cb;


void cb_init_ref(circular_buffer *cb, size_t capacity, size_t sz,void *ref){
    cb->buffer = ref;
    if(cb->buffer == NULL){
        // handle error
        }
    cb->buffer_end = (char *)cb->buffer + capacity * sz;
    cb->capacity = capacity;
    cb->count 	= 0;
    cb->count2 	= 0;
    cb->sz 			= sz;
    cb->head 		= cb->buffer;
    cb->tail2 	= cb->buffer;
    cb->tail 		= cb->buffer;
		cb->top  		= cb->buffer;
}


int str_f_to_steps(const char *str, uint16_t steps_per_unit, char **endptr)
{
    uint8_t ten = 0;
    fixedpt t824 = 0;
    uint32_t number = 0;
    bool negative = false;
    bool fract = false;
    char c;
    while ((c = *str) != 0) {
        if (c >= '0' && c <= '9')   {
            if(fract==true){
                number = number * 10 + (c - '0');
            } else{
                if(ten<3){
                    number = number * 10 + (c - '0');
                }
                ten++;
            }
        } 
        else if (c == '-')  {
            negative = true;
        }
        else if (c == '.') {
            t824 = number * steps_per_unit;
            number = 0;
//          ten = 0;
            fract = true;
        }   
        str++;
    }
    if (endptr != 0) *endptr = (char *)str;

    switch(ten){
        case 1:{ // if only one fract didgit in g-code
            number *= 100;
            break;
        }
        case 2:{// if only two fract didgits in g-code
            number *= 10;
            break;
        }
    }

    t824 += ((number * steps_per_unit * 4914) >> 22); // fast divide by  1000
    if (negative) return -t824;
    else return t824;
}

/**
 * \brief convert value mm/rev in Q824 format to delay according to stepper per rev,
 * 	lead screw pitch and encoder resolution
 * \param[in] 
 *
 * \return 
 */
fixedptu str_f824mm_rev_to_delay824(fixedptu feed){
/* 1800enc lines*2(2x mode) / (400steps p mm / 1mm lead screw * current_code.feed ) << 24 to q824
		1800*2/(400/1*feed)<<24 = 3600/(400feed)<<24=9<<24/feed
*/
	return fixedptu_div(9<<24,feed);
}


/**
 * \brief convert value mm/min in Qxxx format to delay in Q824
 * \param[in] 
 *
 * \return 
 */
	fixedptu str_f824mm_min_to_delay824(fixedptu feed){
	fixedptu f = fixedpt_xdiv2210(hzminps_z, feed);
	f = f << 14; // translate to 8.24 format used for delays
	return f;
}



int str_f_to_824(char *line, uint8_t *char_counter){
  char *str = line + *char_counter;

//fixedptu strto824(char *str) //, char **endptr)
//{
	uint32_t ten = 0;
	fixedpt t824 = 0;
	fixedptu number = 0;
	bool negative = false;
	bool fract = false;
	char c;
	while ((c = *str) != 0) {
		if (c >= '0' && c <= '9') {
			if(fract==false){
					number = number * 10 + (c - '0');
			} else{
					if(ten<3){
							number = number * 10 + (c - '0');
					}
					ten++;
			}
		} else if (c == '-')  {
				negative = true;
		}
		else if (c == '.') {
			t824 = fixedpt_fromint(number);
			fract = true;
			ten = 0;
			number = 0;
		} else break;
		str++;
	}

//    if (endptr != 0) *endptr = (char *)str;
	if(fract == true){
		switch(ten){
				case 1:{
						number *= 1677721; // div 10
						break;
				}
				case 2:{
						number *= 167772; // div 100
						break;
				}
				case 3:{
						number *= 16777; // div 1000
						break;
				}
		}
		t824 |= number;//fixedpt_xdiv(number,ten);
	} else {
		t824 = fixedpt_fromint(number);
	}

	*char_counter = str - line - 1; // Set char_counter to next statement
	if (negative) return -t824;
	else return t824;
}


int str_f_to_2210(char *line, uint8_t *char_counter){
  char *str = line + *char_counter;
	uint32_t ten = 0;
	fixedpt t2210 = 0;
	fixedptu number = 0;
	bool negative = false;
	bool fract = false;
	char c;
	while ((c = *str) != 0) {
		if (c >= '0' && c <= '9') {
			if(fract==false){
					number = number * 10 + (c - '0');
			} else{
					if(ten<3){
							number = number * 10 + (c - '0');
					}
					ten++;
			}
		} else if (c == '-')  {
				negative = true;
		}
		else if (c == '-')  {
				negative = true;
		}
		else if (c == '.') {
				fract = true;
				t2210 = fixedpt_fromint2210(number);
				ten = 0;
				number = 0;
		} else break;   
		str++;
	}

	if(fract == true) {
		switch(ten){
				case 1:{
						number *= 1677721; // div 10
						break;
				}
				case 2:{
						number *= 167772; // div 100
						break;
				}
				case 3:{
						number *= 16777; // div 1000
						break;
				}
		}
		number >>=14;
		t2210 |= number;//fixedpt_xdiv(number,ten);
	} else {
		t2210 = fixedpt_fromint2210(number);
	}

	if (negative) return -t2210;
	else return t2210;
}



//int str_f_inch_to_steps2210(const char *str, char **endptr){
int str_f_inch_to_steps2210(char *line, uint8_t *char_counter){
  Error_Handler(); // todo trap, rewrite
	char *str = line + *char_counter;

	uint8_t ten = 0;
	fixedpt t2210 = 0;
	uint32_t number = 0;
	bool negative = false;
	bool fract = false;
	char c;
	while ((c = *str) != 0) {
		if (c >= '0' && c <= '9')   {
			if(fract==false){
				number = number * 10 + (c - '0');
			} else{
				if(ten<4){
						number = number * 10 + (c - '0');
				}
				ten++;
			}
		} 
		else if (c == '-')  {
			negative = true;
		}
		else if (c == '.') {
			t2210 = number * steps_per_inch_Z_2210; //steps_per_unit_Z_2210 already in 2210 format
			number = 0;
			fract = true;
		}   
		str++;
	}
//    if (endptr != 0) *endptr = (char *)str;

	*char_counter = str - line - 1; // Set char_counter to next statement
	if (fract == true){
    switch(ten){
			case 1:{ // if only one fract didgit in g-code
				number *= 1000;
				break;
			}
			case 2:{// if only two fract didgits in g-code
				number *= 100;
				break;
			}
			case 3:{// if only three fract didgits in g-code
				number *= 10;
				break;
			}
    }
    /* some explanations:
    number contains fract part*10000, ie when 0.9999inch = 9999.
    translate in to 25.4*10, = 9999*254
    next mul to steps per mm(400) and pack it into 2210 by multiplying to 1024.
    and divide to 100000 to get aling fract and fixed parts
    so we have 9999*254*400*1024/100000
    we cannot just calculate this because of long (9999*254*400*1024 = 10402799616 is greater then 2^32= 4294967296)
    but 254*400*1024/100000 equal 127*400*1024/50000 and equal 127*4*512/250 and equal 127*1024/125
    in this case 9999*127*1024 = 1300349952, this value is fitted to uint32 without loosing speed 
    
    For other cases(steps per mm) it should be done the same way.
    Also its possible to find some fraction with smaller numerator to get result with little more error
    in my case 1024*127/125=1040,384
    8323/8=1040,375. That's add acceptable 0.0002mm error and its possible to avoid division at all
    */
//  t2210 += ((number<<10)*127/125);
    t2210 += ((number*8323)>>3);
	} else {
		t2210 = number; //steps_per_unit_Z_2210 already in 2210 format
	}
    if (negative) return -t2210;
    else return t2210;
}



int str_f_to_steps2210(char *line, uint8_t *char_counter){
  char *str = line + *char_counter;

	uint8_t ten = 0;
	fixedpt t2210 = 0;
	uint32_t number = 0;
	bool negative = false;
	bool fract = false;
	char c;
	while ((c = *str) != 0) {
		if (c >= '0' && c <= '9')   {
				if(fract==false){
						number = number * 10 + (c - '0');
				} else{
						if(ten<3){
								number = number * 10 + (c - '0');
						}
						ten++;
				}
		}	else if (c == '-')  {
				negative = true;
		}	else if (c == '.') {
				t2210 = number * steps_per_unit_Z_2210; //steps_per_unit_Z_2210 already in 2210 format
				number = 0;
				fract = true;
		} else if (c == ' '){ //skip blanks
		} else break;   
		str++;
	}
//    if (endptr != 0) *endptr = (char *)str;
	*char_counter = str - line - 1; // Set char_counter to next statement
	if (fract == true){
		switch(ten){
			case 1:{ // if only one fract didgit in g-code
				number *= 100;
				break;
			}
			case 2:{// if only two fract didgits in g-code
				number *= 10;
				break;
			}
		}

		// for 1mm pitch leadscrew:
//		t2210 += ((number << 12) / 10); //		t2210 += ((number << 10) * 400 / 1000); // 

		t2210 += (number * steps_per_unit_Z_2210) / 1000;
//		t2210 += ((number * 419430) >> 10); // quick divide for 400 steps/mm, 400/1000 = 4/10, number<<10*4/10 = number<<12/10. 
	} else {
		t2210 = number * steps_per_unit_Z_2210; //steps_per_unit_Z_2210 already in 2210 format
	}
//  t2210 += (( number * steps_per_unit_Z_fract_2210 ) / 10 );
	if (negative) return -t2210;
	else return t2210;
}




/**
 * \brief    Fast Square root algorithm
 *
 * Fractional parts of the answer are discarded. That is:
 *      - SquareRoot(3) --> 1
 *      - SquareRoot(4) --> 2
 *      - SquareRoot(5) --> 2
 *      - SquareRoot(8) --> 2
 *      - SquareRoot(9) --> 3
 *
 * \param[in] a_nInput - unsigned integer for which to find the square root
 *
 * \return Integer square root of the input value.
 */
//__STATIC_INLINE 
uint32_t SquareRoot(uint32_t a_nInput){
	uint32_t op  = a_nInput;
	uint32_t res = 0;
	uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

  // "one" starts at the highest power of four <= than the argument.
  one >>= __clz(op) & ~0x3;
//    while (one > op) {
//		one >>= 2;
//	}

	while (one != 0){
		if (op >= res + one){
			op = op - (res + one);
			res = res +  2 * one;
		}
		res >>= 1;
		one >>= 2;
	}
	return res;
}



uint64_t SquareRoot64(uint64_t a_nInput){
	uint64_t op  = a_nInput;
	uint64_t res = 0;
	uint64_t one = 1uLL << 62; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

	op = a_nInput;
  // "one" starts at the highest power of four <= than the argument.
//  one >>= __clz(op) & ~0x3;
	while (one > op) {
		one >>= 2;
	}

	while (one != 0){
		if (op >= res + one){
			op = op - (res + one);
			res = res +  2 * one;
		}
		res >>= 1;
		one >>= 2;
	}
	return res;
}


/**
 * \brief    Fast Square root algorithm, with rounding
 *
 * This does arithmetic rounding of the result. That is, if the real answer
 * would have a fractional part of 0.5 or greater, the result is rounded up to
 * the next integer.
 *      - SquareRootRounded(2) --> 1
 *      - SquareRootRounded(3) --> 2
 *      - SquareRootRounded(4) --> 2
 *      - SquareRootRounded(6) --> 2
 *      - SquareRootRounded(7) --> 3
 *      - SquareRootRounded(8) --> 3
 *      - SquareRootRounded(9) --> 3
 *
 * \param[in] a_nInput - unsigned integer for which to find the square root
 *
 * \return Integer square root of the input value.
 */
uint32_t SquareRootRounded(uint32_t a_nInput)
{
	uint32_t op  = a_nInput;
	uint32_t res = 0;
	uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


	// "one" starts at the highest power of four <= than the argument.
//	while (one > op){
//		one >>= 2;
//	}
	one >>= __clz(op) & ~0x3;
	while (one != 0){
			if (op >= res + one){
					op = op - (res + one);
					res = res +  2 * one;
			}
			res >>= 1;
			one >>= 2;
	}

	/* Do arithmetic rounding to nearest integer */
	if (op > res){
			res++;
	}

	return res;
}
