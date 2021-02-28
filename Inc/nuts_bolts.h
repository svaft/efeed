#ifndef nuts_bolts_h
#define nuts_bolts_h

#include "main.h"

#define CRC_BASE64_STRLEN 6

uint32_t Calculate_CRC(uint32_t len, uint8_t *bfr, int clear);
uint32_t atoui32(uint8_t* str);
uint32_t atoui64(uint8_t* str);

void ui10toa(uint32_t n, uint8_t s[]);
void ui64toa(uint32_t n, uint8_t s[]);
void sendResponce(uint32_t SrcAddress, uint32_t NbData);
__STATIC_INLINE bool is_crc_ok(uint8_t *bfr, uint8_t len){
	uint32_t crc = Calculate_CRC(len,bfr,1);
	uint32_t atoi = atoui64((uint8_t *)(bfr+len));
	if(crc == atoi)
		return true;
	else
		return false;

}

uint64_t SquareRoot64(uint64_t a_nInput);
uint32_t SquareRoot(uint32_t a_nInput);
uint32_t SquareRootRounded(uint32_t a_nInput);
int str_f_to_steps2210(char *str, uint8_t *char_counter);
int str_f_inch_to_steps2210(char *str, uint8_t *char_counter);
//int str_f_inch_to_steps2210(const char *str, char **endptr);
int str_f_to_824(char *line, uint8_t *char_counter);

int str_f_to_2210(char *line, uint8_t *char_counter);

fixedptu str_f824mm_rev_to_delay824(fixedptu feed); //mm/rev
//fixedptu str_f824mm_min_to_delay824(fixedptu feed); //mm/min
typedef struct circular_buffer{
    void *buffer;     // data buffer
    void *buffer_end; // end of data buffer
    size_t capacity;  // maximum number of items in the buffer
    size_t count;     // number of items in the buffer
    size_t count2;     // number of items in the buffer
    size_t sz;        // size of each item in the buffer
    void *head;       // pointer to head
		void *top;				// pointer to last added item
    void *tail;       // pointer to tail
    void *tail2;       // pointer to read-only tail
} circular_buffer;
extern circular_buffer gp_cb;
extern circular_buffer task_cb;
extern circular_buffer substep_cb;
extern circular_buffer task_precalc_cb;
extern circular_buffer substep_job_cb;
extern circular_buffer sma_cb;
extern circular_buffer sma_substep_cb;

void cb_init_ref(circular_buffer *cb, size_t capacity, size_t sz,void *ref);

__STATIC_INLINE void cb_init(circular_buffer *cb, size_t capacity, size_t sz){
    cb->buffer = malloc(capacity * sz);
    if(cb->buffer == NULL){
        // handle error
        }
    cb->buffer_end = (char *)cb->buffer + capacity * sz;
    cb->capacity = capacity;
    cb->count 	= 0;
    cb->count2 	= 0;
    cb->sz 			= sz;
    cb->head 		= cb->buffer;
    cb->tail 		= cb->buffer;
		cb->tail2 	= cb->buffer;
		cb->top  		= cb->buffer;
}

__STATIC_INLINE void cb_free(circular_buffer *cb){
    free(cb->buffer);
    // clear out other fields too, just to be safe
}

__STATIC_INLINE void sysFastMemCopy2( void *pDest,const void *pSrc )
{
    uint32_t *pLongSrc;
    uint32_t *pLongDest;
    // Convert byte addressing to long addressing
    pLongSrc = (uint32_t*) pSrc;
    pLongDest = (uint32_t*) pDest;
		*pLongDest = *pLongSrc;
}


__STATIC_INLINE void cb_push_back32(circular_buffer *cb, const void *item){
    if(cb->count == cb->capacity){
        Error_Handler();
            // handle error
    }
		sysFastMemCopy2(cb->head, item);
//    memcpy(cb->head, item, cb->sz);
		cb->top = cb->head;
    cb->head = (uint8_t *)cb->head + cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
}




__STATIC_INLINE void cb_push_back(circular_buffer *cb, const void *item){
    if(cb->count == cb->capacity){
        Error_Handler();
            // handle error
    }
    memcpy(cb->head, item, cb->sz);
		cb->top = cb->head;
    cb->head = (uint8_t *)cb->head + cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
    cb->count2++;
}

__STATIC_INLINE void* cb_iterate_back(circular_buffer *cb, int step){
	void *ref1 = (char*)cb->top - cb->sz*step;
	if (ref1 < cb->buffer) {
		char aa = (char*)cb->buffer - (char*)ref1;
		ref1 = (char*)cb->buffer_end - cb->sz*aa;
	}
	return ref1;
}

__STATIC_INLINE void cb_push_back_empty(circular_buffer *cb){
    if(cb->count == cb->capacity){
        Error_Handler();
            // handle error
    }
    memset(cb->head, 0, cb->sz);
		cb->top = cb->head;
    cb->head = (uint8_t *)cb->head + cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
    cb->count2++;
}

__STATIC_INLINE void cb_push_back_item(circular_buffer *cb, void *item){
    if(cb->count == cb->capacity){
        Error_Handler();
            // handle error
    }
//    memset(cb->head, 0, cb->sz);
    memcpy(cb->head, item, cb->sz);
		cb->top = cb->head;
    cb->head = (uint8_t *)cb->head + cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
    cb->count2++;
}


__STATIC_INLINE void cb_init_by_top(circular_buffer *cb, void *item){
    if(cb->count == 0){
        return;
    }
    memcpy(item, cb->top, cb->sz);
}
/**
  * @brief pop item from tail of buffer and free slot
*/
__STATIC_INLINE void cb_pop_front(circular_buffer *cb, void *item){
    if(cb->count == 0){
        return;
        // handle error
    }
    memcpy(item, cb->tail, cb->sz);
    if(cb->tail == cb->tail2)
			cb->count2--;
			
		cb->tail = (char*)cb->tail + cb->sz;
    if(cb->tail == cb->buffer_end)
        cb->tail = cb->buffer;
    cb->count--;
}

/**
  * @brief pop item by ref from tail of buffer and free slot
*/
__STATIC_INLINE void* cb_pop_front_ref(circular_buffer *cb){
    if(cb->count == 0){
        return 0;
        // handle error
    }
    void *ref = cb->tail; // get ref to stored value to return at the end and step to next
//    if(cb->tail == cb->tail2)
		if(cb->count2 >= cb->count)
			cb->count2--;
    cb->tail = (char*)cb->tail + cb->sz;
    if(cb->tail == cb->buffer_end) // if reach the end go to head of buffer
        cb->tail = cb->buffer;
    cb->count--;
    return ref;
}


__STATIC_INLINE void* cb_pop_front_ref2(circular_buffer *cb){
    if(cb->count2 == 0){
        return 0;
        // handle error
    }
    void *ref = cb->tail2;
    cb->tail2 = (char*)cb->tail2 + cb->sz;
    if(cb->tail2 == cb->buffer_end)
        cb->tail2 = cb->buffer;
    cb->count2--;
    return ref;
}



/**
  * @brief get item ref from tail of buffer, slot is locked untill cb_pop_front_ref 
*/
__STATIC_INLINE void* cb_get_front_ref(circular_buffer *cb){
    if(cb->count == 0){
        return 0;
        // handle error
    }
    return cb->tail; // get ref to stored value to return at the end and step to next
}

__STATIC_INLINE void* cb_step_back(circular_buffer *cb){
    if(cb->count == 0){
        return 0;
        // handle error
    }
		cb->head = cb->top;
    cb->top = (uint8_t *)cb->top - cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count--;
    return cb->head;
}




#endif
