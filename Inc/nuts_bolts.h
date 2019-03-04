#ifndef nuts_bolts_h
#define nuts_bolts_h

#include "main.h"

uint64_t SquareRoot64(uint64_t a_nInput);
uint32_t SquareRoot(uint32_t a_nInput);
uint32_t SquareRootRounded(uint32_t a_nInput);
int str_f_to_steps2210(char *str, uint8_t *char_counter);
int str_f_inch_to_steps2210(const char *str, char **endptr);

typedef struct circular_buffer{
    void *buffer;     // data buffer
    void *buffer_end; // end of data buffer
    size_t capacity;  // maximum number of items in the buffer
    size_t count;     // number of items in the buffer
    size_t sz;        // size of each item in the buffer
    void *head;       // pointer to head
		void *top;				// pointer to last added item
    void *tail;       // pointer to tail
} circular_buffer;
extern circular_buffer gp_cb;
extern circular_buffer task_cb;
void cb_init_ref(circular_buffer *cb, size_t capacity, size_t sz,void *ref);

__STATIC_INLINE void cb_init(circular_buffer *cb, size_t capacity, size_t sz){
    cb->buffer = malloc(capacity * sz);
    if(cb->buffer == NULL){
        // handle error
        }
    cb->buffer_end = (char *)cb->buffer + capacity * sz;
    cb->capacity = capacity;
    cb->count = 0;
    cb->sz = sz;
    cb->head = cb->buffer;
    cb->tail = cb->buffer;
		cb->top  = cb->buffer;
}

__STATIC_INLINE void cb_free(circular_buffer *cb){
    free(cb->buffer);
    // clear out other fields too, just to be safe
}

__STATIC_INLINE void cb_push_back(circular_buffer *cb, const void *item){
    if(cb->count == cb->capacity){
        while(1){}
            // handle error
    }
    memcpy(cb->head, item, cb->sz);
		cb->top = cb->head;
    cb->head = (uint8_t *)cb->head + cb->sz;
    if(cb->head == cb->buffer_end)
        cb->head = cb->buffer;
    cb->count++;
}

__STATIC_INLINE void cb_init_by_top(circular_buffer *cb, void *item){
    if(cb->count == 0){
        return;
    }
    memcpy(item, cb->top, cb->sz);
}

__STATIC_INLINE void cb_pop_front(circular_buffer *cb, void *item){
    if(cb->count == 0){
        return;
        // handle error
    }
    memcpy(item, cb->tail, cb->sz);
    cb->tail = (char*)cb->tail + cb->sz;
    if(cb->tail == cb->buffer_end)
        cb->tail = cb->buffer;
    cb->count--;
}

__STATIC_INLINE void* cb_pop_front_ref(circular_buffer *cb){
    if(cb->count == 0){
        return 0;
        // handle error
    }
    void *ref = cb->tail;
//  memcpy(item, cb->tail, cb->sz);
    cb->tail = (char*)cb->tail + cb->sz;
    if(cb->tail == cb->buffer_end)
        cb->tail = cb->buffer;
    cb->count--;
    return ref;
}

#endif
