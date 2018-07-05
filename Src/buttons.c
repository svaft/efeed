#include "buttons.h"


uint32_t buttons_flag_set __attribute__((at(0x20004000)));

uint32_t clk_mode = 10, buttons_flag_set_prev = 0;
BUTTON bt;

// реализация конечного автомата обработки событий кнопки
inline void process_button(void){
/*
click Nondeterministic finite automaton(NFA):
10. ждем сигнала с кнопки
20. кнопка нажата, считаем тики. если тиков > 1000 это лонг пресс, идем в 30
30. сигнал long_press_start, идем в 40
40. ждем отпуска кнопки, далее в 50
50. кнопку отпустили, если тиков меньше 200 идем в 70, иначе в 60
60. если тиков < 1000 генерим сигнал CLICK, если тиков больше генерим сигнал long_press_end, идем в 10
70. тиков меньше 200, это может быть дабл-клик, ждем еще 100, если ничего идем в 60, если клик идем в 80
80. ждем отпуска кнопки, далее в 90
90. кнопку отпустили, генерим DOUBLE_CLICK, идем в 10
*/

#if defined ( _SIMU )
//          uint32_t tmp_buttons = 0;
                uint32_t tmp_buttons = GPIOA->IDR & GPIO_PIN_8;
#else
                uint32_t tmp_buttons = GPIOA->IDR & GPIO_PIN_8;
#endif  

	
        if( tmp_buttons != bt.buttons ) { // start debounce
                        bt.buttons = tmp_buttons;
        // reset debounce counter and start count every one ms
                        bt.buttons_mstick = 1;
                        return;
                }

        if( bt.buttons_mstick > debounce ){
                switch(clk_mode){
                        case 10: {
                                if ( tmp_buttons & GPIO_PIN_8 ){    // released          
                                } else { // pressed
//                                          buttons_mstick = 1;
                                        clk_mode = 20;
                                }
                                break;
                        }
                        case 20: {
                                if ( tmp_buttons & GPIO_PIN_8 ){ // released
                                        clk_mode = 50;
                                } else {
                                        bt.downTime = bt.buttons_mstick;
                                }
                                if (bt.downTime > holdTime ){ // long press detected
                                        clk_mode = 30;
                                }
                                break;
                        }
                        case 30: { // long_press_start event
                                long_press_start = 1;
                                clk_mode = 40;
                                break;
                        }
                        case 40: {
                                if ( tmp_buttons & GPIO_PIN_8 ){ //released      
                                        clk_mode = 50;
                                } else {
                                        bt.downTime = bt.buttons_mstick;
                                }
                                break;
                        }
                        case 50: {
                                clk_mode = bt.downTime < clickTime ? 70 : 60;
                                break;
                        }
                        case 60: {//60 if tick count < 1000 generate CLICK event, else generate long_press_end event, go to 10 state
                                if(bt.downTime < holdTime) { //single CLICK event
                                        single_click = 1; 
                                } else { //  long_press_end event
                                        long_press_end = 1;
                                }
                                bt.downTime = bt.buttons_mstick = 0;
                                clk_mode = 10;
                                break;
                        }
                        case 70: { //70. тиков меньше 200, это может быть дабл-клик, ждем нажатия еще 100, если ничего идем в 60, если клик идем в 80
                                if ( tmp_buttons & GPIO_PIN_8 ){
                                        bt.downTime = bt.buttons_mstick;
                                        if( bt.downTime > DCgap ){
                                                clk_mode = 60;
                                        }
                                } else {
                                        clk_mode = 80;
                                }
                                break;
                        }
                        case 80: {
                                if ( tmp_buttons & GPIO_PIN_8 ){ // released                
                                        clk_mode = 90;
                                } else {
                                        bt.downTime = bt.buttons_mstick;
                                }
                                break;
                        }
                        case 90: { // сигнал DOUBLE_CLICK
                                double_click = 1;
                                clk_mode = 10;
                                bt.buttons_mstick = 0;
                                break;
                        }
                }
        }
}
