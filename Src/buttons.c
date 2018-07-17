#include "buttons.h"


//uint32_t buttons_flag_set = 0;
uint32_t buttons_flag_set __attribute__((at(BB_VAR)));

uint32_t buttons_flag_set_prev = 0;
BUTTON bt[BT_TOTAL];


void init_buttons(void){
	bt[0].clk_mode = 10;
	bt[0].GPIOx = BUTTON_1_GPIO_Port;
	bt[0].button_pin = BUTTON_1_Pin;
	bt[0].buttons = bt[0].buttons_mask = bt[0].GPIOx->IDR & bt[0].button_pin;
	
	bt[1].clk_mode = 10;
	bt[1].GPIOx = BUTTON_2_GPIO_Port;
	bt[1].button_pin = BUTTON_2_Pin;
	bt[1].buttons = bt[1].buttons_mask = bt[1].GPIOx->IDR & bt[1].button_pin;

}

// реализация конечного автомата обработки событий кнопки
inline void process_button()
{
	for(int a =0; a<BT_TOTAL;a++){
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
		uint32_t tmp_buttons = bt[a].GPIOx->IDR & bt[a].button_pin;
	#else
		uint32_t tmp_buttons = bt[a].GPIOx->IDR & bt[a].button_pin; //BUTTON_1_GPIO_Port->IDR & bt[a].button_pin;
	#endif


		if( tmp_buttons != bt[a].buttons ) { // start debounce
			bt[a].buttons = tmp_buttons;
			// reset debounce counter and start count every one ms
			bt[a].buttons_mstick = 1;
			return;
		}

		if( bt[a].buttons_mstick > DEBOUNCE_MS ) {
			switch(bt[a].clk_mode) {
			case 10: {
				if ( tmp_buttons & bt[a].button_pin ) {   // released
				} else { // pressed
//					buttons_mstick = 1;
					bt[a].clk_mode = 20;
				}
				break;
			}
			case 20: {
				if ( tmp_buttons & bt[a].button_pin ) { // released
					bt[a].clk_mode = 50;
				} else {
					bt[a].downTime = bt[a].buttons_mstick;
				}
				if (bt[a].downTime > HOLDTIME_MS ) { // long press detected
					bt[a].clk_mode = 30;
				}
				break;
			}
			case 30: { // long_press_start event
				long_press_start = 1;
				bt[a].clk_mode = 40;
				break;
			}
			case 40: {
				if ( tmp_buttons & bt[a].button_pin ) { //released
					bt[a].clk_mode = 50;
				} else {
					bt[a].downTime = bt[a].buttons_mstick;
				}
				break;
			}
			case 50: {
				bt[a].clk_mode = bt[a].downTime < CLICKTIME_MS ? 70 : 60;
				break;
			}
			case 60: {//60 if tick count < 1000 generate CLICK event, else generate long_press_end event, go to 10 state
				if(bt[a].downTime < HOLDTIME_MS) { //single CLICK event
					single_click = 1;
				} else { //  long_press_end event
					long_press_end = 1;
				}
				bt[a].downTime = bt[a].buttons_mstick = 0;
				bt[a].clk_mode = 10;
				break;
			}
			case 70: { //70. тиков меньше 200, это может быть дабл-клик, ждем нажатия еще 100, если ничего идем в 60, если клик идем в 80
				if ( tmp_buttons & bt[a].button_pin ) {
					bt[a].downTime = bt[a].buttons_mstick;
					if( bt[a].downTime > DOUBLECLICK_GAP_MS ) {
						bt[a].clk_mode = 60;
					}
				} else {
					bt[a].clk_mode = 80;
				}
				break;
			}
			case 80: {
				if ( tmp_buttons & bt[a].button_pin ) { // released
					bt[a].clk_mode = 90;
				} else {
					bt[a].downTime = bt[a].buttons_mstick;
				}
				break;
			}
			case 90: { // сигнал DOUBLE_CLICK
				double_click = 1;
				bt[a].clk_mode = 10;
				bt[a].buttons_mstick = 0;
				break;
			}
			}
		}
	}

}
