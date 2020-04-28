#include "buttons.h"
#include "i2c_interface.h"


//uint32_t buttons_flag_set = 0;
uint32_t buttons_flag_set =0;//__attribute__((at(BB_VAR)));

uint32_t buttons_flag_set_prev = 0;
BUTTON bt[BT_TOTAL];

void init_buttons(void){

	bt[0].clk_mode = 10;
	bt[0].GPIOx = BUTTON_1_GPIO_Port;
	bt[0].button_pin = BUTTON_1_Pin;
	bt[0].buttons = bt[0].buttons_mask = LL_GPIO_IsInputPinSet(bt[0].GPIOx,bt[0].button_pin); //bt[0].GPIOx->IDR & bt[0].button_pin;
	return;
/*
	bt[1] = bt[0];
	bt[1].clk_mode = 10;
	bt[1].GPIOx = BUTTON_2_GPIO_Port;
	bt[1].button_pin = BUTTON_2_Pin;
	bt[1].buttons = bt[1].buttons_mask = bt[1].GPIOx->IDR & bt[1].button_pin;

	if(device_ready == 1){
//		read_sample_i2c(&i2c_device_logging.sample[i2c_device_logging.index]);		
		reqest_sample_i2c_dma();
//		while(ubTransferComplete == 0){
//		}
	}
	bt[2].clk_mode = 10;
	bt[2].button_pin = 0x02; // button_c code
	bt[2].buttons = bt[2].buttons_mask = dma_data[5]&bt[2].button_pin; // = bt[1].GPIOx->IDR & bt[1].button_pin;
	
	bt[3].clk_mode = 10;
	bt[3].button_pin = 0x01; // button_c code
	bt[3].buttons = bt[3].buttons_mask = dma_data[5]&bt[3].button_pin; // = bt[1].GPIOx->IDR & bt[1].button_pin;
*/
}
inline void process_joystick()
{
// to make sure ew really receive this data we need to set flag in interrapt in dma i2c RX dma handler 
//	sample->joy_x = dma_data[0];
//	sample->joy_y = data[1];
//	sample->accel_x = (data[2] << 2)|((data[5] >> 2) & 0x03) ;
//	sample->accel_y = (data[3] << 2)|((data[5] >> 4) & 0x03) ;
//	sample->accel_z = (data[4] << 2)|((data[5] >> 6) & 0x03) ;

	if(ubTransferComplete == 1){
		ubTransferComplete = 0;
//		uint8_t button_c=(dma_data[5]&0x02)>>1;
//		uint8_t button_z=dma_data[5]&0x01;
	}
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
 
//	#if defined ( _SIMU )
//		uint32_t tmp_buttons = bt[a].GPIOx->IDR & bt[a].button_pin;
//	#else
		uint32_t tmp_buttons;
		if(bt[a].GPIOx != 0)
			tmp_buttons = LL_GPIO_IsInputPinSet(bt[a].GPIOx,bt[a].button_pin);
//			tmp_buttons = bt[a].GPIOx->IDR & bt[a].button_pin; //BUTTON_1_GPIO_Port->IDR & bt[a].button_pin;
		else{
			
//			if(ubTransferComplete == 0)
//				continue;
			//	dma_delay = 0;
//	while(hi2c2->hdmarx->State != HAL_DMA_STATE_READY){
//		dma_delay++;
//		HAL_Delay(1);
//	}
//	dma_delay2 = dma_delay;

			tmp_buttons = dma_data[5] & bt[a].button_pin;
		}
//	#endif

		if( tmp_buttons != bt[a].buttons ) { // start debounce
			bt[a].buttons = tmp_buttons;
			// reset debounce counter and start count every one ms
			bt[a].buttons_mstick = 1;
			return;
		}
#ifdef _SIMU
	#define released 0
		#else
	#define released 1
#endif		
		
		if( bt[a].buttons_mstick > DEBOUNCE_MS ) {
			switch(bt[a].clk_mode) {
			case 10: {
				if ( tmp_buttons == released ) {   // released
				} else { // pressed
//					buttons_mstick = 1;
					bt[a].clk_mode = 20;
				}
				break;
			}
			case 20: {
				if ( tmp_buttons == released ) { // released
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
				buttons_flag_setbb[(a<<2)+long_press_start_Pos]  = 1; //long_press_start = 1;
				bt[a].clk_mode = 40;
				break;
			}
			case 40: {
				if ( tmp_buttons == released ) { //released
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
					buttons_flag_setbb[(a<<2)+single_click_Pos]  = 1; //single_click = 1;
				} else { //  long_press_end event
					buttons_flag_setbb[(a<<2)+long_press_end_Pos]  = 1; //long_press_end = 1;
				}
				bt[a].downTime = bt[a].buttons_mstick = 0;
				bt[a].clk_mode = 10;
				break;
			}
			case 70: { //70. тиков меньше 200, это может быть дабл-клик, ждем нажатия еще 100, если ничего идем в 60, если клик идем в 80
				if ( tmp_buttons == released ) {
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
				if ( tmp_buttons == released ) { // released
					bt[a].clk_mode = 90;
				} else {
					bt[a].downTime = bt[a].buttons_mstick;
				}
				break;
			}
			case 90: { // сигнал DOUBLE_CLICK
				buttons_flag_setbb[(a<<2)+double_click_Pos]  = 1; //double_click = 1;
				bt[a].clk_mode = 10;
				bt[a].buttons_mstick = 0;
				break;
			}
			}
		}
	}
//	ubTransferComplete = 0;
}
