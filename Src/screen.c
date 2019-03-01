#include "screen.h"
#include "ssd1306.h"
#include "i2c_interface.h"

extern const THREAD_INFO Thread_Info[];
extern uint8_t Menu_Step;																					// выборка из массива по умолчанию (1.5mm)
extern bool feed_direction;
extern __IO uint8_t  ubTransferComplete;

void init_screen(I2C_TypeDef *hi2c){
	SSD1306_Init(hi2c);
}


char * utoa_builtin_div_1(uint32_t value, char *buffer)
{
	buffer += 11;
// 11 байт достаточно для десятичного представления 32-х байтного числа и завершающего нуля
	*--buffer = 0;
	do {
		*--buffer = value % 10 + '0';
		value /= 10;
	} while (value != 0);
	return buffer;
}


int update_screen(void){
#ifndef _SIMU
	if(ubTransferComplete == 0) {
		return 1;
	}
	SSD1306_Fill(SSD1306_COLOR_BLACK);
// first line
	SSD1306_GotoXY(0, 16*0);
	feed_direction == feed_direction_left ? SSD1306_Putc2big(left_arrow, &consolas_18ptFontInfo) : SSD1306_Putc2big(right_arrow, &consolas_18ptFontInfo);
	SSD1306_Puts2((char *)Thread_Info[Menu_Step].Unit, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
	SSD1306_Puts2((char *)Thread_Info[Menu_Step].infeed_inch, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // infeed recommendation

	char text_buffer[11];

	SSD1306_GotoXY(SSD1306_WIDTH - 16, 0);
//	SSD1306_Puts2(utoa_builtin_div_1(z_axis.mode, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode

	SSD1306_GotoXY(SSD1306_WIDTH - 60, 16);
//	SSD1306_Puts2(utoa_builtin_div_1(z_axis.current_pos, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode

//	SSD1306_GotoXY(SSD1306_WIDTH - 60, 32);
//	SSD1306_Puts2(utoa_builtin_div_1(z_axis.ramp_step, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode
	if(i2c_device_logging.sample[i2c_device_logging.index].button_c > 0){
		SSD1306_GotoXY(SSD1306_WIDTH - 60, 32);
		SSD1306_Puts2(utoa_builtin_div_1(i2c_device_logging.sample[i2c_device_logging.index].button_c, text_buffer), &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE); // DKA mode
	}

// second line

	SSD1306_GotoXY(0, 16*1); //Устанавливаем курсор в позицию 0;16. Сначала по горизонтали, потом вертикали.
	SSD1306_Puts2((char *)Thread_Info[Menu_Step].Text, &microsoftSansSerif_20ptFontInfo, SSD1306_COLOR_WHITE);
	//			SSD1306_GotoXY(50, 16*1);
	//			SSD1306_Puts2(Thread_Info[Menu_Step].infeed_mm, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
	//			SSD1306_GotoXY(50, 16*2);
	//			SSD1306_Puts2(Thread_Info[Menu_Step].infeed_inch, &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY(0, 16*3);
	switch(Thread_Info[Menu_Step].infeed_strategy) {
	case 0:
		SSD1306_Puts2("radial", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
		break;
	case 1:
		SSD1306_Puts2("flank", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
		break;
	case 2:
		SSD1306_Puts2("incremental", &microsoftSansSerif_12ptFontInfo, SSD1306_COLOR_WHITE);
		break;
	}


/*
	if(auto_mode == true) {
		SSD1306_GotoXY(SSD1306_WIDTH - 32, 0);
		SSD1306_Putc2big('A', &microsoftSansSerif_12ptFontInfo);
//  SSD1306_Putc2big(auto_symbol, &consolas_18ptFontInfo);
	}
*/
#endif /* _SIMU */
	return 0;
}
