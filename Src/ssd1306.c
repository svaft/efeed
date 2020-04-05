/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#include "ssd1306.h"

I2C_TypeDef *hi2c_screen;

//extern I2C_HandleTypeDef hi2c2;

/* Write command */
#define SSD1306_WRITECOMMAND(command)      ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, (command))
/* Write data */
#define SSD1306_WRITEDATA(data)            ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x40, (data))
/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer_all[SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1], *SSD1306_Buffer = SSD1306_Buffer_all + 1;

/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

/* Private variable */
static SSD1306_t SSD1306;

uint8_t SSD1306_Init(I2C_TypeDef *hi2c) {

	hi2c_screen = hi2c;
	/* Init I2C */
//	ssd1306_I2C_Init();
	/* A little delay */
	uint32_t p = 2500;
	while(p>0)
		p--;
	
	/* Init LCD */
	SSD1306_WRITECOMMAND(0xAE); //display off
	SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode   
	SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	SSD1306_WRITECOMMAND(0x00); //---set low column address
	SSD1306_WRITECOMMAND(0x10); //---set high column address
	SSD1306_WRITECOMMAND(0x40); //--set start line address
	SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	SSD1306_WRITECOMMAND(0xFF);
	SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	SSD1306_WRITECOMMAND(0xA6); //--set normal display
	SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	SSD1306_WRITECOMMAND(0x3F); //
	SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WRITECOMMAND(0xD3); //-set display offset
	SSD1306_WRITECOMMAND(0x00); //-not offset
	SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	SSD1306_WRITECOMMAND(0x22); //
	SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	SSD1306_WRITECOMMAND(0x14); //
	SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel
	
	/* Clear screen */
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	
	/* Update screen */
	SSD1306_UpdateScreen();
	
	/* Set default values */
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	
	/* Initialized OK */
	SSD1306.Initialized = 1;
	
	/* Return OK */
	return 1;
}

void SSD1306_ToggleInvert(void) {
	uint16_t i;
	
	/* Toggle invert */
	SSD1306.Inverted = !SSD1306.Inverted;
	
	/* Do memory toggle */
	for (i = 0; i < sizeof(SSD1306_Buffer); i++) {
		SSD1306_Buffer[i] = ~SSD1306_Buffer[i];
	}
}

void SSD1306_Fill(uint8_t color) {
	/* Set memory */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, SSD1306_WIDTH * SSD1306_HEIGHT / 8);
}

void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (uint8_t)!color;
	}
	
	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void SSD1306_GotoXY(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void SSD1306_Putc2big(char ch, const FONT_INFO* Font) {
	char ch_index = ch - Font->startChar;
	uint16_t offset =  Font->charInfo[ch_index].offset;
	uint16_t x = SSD1306.CurrentX, y = SSD1306.CurrentY / 8;
	for (int h = y; h < Font->heightPages + y; h++ ){
		for (int w = x; w < Font->charInfo[ch_index].widthBits+x; w++ ){
			SSD1306_Buffer[w + h * SSD1306_WIDTH] = Font->data[offset++];
		}
	}
	SSD1306.CurrentX += Font->charInfo[ch_index].widthBits + 1;
}

void SSD1306_Putc2bigInv(char ch, const FONT_INFO* Font) {
	char ch_index = ch - Font->startChar;
	uint16_t offset =  Font->charInfo[ch_index].offset;
	uint16_t x = SSD1306.CurrentX, y = SSD1306.CurrentY / 8;
	for (int h = y; h < Font->heightPages + y; h++ ){
		for (int w = x; w < Font->charInfo[ch_index].widthBits+x; w++ ){
			SSD1306_Buffer[w + h * SSD1306_WIDTH] = !Font->data[offset++];
		}
	}
	SSD1306.CurrentX += Font->charInfo[ch_index].widthBits + 1;
}

char SSD1306_Puts2(char* str, const FONT_INFO* Font, uint8_t color) {
	/* Write characters */
	if(color){
		while (*str) {
			/* Write character by character */
			SSD1306_Putc2big(*str, Font);
			str++;
		}
	} else {
		while (*str) {
			/* Write character by character */
			SSD1306_Putc2bigInv(*str, Font);
			str++;
		}
	}

	/* Everything OK, zero should be returned */
	return *str;
}

char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color) {
	uint32_t i, b, j;
	
	b = 0;
	
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (uint8_t) color);
			} else {
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (uint8_t) !color);
			}
		}
	}
	/* Increase pointer */
	SSD1306.CurrentX += Font->FontWidth;
	
	/* Return character written */
	return ch;
}

char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}
		
		/* Increase string pointer */
		str++;
	}
	
	/* Everything OK, zero should be returned */
	return *str;
}
 

void ssd1306_I2C_Init() {
	//MX_I2C1_Init();
//	Activate_I2C_Master();

	uint32_t p = 250000;
	while(p>0)
		p--;
	//HAL_I2C_DeInit(hi2c_screen);
	//p = 250000;
	//while(p>0)
	//	p--;
	//MX_I2C1_Init();
}
/*
void ssd1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	uint8_t dt[count + 1];
	dt[0] = reg;
	uint8_t i;
	for(i = 1; i <= count; i++)
		dt[i] = data[i-1];
//	Handle_I2C_Master(hi2c_screen, address, dt, count, 10);
	// haltodo HAL_I2C_Master_Transmit(hi2c_screen, address, dt, count, 10);
}


void ssd1306_I2C_WriteMulti_DMA(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {	
	//haltodo HAL_I2C_Master_Transmit(hi2c_screen, address, &reg, 1, 100);
	//haltodo HAL_I2C_Master_Transmit_DMA(hi2c_screen, address, data, count);
}
*/

void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data) {
	uint8_t dt[2] = {reg, data};
//	dt[0] = reg;
//	dt[1] = data;
	Handle_I2C_MasterDMA_IT(hi2c_screen, address, dt, 2, 10);
//	HAL_I2C_Master_Transmit(hi2c_screen, address, dt, 2, 10);
}

void SSD1306_UpdateScreen(void) {
	SSD1306_Buffer_all[0] = 0x40;
//	HAL_I2C_Master_Transmit(hi2c_screen, SSD1306_I2C_ADDR, SSD1306_Buffer_all, SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1, 100);
	Handle_I2C_MasterDMA_IT_async(SSD1306_I2C_ADDR, SSD1306_Buffer_all, SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1);
//	Handle_I2C_MasterDMA_IT(hi2c_screen, SSD1306_I2C_ADDR, SSD1306_Buffer_all, SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1, 100);
//  if(LL_I2C_IsActiveFlag_SB(I2C2)) {

	/* haltodo
	if(hi2c_screen->hdmatx->State == HAL_DMA_STATE_READY)
		HAL_I2C_Master_Transmit_DMA(hi2c_screen, SSD1306_I2C_ADDR, SSD1306_Buffer_all, SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1);
//	HAL_I2C_Master_Transmit(hi2c_screen, SSD1306_I2C_ADDR, SSD1306_Buffer_all, SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1, 100);
//	while(HAL_DMA_GetState(hi2c_screen->hdmatx) != HAL_DMA_STATE_READY)
//	{
//		HAL_Delay(1); //Change for your RTOS
//	}
	*/
//	}
}
