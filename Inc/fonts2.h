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
#ifndef FONTS2_H
#define FONTS2_H 130

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include "stm32f1xx_hal.h"
#include "string.h"

	// This structure describes a single character's display information
typedef struct
{
	const uint8_t widthBits;					// width, in bits (or pixels), of the character
	const uint8_t heightBits;					// width, in bits (or pixels), of the character
	const uint16_t offset;					// offset of the character's bitmap, in bytes, into the the FONT_INFO's data array
	
} FONT_CHAR_INFO;	

// Describes a single font
typedef struct
{
	const uint8_t 			heightPages;	// height, in pages (8 pixels), of the font's characters
	const uint8_t 			startChar;		// the first character in the font (e.g. in charInfo and data)
	const uint8_t 			endChar;		// the last character in the font
	const uint8_t			spacePixels;	// number of pixels that a space character takes up
	const FONT_CHAR_INFO*	charInfo;		// pointer to array of char information
	const uint8_t*			data;			// pointer to generated array of character visual representation
		
} FONT_INFO;	


/* Font data for Microsoft Sans Serif 12pt */
extern const unsigned char microsoftSansSerif_12ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_12ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_12ptDescriptors[];



/* Font data for Microsoft Sans Serif 46pt */
extern const unsigned char microsoftSansSerif_46ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_46ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_46ptDescriptors[];



/* Font data for Consolas 18pt */
extern const unsigned char consolas_18ptBitmaps[];
extern const FONT_INFO consolas_18ptFontInfo;
extern const FONT_CHAR_INFO consolas_18ptDescriptors[];





/* C++ detection */
#ifdef __cplusplus
}
#endif

 
#endif
