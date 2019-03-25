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
#include "fonts.h"
/*
const uint8_t Font7x10[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x87, 0xe3, 0xe3, 0x70, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x15, 0x45, 0x45, 0x41, 0x51, 0x54, 0x00, 0x00, 0x00,  //
0x00, 0x89, 0xc4, 0x23, 0x91, 0xf0, 0x20, 0x10, 0x08, 0x00,  //
0x00, 0x87, 0xc0, 0x20, 0xd0, 0x21, 0x30, 0x08, 0x04, 0x00,  //
0x00, 0x8e, 0x40, 0xc0, 0xe1, 0x90, 0x38, 0x24, 0x12, 0x00,  //
0x00, 0x81, 0x40, 0xe0, 0xe1, 0x11, 0x38, 0x04, 0x02, 0x00,  //
0x00, 0x0e, 0x85, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x02, 0xe1, 0x43, 0x20, 0x7c, 0x00, 0x00, 0x00,  //
0x00, 0x89, 0xc5, 0xa2, 0x91, 0x10, 0x08, 0x04, 0x1e, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x08, 0x04, 0x02, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0xf0, 0x40, 0x20, 0x10, 0x08, 0x04, 0x00,  //
0x00, 0x00, 0x00, 0x80, 0x43, 0x20, 0x10, 0x08, 0x04, 0x00,  //
0x08, 0x04, 0x02, 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x08, 0x04, 0x02, 0xf1, 0x43, 0x20, 0x10, 0x08, 0x04, 0x00,  //
0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00,  //
0x08, 0x04, 0x02, 0x81, 0x43, 0x20, 0x10, 0x08, 0x04, 0x00,  //
0x08, 0x04, 0x02, 0xf1, 0x40, 0x20, 0x10, 0x08, 0x04, 0x00,  //
0x08, 0x04, 0x02, 0xf1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0xf0, 0x43, 0x20, 0x10, 0x08, 0x04, 0x00,  //
0x08, 0x04, 0x02, 0x81, 0x40, 0x20, 0x10, 0x08, 0x04, 0x00,  //
0x00, 0x10, 0x84, 0x20, 0x20, 0x40, 0x40, 0x3e, 0x00, 0x00,  //
0x00, 0x01, 0x01, 0x02, 0x82, 0x10, 0x04, 0x3e, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x47, 0xa1, 0x50, 0x28, 0x00, 0x00, 0x00,  //
0x00, 0x10, 0xc4, 0x87, 0xf0, 0x11, 0x04, 0x00, 0x00, 0x00,  //
0x00, 0x0c, 0x89, 0xe0, 0x21, 0x70, 0x6c, 0x04, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x02, 0x81, 0x40, 0x00, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x0a, 0x85, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x0a, 0xc5, 0x47, 0xf1, 0x51, 0x28, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x47, 0xc1, 0x41, 0x71, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x92, 0x8a, 0x82, 0xa0, 0xa8, 0x24, 0x00, 0x00, 0x00,  //
0x00, 0x82, 0x42, 0x41, 0x50, 0x49, 0x58, 0x00, 0x00, 0x00,  //
0x00, 0x0c, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x08, 0x82, 0x40, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00,  //
0x00, 0x02, 0x02, 0x02, 0x81, 0x20, 0x08, 0x00, 0x00, 0x00,  //
0x00, 0x80, 0x88, 0xe2, 0xa3, 0x88, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x02, 0xe1, 0x43, 0x20, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x10, 0x04, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x38, 0x08, 0x00, 0x00,  //
0x00, 0x10, 0x08, 0x82, 0x20, 0x08, 0x04, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x45, 0x24, 0x12, 0x51, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x43, 0x81, 0x40, 0x20, 0x7c, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x08, 0x84, 0x21, 0x08, 0x7c, 0x00, 0x00, 0x00,  //
0x00, 0x1f, 0x08, 0x82, 0x01, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x08, 0x86, 0x22, 0xf1, 0x41, 0x20, 0x00, 0x00, 0x00,  //
0x00, 0x9f, 0x40, 0x63, 0x02, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x0c, 0x41, 0xa0, 0x31, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x1f, 0x08, 0x02, 0x41, 0x10, 0x08, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0xc4, 0x11, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0xc6, 0x02, 0x41, 0x18, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x82, 0x83, 0x00, 0x20, 0x38, 0x08, 0x00, 0x00,  //
0x00, 0x00, 0x82, 0x83, 0x00, 0x60, 0x10, 0x04, 0x00, 0x00,  //
0x00, 0x10, 0x04, 0x41, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x07, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x01, 0x01, 0x01, 0x41, 0x10, 0x04, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x08, 0x82, 0x40, 0x00, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0xa6, 0xd2, 0x08, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x45, 0x24, 0xf2, 0x89, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x0f, 0x89, 0xc4, 0x21, 0x91, 0x3c, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0x20, 0x10, 0x88, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x0f, 0x89, 0x44, 0x22, 0x91, 0x3c, 0x00, 0x00, 0x00,  //
0x00, 0x9f, 0x40, 0xe0, 0x11, 0x08, 0x7c, 0x00, 0x00, 0x00,  //
0x00, 0x9f, 0x40, 0xe0, 0x11, 0x08, 0x04, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0x20, 0x90, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0x48, 0xe4, 0x13, 0x89, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x0e, 0x02, 0x81, 0x40, 0x20, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x1c, 0x08, 0x04, 0x02, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0x44, 0x61, 0x50, 0x48, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x81, 0x40, 0x20, 0x10, 0x08, 0x7c, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0xc8, 0xa6, 0x12, 0x89, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0xc8, 0xa4, 0x92, 0x89, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0x24, 0x12, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x8f, 0x48, 0xe4, 0x11, 0x08, 0x04, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0x24, 0x12, 0xa9, 0x38, 0x20, 0x00, 0x00,  //
0x00, 0x8f, 0x48, 0xe4, 0x51, 0x48, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x48, 0xc0, 0x01, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x1f, 0x02, 0x81, 0x40, 0x20, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0x48, 0x24, 0x12, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0x48, 0x44, 0xa1, 0x50, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0x48, 0xa4, 0x52, 0xd9, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0x88, 0x82, 0xa0, 0x88, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x91, 0x88, 0x82, 0x40, 0x20, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x1f, 0x08, 0x82, 0x20, 0x08, 0x7c, 0x00, 0x00, 0x00,  //
0x00, 0x0e, 0x81, 0x40, 0x20, 0x10, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x81, 0x80, 0x80, 0x80, 0x80, 0x40, 0x00, 0x00, 0x00,  //
0x00, 0x0e, 0x04, 0x02, 0x81, 0x40, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x45, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00,  //
0x00, 0x06, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0x03, 0xe2, 0x89, 0x78, 0x00, 0x00, 0x00,  //
0x00, 0x81, 0x40, 0x63, 0x12, 0x99, 0x34, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0x23, 0x12, 0x88, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x10, 0x88, 0x25, 0x13, 0xc9, 0x58, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0x23, 0xf2, 0x09, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x0c, 0x89, 0xe0, 0x21, 0x10, 0x08, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0x25, 0x61, 0x08, 0x38, 0x22, 0x0e, 0x00,  //
0x00, 0x81, 0x40, 0x63, 0x12, 0x89, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x80, 0x81, 0x40, 0x20, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x08, 0x00, 0x03, 0x81, 0x40, 0x24, 0x12, 0x06, 0x00,  //
0x00, 0x81, 0x40, 0x24, 0x71, 0x48, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x06, 0x02, 0x81, 0x40, 0x20, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0xa2, 0x52, 0xa9, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x63, 0x12, 0x89, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0x23, 0x12, 0x89, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x63, 0x32, 0x69, 0x04, 0x02, 0x01, 0x00,  //
0x00, 0x00, 0x80, 0x25, 0x93, 0xb1, 0x40, 0x20, 0x10, 0x00,  //
0x00, 0x00, 0x40, 0x63, 0x12, 0x08, 0x04, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0x23, 0xe0, 0x80, 0x3c, 0x00, 0x00, 0x00,  //
0x00, 0x02, 0xc1, 0x43, 0x20, 0x90, 0x30, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x24, 0x12, 0xc9, 0x58, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x24, 0xa2, 0x50, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x24, 0x52, 0xa9, 0x28, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x44, 0x41, 0x50, 0x44, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x24, 0x92, 0xb1, 0x40, 0x22, 0x0e, 0x00,  //
0x00, 0x00, 0xc0, 0x07, 0x41, 0x10, 0x7c, 0x00, 0x00, 0x00,  //
0x00, 0x18, 0x02, 0xc2, 0x80, 0x20, 0x60, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x02, 0x81, 0x40, 0x20, 0x10, 0x00, 0x00, 0x00,  //
0x00, 0x03, 0x82, 0x80, 0x21, 0x20, 0x0c, 0x00, 0x00, 0x00,  //
0x00, 0x92, 0x4a, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x14, 0x80, 0x4f, 0xe0, 0x11, 0x08, 0x7c, 0x00, 0x00, 0x00,  //
0x1c, 0x51, 0xb2, 0x5a, 0xcc, 0x8a, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x04, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x0a, 0x80, 0x23, 0xf2, 0x09, 0x38, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //
0x00, 0x82, 0x22, 0x12, 0xf9, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x80, 0x47, 0x20, 0xf0, 0x88, 0x44, 0x1e, 0x00, 0x00, 0x00,  //
0x80, 0x47, 0x24, 0xf2, 0x88, 0x44, 0x1e, 0x00, 0x00, 0x00,  //
0x80, 0x4f, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00,  //
0x00, 0x0c, 0x85, 0x22, 0x91, 0x48, 0x3e, 0x11, 0x00, 0x00,  //
0x80, 0x4f, 0x20, 0xf0, 0x08, 0x04, 0x3e, 0x00, 0x00, 0x00,  //
0x80, 0x4a, 0xa5, 0xe2, 0xa8, 0x54, 0x2a, 0x00, 0x00, 0x00,  //
0x00, 0x47, 0x04, 0xc2, 0x80, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x80, 0x48, 0x24, 0x53, 0x99, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x0a, 0x42, 0x24, 0x53, 0x99, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x80, 0x48, 0xa2, 0x30, 0x28, 0x24, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x8e, 0x44, 0x22, 0x91, 0x48, 0x22, 0x00, 0x00, 0x00,  //
0x80, 0x48, 0x64, 0x53, 0x89, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x80, 0x48, 0x24, 0xf2, 0x89, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x47, 0x24, 0x12, 0x89, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x80, 0x4f, 0x24, 0x12, 0x89, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x80, 0x47, 0x24, 0xf2, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00,  //
0x00, 0x47, 0x24, 0x10, 0x08, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x80, 0x0f, 0x81, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00,  //
0x80, 0x48, 0x24, 0xa2, 0x20, 0x08, 0x02, 0x00, 0x00, 0x00,  //
0x00, 0x82, 0xa3, 0x52, 0xa9, 0x38, 0x08, 0x00, 0x00, 0x00,  //
0x80, 0x48, 0x44, 0x41, 0x50, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x80, 0x44, 0x22, 0x91, 0x48, 0x24, 0x3e, 0x10, 0x08, 0x00,  //
0x80, 0x48, 0x24, 0xe2, 0x81, 0x40, 0x20, 0x00, 0x00, 0x00,  //
0x80, 0x4a, 0xa5, 0x52, 0xa9, 0x54, 0x3e, 0x00, 0x00, 0x00,  //
0x80, 0x4a, 0xa5, 0x52, 0xa9, 0x54, 0x3e, 0x10, 0x08, 0x00,  //
0x80, 0x81, 0x40, 0xe0, 0x90, 0x48, 0x1c, 0x00, 0x00, 0x00,  //
0x80, 0x48, 0x24, 0x72, 0xc9, 0x64, 0x2e, 0x00, 0x00, 0x00,  //
0x00, 0x81, 0x40, 0xe0, 0x90, 0x48, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x47, 0x04, 0xc2, 0x81, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x80, 0x44, 0xa5, 0x72, 0xa9, 0x54, 0x12, 0x00, 0x00, 0x00,  //
0x00, 0x4f, 0x24, 0xe2, 0xa1, 0x48, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x01, 0xf1, 0x44, 0x3c, 0x00, 0x00, 0x00,  //
0x00, 0x4f, 0xc0, 0x11, 0x89, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xe0, 0x11, 0x79, 0x44, 0x1e, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x23, 0x10, 0x08, 0x04, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0xa1, 0x50, 0x28, 0x3e, 0x11, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x11, 0xf9, 0x04, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xa0, 0x52, 0x71, 0x54, 0x2a, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x11, 0x61, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0x92, 0xa9, 0x4c, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x05, 0x21, 0x92, 0xa9, 0x4c, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0x92, 0x38, 0x24, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x80, 0x23, 0x91, 0x48, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0xb2, 0xa9, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0x12, 0xf9, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x11, 0x89, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xe0, 0x13, 0x89, 0x44, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xa0, 0x31, 0x89, 0x4c, 0x1a, 0x81, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x11, 0x09, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xe0, 0x43, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0x12, 0x89, 0x78, 0x20, 0x11, 0x07, 0x00,  //
0x00, 0x02, 0xc1, 0x51, 0xa9, 0x54, 0x1c, 0x04, 0x02, 0x00,  //
0x00, 0x00, 0x20, 0xa2, 0x20, 0x28, 0x22, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0x91, 0x48, 0x24, 0x3e, 0x10, 0x08, 0x00,  //
0x00, 0x00, 0x20, 0x12, 0xf1, 0x40, 0x20, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xa0, 0x52, 0xa9, 0x54, 0x3e, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xa0, 0x52, 0xa9, 0x54, 0x3e, 0x10, 0x08, 0x00,  //
0x00, 0x00, 0x60, 0x20, 0x70, 0x48, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0x12, 0xb9, 0x64, 0x2e, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x40, 0x20, 0x70, 0x48, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x11, 0xe1, 0x44, 0x1c, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0x20, 0x51, 0xb9, 0x54, 0x12, 0x00, 0x00, 0x00,  //
0x00, 0x00, 0xc0, 0x13, 0xf1, 0x44, 0x22, 0x00, 0x00, 0x00,  //
};



const uint8_t Font4x6[] = {
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x22, 0x02, 0x02, 0x00,
0x55, 0x00, 0x00, 0x00,
0x75, 0x75, 0x05, 0x00,
0x36, 0x36, 0x02, 0x00,
0x41, 0x12, 0x04, 0x00,
0x33, 0x57, 0x06, 0x00,
0x22, 0x00, 0x00, 0x00,
0x24, 0x22, 0x04, 0x00,
0x21, 0x22, 0x01, 0x00,
0x25, 0x05, 0x00, 0x00,
0x20, 0x27, 0x00, 0x00,
0x00, 0x20, 0x01, 0x00,
0x00, 0x07, 0x00, 0x00,
0x00, 0x00, 0x02, 0x00,
0x44, 0x12, 0x01, 0x00,
0x56, 0x55, 0x03, 0x00,
0x32, 0x22, 0x02, 0x00,
0x43, 0x12, 0x07, 0x00,
0x43, 0x42, 0x03, 0x00,
0x55, 0x47, 0x04, 0x00,
0x17, 0x43, 0x03, 0x00,
0x16, 0x57, 0x07, 0x00,
0x47, 0x12, 0x01, 0x00,
0x57, 0x57, 0x07, 0x00,
0x57, 0x47, 0x03, 0x00,
0x20, 0x20, 0x00, 0x00,
0x20, 0x20, 0x01, 0x00,
0x24, 0x21, 0x04, 0x00,
0x70, 0x70, 0x00, 0x00,
0x21, 0x24, 0x01, 0x00,
0x47, 0x02, 0x02, 0x00,
0x52, 0x17, 0x06, 0x00,
0x52, 0x57, 0x05, 0x00,
0x53, 0x53, 0x03, 0x00,
0x16, 0x11, 0x06, 0x00,
0x53, 0x55, 0x03, 0x00,
0x17, 0x17, 0x07, 0x00,
0x17, 0x17, 0x01, 0x00,
0x16, 0x57, 0x06, 0x00,
0x55, 0x57, 0x05, 0x00,
0x27, 0x22, 0x07, 0x00,
0x44, 0x54, 0x02, 0x00,
0x55, 0x53, 0x05, 0x00,
0x11, 0x11, 0x07, 0x00,
0x75, 0x57, 0x05, 0x00,
0x75, 0x77, 0x05, 0x00,
0x52, 0x55, 0x02, 0x00,
0x53, 0x13, 0x01, 0x00,
0x52, 0x75, 0x06, 0x00,
0x53, 0x37, 0x05, 0x00,
0x16, 0x42, 0x03, 0x00,
0x27, 0x22, 0x02, 0x00,
0x55, 0x55, 0x06, 0x00,
0x55, 0x25, 0x02, 0x00,
0x55, 0x77, 0x05, 0x00,
0x55, 0x52, 0x05, 0x00,
0x55, 0x22, 0x02, 0x00,
0x47, 0x12, 0x07, 0x00,
0x17, 0x11, 0x07, 0x00,
0x10, 0x42, 0x00, 0x00,
0x47, 0x44, 0x07, 0x00,
0x52, 0x00, 0x00, 0x00,
0x00, 0x00, 0x07, 0x00,
0x21, 0x00, 0x00, 0x00,
0x30, 0x56, 0x07, 0x00,
0x31, 0x55, 0x03, 0x00,
0x60, 0x11, 0x06, 0x00,
0x64, 0x55, 0x06, 0x00,
0x60, 0x35, 0x06, 0x00,
0x24, 0x27, 0x02, 0x00,
0x60, 0x75, 0x24, 0x00,
0x31, 0x55, 0x05, 0x00,
0x02, 0x22, 0x02, 0x00,
0x04, 0x44, 0x25, 0x00,
0x51, 0x33, 0x05, 0x00,
0x23, 0x22, 0x07, 0x00,
0x70, 0x77, 0x05, 0x00,
0x30, 0x55, 0x05, 0x00,
0x20, 0x55, 0x02, 0x00,
0x30, 0x55, 0x13, 0x00,
0x60, 0x55, 0x46, 0x00,
0x60, 0x11, 0x01, 0x00,
0x60, 0x63, 0x03, 0x00,
0x72, 0x22, 0x06, 0x00,
0x50, 0x55, 0x06, 0x00,
0x50, 0x75, 0x02, 0x00,
0x50, 0x77, 0x07, 0x00,
0x50, 0x22, 0x05, 0x00,
0x50, 0x65, 0x24, 0x00,
0x70, 0x36, 0x07, 0x00,
0x26, 0x21, 0x06, 0x00,
0x22, 0x20, 0x02, 0x00,
0x23, 0x24, 0x03, 0x00,
0x36, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
};




const uint16_t Font16x26 [] = {
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [ ]
0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03C0,0x03C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x0000,0x0000,0x0000,0x03E0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [!]
0x1E3C,0x1E3C,0x1E3C,0x1E3C,0x1E3C,0x1E3C,0x1E3C,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = ["]
0x01CE,0x03CE,0x03DE,0x039E,0x039C,0x079C,0x3FFF,0x7FFF,0x0738,0x0F38,0x0F78,0x0F78,0x0E78,0xFFFF,0xFFFF,0x1EF0,0x1CF0,0x1CE0,0x3CE0,0x3DE0,0x39E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [#]
0x03FC,0x0FFE,0x1FEE,0x1EE0,0x1EE0,0x1EE0,0x1EE0,0x1FE0,0x0FE0,0x07E0,0x03F0,0x01FC,0x01FE,0x01FE,0x01FE,0x01FE,0x01FE,0x01FE,0x3DFE,0x3FFC,0x0FF0,0x01E0,0x01E0,0x0000,0x0000,0x0000, // Ascii = [$]
0x3E03,0xF707,0xE78F,0xE78E,0xE39E,0xE3BC,0xE7B8,0xE7F8,0xF7F0,0x3FE0,0x01C0,0x03FF,0x07FF,0x07F3,0x0FF3,0x1EF3,0x3CF3,0x38F3,0x78F3,0xF07F,0xE03F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [%]
0x07E0,0x0FF8,0x0F78,0x1F78,0x1F78,0x1F78,0x0F78,0x0FF0,0x0FE0,0x1F80,0x7FC3,0xFBC3,0xF3E7,0xF1F7,0xF0F7,0xF0FF,0xF07F,0xF83E,0x7C7F,0x3FFF,0x1FEF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [&]
0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03C0,0x01C0,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [']
0x003F,0x007C,0x01F0,0x01E0,0x03C0,0x07C0,0x0780,0x0780,0x0F80,0x0F00,0x0F00,0x0F00,0x0F00,0x0F00,0x0F00,0x0F80,0x0780,0x0780,0x07C0,0x03C0,0x01E0,0x01F0,0x007C,0x003F,0x000F,0x0000, // Ascii = [(]
0x7E00,0x1F00,0x07C0,0x03C0,0x01E0,0x01F0,0x00F0,0x00F0,0x00F8,0x0078,0x0078,0x0078,0x0078,0x0078,0x0078,0x00F8,0x00F0,0x00F0,0x01F0,0x01E0,0x03C0,0x07C0,0x1F00,0x7E00,0x7800,0x0000, // Ascii = [)]
0x03E0,0x03C0,0x01C0,0x39CE,0x3FFF,0x3F7F,0x0320,0x0370,0x07F8,0x0F78,0x1F3C,0x0638,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [*]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0xFFFF,0xFFFF,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [+]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03E0,0x03E0,0x03E0,0x03E0,0x01E0,0x01E0,0x01E0,0x01C0,0x0380, // Ascii = [,]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x3FFE,0x3FFE,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [-]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03E0,0x03E0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [.]
0x000F,0x000F,0x001E,0x001E,0x003C,0x003C,0x0078,0x0078,0x00F0,0x00F0,0x01E0,0x01E0,0x03C0,0x03C0,0x0780,0x0780,0x0F00,0x0F00,0x1E00,0x1E00,0x3C00,0x3C00,0x7800,0x7800,0xF000,0x0000, // Ascii = [/]
0x07F0,0x0FF8,0x1F7C,0x3E3E,0x3C1E,0x7C1F,0x7C1F,0x780F,0x780F,0x780F,0x780F,0x780F,0x780F,0x780F,0x7C1F,0x7C1F,0x3C1E,0x3E3E,0x1F7C,0x0FF8,0x07F0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [0]
0x00F0,0x07F0,0x3FF0,0x3FF0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x3FFF,0x3FFF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [1]
0x0FE0,0x3FF8,0x3C7C,0x003C,0x003E,0x003E,0x003E,0x003C,0x003C,0x007C,0x00F8,0x01F0,0x03E0,0x07C0,0x0780,0x0F00,0x1E00,0x3E00,0x3C00,0x3FFE,0x3FFE,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [2]
0x0FF0,0x1FF8,0x1C7C,0x003E,0x003E,0x003E,0x003C,0x003C,0x00F8,0x0FF0,0x0FF8,0x007C,0x003E,0x001E,0x001E,0x001E,0x001E,0x003E,0x1C7C,0x1FF8,0x1FE0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [3]
0x0078,0x00F8,0x00F8,0x01F8,0x03F8,0x07F8,0x07F8,0x0F78,0x1E78,0x1E78,0x3C78,0x7878,0x7878,0xFFFF,0xFFFF,0x0078,0x0078,0x0078,0x0078,0x0078,0x0078,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [4]
0x1FFC,0x1FFC,0x1FFC,0x1E00,0x1E00,0x1E00,0x1E00,0x1E00,0x1FE0,0x1FF8,0x00FC,0x007C,0x003E,0x003E,0x001E,0x003E,0x003E,0x003C,0x1C7C,0x1FF8,0x1FE0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [5]
0x01FC,0x07FE,0x0F8E,0x1F00,0x1E00,0x3E00,0x3C00,0x3C00,0x3DF8,0x3FFC,0x7F3E,0x7E1F,0x3C0F,0x3C0F,0x3C0F,0x3C0F,0x3E0F,0x1E1F,0x1F3E,0x0FFC,0x03F0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [6]
0x3FFF,0x3FFF,0x3FFF,0x000F,0x001E,0x001E,0x003C,0x0038,0x0078,0x00F0,0x00F0,0x01E0,0x01E0,0x03C0,0x03C0,0x0780,0x0F80,0x0F80,0x0F00,0x1F00,0x1F00,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [7]
0x07F8,0x0FFC,0x1F3E,0x1E1E,0x3E1E,0x3E1E,0x1E1E,0x1F3C,0x0FF8,0x07F0,0x0FF8,0x1EFC,0x3E3E,0x3C1F,0x7C1F,0x7C0F,0x7C0F,0x3C1F,0x3F3E,0x1FFC,0x07F0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [8]
0x07F0,0x0FF8,0x1E7C,0x3C3E,0x3C1E,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x3C1F,0x3E3F,0x1FFF,0x07EF,0x001F,0x001E,0x001E,0x003E,0x003C,0x38F8,0x3FF0,0x1FE0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [9]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03E0,0x03E0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03E0,0x03E0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [:]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03E0,0x03E0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03E0,0x03E0,0x03E0,0x03E0,0x01E0,0x01E0,0x01E0,0x03C0,0x0380, // Ascii = [;]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0003,0x000F,0x003F,0x00FC,0x03F0,0x0FC0,0x3F00,0xFE00,0x3F00,0x0FC0,0x03F0,0x00FC,0x003F,0x000F,0x0003,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [<]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [=]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xE000,0xF800,0x7E00,0x1F80,0x07E0,0x01F8,0x007E,0x001F,0x007E,0x01F8,0x07E0,0x1F80,0x7E00,0xF800,0xE000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [>]
0x1FF0,0x3FFC,0x383E,0x381F,0x381F,0x001E,0x001E,0x003C,0x0078,0x00F0,0x01E0,0x03C0,0x03C0,0x07C0,0x07C0,0x0000,0x0000,0x0000,0x07C0,0x07C0,0x07C0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [?]
0x03F8,0x0FFE,0x1F1E,0x3E0F,0x3C7F,0x78FF,0x79EF,0x73C7,0xF3C7,0xF38F,0xF38F,0xF38F,0xF39F,0xF39F,0x73FF,0x7BFF,0x79F7,0x3C00,0x1F1C,0x0FFC,0x03F8,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [@]
0x0000,0x0000,0x0000,0x03E0,0x03E0,0x07F0,0x07F0,0x07F0,0x0F78,0x0F78,0x0E7C,0x1E3C,0x1E3C,0x3C3E,0x3FFE,0x3FFF,0x781F,0x780F,0xF00F,0xF007,0xF007,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [A]
0x0000,0x0000,0x0000,0x3FF8,0x3FFC,0x3C3E,0x3C1E,0x3C1E,0x3C1E,0x3C3E,0x3C7C,0x3FF0,0x3FF8,0x3C7E,0x3C1F,0x3C1F,0x3C0F,0x3C0F,0x3C1F,0x3FFE,0x3FF8,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [B]
0x0000,0x0000,0x0000,0x01FF,0x07FF,0x1F87,0x3E00,0x3C00,0x7C00,0x7800,0x7800,0x7800,0x7800,0x7800,0x7C00,0x7C00,0x3E00,0x3F00,0x1F83,0x07FF,0x01FF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [C]
0x0000,0x0000,0x0000,0x7FF0,0x7FFC,0x787E,0x781F,0x781F,0x780F,0x780F,0x780F,0x780F,0x780F,0x780F,0x780F,0x780F,0x781F,0x781E,0x787E,0x7FF8,0x7FE0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [D]
0x0000,0x0000,0x0000,0x3FFF,0x3FFF,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3FFE,0x3FFE,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3FFF,0x3FFF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [E]
0x0000,0x0000,0x0000,0x1FFF,0x1FFF,0x1E00,0x1E00,0x1E00,0x1E00,0x1E00,0x1E00,0x1FFF,0x1FFF,0x1E00,0x1E00,0x1E00,0x1E00,0x1E00,0x1E00,0x1E00,0x1E00,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [F]
0x0000,0x0000,0x0000,0x03FE,0x0FFF,0x1F87,0x3E00,0x7C00,0x7C00,0x7800,0xF800,0xF800,0xF87F,0xF87F,0x780F,0x7C0F,0x7C0F,0x3E0F,0x1F8F,0x0FFF,0x03FE,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [G]
0x0000,0x0000,0x0000,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7FFF,0x7FFF,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x7C1F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [H]
0x0000,0x0000,0x0000,0x3FFF,0x3FFF,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x3FFF,0x3FFF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [I]
0x0000,0x0000,0x0000,0x1FFC,0x1FFC,0x007C,0x007C,0x007C,0x007C,0x007C,0x007C,0x007C,0x007C,0x007C,0x007C,0x007C,0x0078,0x0078,0x38F8,0x3FF0,0x3FC0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [J]
0x0000,0x0000,0x0000,0x3C1F,0x3C1E,0x3C3C,0x3C78,0x3CF0,0x3DE0,0x3FE0,0x3FC0,0x3F80,0x3FC0,0x3FE0,0x3DF0,0x3CF0,0x3C78,0x3C7C,0x3C3E,0x3C1F,0x3C0F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [K]
0x0000,0x0000,0x0000,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3FFF,0x3FFF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [L]
0x0000,0x0000,0x0000,0xF81F,0xFC1F,0xFC1F,0xFE3F,0xFE3F,0xFE3F,0xFF7F,0xFF77,0xFF77,0xF7F7,0xF7E7,0xF3E7,0xF3E7,0xF3C7,0xF007,0xF007,0xF007,0xF007,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [M]
0x0000,0x0000,0x0000,0x7C0F,0x7C0F,0x7E0F,0x7F0F,0x7F0F,0x7F8F,0x7F8F,0x7FCF,0x7BEF,0x79EF,0x79FF,0x78FF,0x78FF,0x787F,0x783F,0x783F,0x781F,0x781F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [N]
0x0000,0x0000,0x0000,0x07F0,0x1FFC,0x3E3E,0x7C1F,0x780F,0x780F,0xF80F,0xF80F,0xF80F,0xF80F,0xF80F,0xF80F,0x780F,0x780F,0x7C1F,0x3E3E,0x1FFC,0x07F0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [O]
0x0000,0x0000,0x0000,0x3FFC,0x3FFF,0x3E1F,0x3E0F,0x3E0F,0x3E0F,0x3E0F,0x3E1F,0x3E3F,0x3FFC,0x3FF0,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x3E00,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [P]
0x0000,0x0000,0x0000,0x07F0,0x1FFC,0x3E3E,0x7C1F,0x780F,0x780F,0xF80F,0xF80F,0xF80F,0xF80F,0xF80F,0xF80F,0x780F,0x780F,0x7C1F,0x3E3E,0x1FFC,0x07F8,0x007C,0x003F,0x000F,0x0003,0x0000, // Ascii = [Q]
0x0000,0x0000,0x0000,0x3FF0,0x3FFC,0x3C7E,0x3C3E,0x3C1E,0x3C1E,0x3C3E,0x3C3C,0x3CFC,0x3FF0,0x3FE0,0x3DF0,0x3CF8,0x3C7C,0x3C3E,0x3C1E,0x3C1F,0x3C0F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [R]
0x0000,0x0000,0x0000,0x07FC,0x1FFE,0x3E0E,0x3C00,0x3C00,0x3C00,0x3E00,0x1FC0,0x0FF8,0x03FE,0x007F,0x001F,0x000F,0x000F,0x201F,0x3C3E,0x3FFC,0x1FF0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [S]
0x0000,0x0000,0x0000,0xFFFF,0xFFFF,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [T]
0x0000,0x0000,0x0000,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x7C0F,0x3C1E,0x3C1E,0x3E3E,0x1FFC,0x07F0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [U]
0x0000,0x0000,0x0000,0xF007,0xF007,0xF807,0x780F,0x7C0F,0x3C1E,0x3C1E,0x3E1E,0x1E3C,0x1F3C,0x1F78,0x0F78,0x0FF8,0x07F0,0x07F0,0x07F0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [V]
0x0000,0x0000,0x0000,0xE003,0xF003,0xF003,0xF007,0xF3E7,0xF3E7,0xF3E7,0x73E7,0x7BF7,0x7FF7,0x7FFF,0x7F7F,0x7F7F,0x7F7E,0x3F7E,0x3E3E,0x3E3E,0x3E3E,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [W]
0x0000,0x0000,0x0000,0xF807,0x7C0F,0x3E1E,0x3E3E,0x1F3C,0x0FF8,0x07F0,0x07E0,0x03E0,0x03E0,0x07F0,0x0FF8,0x0F7C,0x1E7C,0x3C3E,0x781F,0x780F,0xF00F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [X]
0x0000,0x0000,0x0000,0xF807,0x7807,0x7C0F,0x3C1E,0x3E1E,0x1F3C,0x0F78,0x0FF8,0x07F0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [Y]
0x0000,0x0000,0x0000,0x7FFF,0x7FFF,0x000F,0x001F,0x003E,0x007C,0x00F8,0x00F0,0x01E0,0x03E0,0x07C0,0x0F80,0x0F00,0x1E00,0x3E00,0x7C00,0x7FFF,0x7FFF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [Z]
0x07FF,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x07FF,0x07FF,0x0000, // Ascii = [[]
0x7800,0x7800,0x3C00,0x3C00,0x1E00,0x1E00,0x0F00,0x0F00,0x0780,0x0780,0x03C0,0x03C0,0x01E0,0x01E0,0x00F0,0x00F0,0x0078,0x0078,0x003C,0x003C,0x001E,0x001E,0x000F,0x000F,0x0007,0x0000, // Ascii = [\]
0x7FF0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x00F0,0x7FF0,0x7FF0,0x0000, // Ascii = []]
0x00C0,0x01C0,0x01C0,0x03E0,0x03E0,0x07F0,0x07F0,0x0778,0x0F78,0x0F38,0x1E3C,0x1E3C,0x3C1E,0x3C1E,0x380F,0x780F,0x7807,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [^]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xFFFF,0xFFFF,0x0000,0x0000,0x0000, // Ascii = [_]
0x00F0,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [`]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0FF8,0x3FFC,0x3C7C,0x003E,0x003E,0x003E,0x07FE,0x1FFE,0x3E3E,0x7C3E,0x783E,0x7C3E,0x7C7E,0x3FFF,0x1FCF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [a]
0x3C00,0x3C00,0x3C00,0x3C00,0x3C00,0x3C00,0x3DF8,0x3FFE,0x3F3E,0x3E1F,0x3C0F,0x3C0F,0x3C0F,0x3C0F,0x3C0F,0x3C0F,0x3C1F,0x3C1E,0x3F3E,0x3FFC,0x3BF0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [b]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03FE,0x0FFF,0x1F87,0x3E00,0x3E00,0x3C00,0x7C00,0x7C00,0x7C00,0x3C00,0x3E00,0x3E00,0x1F87,0x0FFF,0x03FE,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [c]
0x001F,0x001F,0x001F,0x001F,0x001F,0x001F,0x07FF,0x1FFF,0x3E3F,0x3C1F,0x7C1F,0x7C1F,0x7C1F,0x781F,0x781F,0x7C1F,0x7C1F,0x3C3F,0x3E7F,0x1FFF,0x0FDF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [d]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x03F8,0x0FFC,0x1F3E,0x3E1E,0x3C1F,0x7C1F,0x7FFF,0x7FFF,0x7C00,0x7C00,0x3C00,0x3E00,0x1F07,0x0FFF,0x03FE,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [e]
0x01FF,0x03E1,0x03C0,0x07C0,0x07C0,0x07C0,0x7FFF,0x7FFF,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x07C0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [f]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x07EF,0x1FFF,0x3E7F,0x3C1F,0x7C1F,0x7C1F,0x781F,0x781F,0x781F,0x7C1F,0x7C1F,0x3C3F,0x3E7F,0x1FFF,0x0FDF,0x001E,0x001E,0x001E,0x387C,0x3FF8, // Ascii = [g]
0x3C00,0x3C00,0x3C00,0x3C00,0x3C00,0x3C00,0x3DFC,0x3FFE,0x3F9E,0x3F1F,0x3E1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [h]
0x01F0,0x01F0,0x0000,0x0000,0x0000,0x0000,0x7FE0,0x7FE0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [i]
0x00F8,0x00F8,0x0000,0x0000,0x0000,0x0000,0x3FF8,0x3FF8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F8,0x00F0,0x71F0,0x7FE0, // Ascii = [j]
0x3C00,0x3C00,0x3C00,0x3C00,0x3C00,0x3C00,0x3C1F,0x3C3E,0x3C7C,0x3CF8,0x3DF0,0x3DE0,0x3FC0,0x3FC0,0x3FE0,0x3DF0,0x3CF8,0x3C7C,0x3C3E,0x3C1F,0x3C1F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [k]
0x7FF0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x01F0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [l]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xF79E,0xFFFF,0xFFFF,0xFFFF,0xFBE7,0xF9E7,0xF1C7,0xF1C7,0xF1C7,0xF1C7,0xF1C7,0xF1C7,0xF1C7,0xF1C7,0xF1C7,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [m]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x3DFC,0x3FFE,0x3F9E,0x3F1F,0x3E1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x3C1F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [n]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x07F0,0x1FFC,0x3E3E,0x3C1F,0x7C1F,0x780F,0x780F,0x780F,0x780F,0x780F,0x7C1F,0x3C1F,0x3E3E,0x1FFC,0x07F0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [o]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x3DF8,0x3FFE,0x3F3E,0x3E1F,0x3C0F,0x3C0F,0x3C0F,0x3C0F,0x3C0F,0x3C0F,0x3C1F,0x3E1E,0x3F3E,0x3FFC,0x3FF8,0x3C00,0x3C00,0x3C00,0x3C00,0x3C00, // Ascii = [p]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x07EE,0x1FFE,0x3E7E,0x3C1E,0x7C1E,0x781E,0x781E,0x781E,0x781E,0x781E,0x7C1E,0x7C3E,0x3E7E,0x1FFE,0x0FDE,0x001E,0x001E,0x001E,0x001E,0x001E, // Ascii = [q]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x1F7F,0x1FFF,0x1FE7,0x1FC7,0x1F87,0x1F00,0x1F00,0x1F00,0x1F00,0x1F00,0x1F00,0x1F00,0x1F00,0x1F00,0x1F00,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [r]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x07FC,0x1FFE,0x1E0E,0x3E00,0x3E00,0x3F00,0x1FE0,0x07FC,0x00FE,0x003E,0x001E,0x001E,0x3C3E,0x3FFC,0x1FF0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [s]
0x0000,0x0000,0x0000,0x0780,0x0780,0x0780,0x7FFF,0x7FFF,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x0780,0x07C0,0x03FF,0x01FF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [t]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x3C1E,0x3C1E,0x3C1E,0x3C1E,0x3C1E,0x3C1E,0x3C1E,0x3C1E,0x3C1E,0x3C1E,0x3C3E,0x3C7E,0x3EFE,0x1FFE,0x0FDE,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [u]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xF007,0x780F,0x780F,0x3C1E,0x3C1E,0x3E1E,0x1E3C,0x1E3C,0x0F78,0x0F78,0x0FF0,0x07F0,0x07F0,0x03E0,0x03E0,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [v]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xF003,0xF1E3,0xF3E3,0xF3E7,0xF3F7,0xF3F7,0x7FF7,0x7F77,0x7F7F,0x7F7F,0x7F7F,0x3E3E,0x3E3E,0x3E3E,0x3E3E,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [w]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x7C0F,0x3E1E,0x3E3C,0x1F3C,0x0FF8,0x07F0,0x07F0,0x03E0,0x07F0,0x07F8,0x0FF8,0x1E7C,0x3E3E,0x3C1F,0x781F,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [x]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xF807,0x780F,0x7C0F,0x3C1E,0x3C1E,0x1E3C,0x1E3C,0x1F3C,0x0F78,0x0FF8,0x07F0,0x07F0,0x03E0,0x03E0,0x03C0,0x03C0,0x03C0,0x0780,0x0F80,0x7F00, // Ascii = [y]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x3FFF,0x3FFF,0x001F,0x003E,0x007C,0x00F8,0x01F0,0x03E0,0x07C0,0x0F80,0x1F00,0x1E00,0x3C00,0x7FFF,0x7FFF,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [z]
0x01FE,0x03E0,0x03C0,0x03C0,0x03C0,0x03C0,0x01E0,0x01E0,0x01E0,0x01C0,0x03C0,0x3F80,0x3F80,0x03C0,0x01C0,0x01E0,0x01E0,0x01E0,0x03C0,0x03C0,0x03C0,0x03C0,0x03E0,0x01FE,0x007E,0x0000, // Ascii = [{]
0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x01C0,0x0000, // Ascii = [|]
0x3FC0,0x03E0,0x01E0,0x01E0,0x01E0,0x01E0,0x01C0,0x03C0,0x03C0,0x01C0,0x01E0,0x00FE,0x00FE,0x01E0,0x01C0,0x03C0,0x03C0,0x01C0,0x01E0,0x01E0,0x01E0,0x01E0,0x03E0,0x3FC0,0x3F00,0x0000, // Ascii = [}]
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x3F07,0x7FC7,0x73E7,0xF1FF,0xF07E,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000, // Ascii = [~]
};

FontDef_t Font_16x26 = {
	16,
	26,
	Font16x26
};
*/

/*
FontDef_t Font_16x26 = {
	16,
	23,
	46,
	Font16x26
};





FontDef_t Font_4x6 = {
	4,
	6,
	4,
	Font4x6
};

FontDef_t Font_7x10 = {
	7,
	10,
	10,
	Font7x10
};
*/

char* FONTS_GetStringSize(char* str, FONTS_SIZE_t* SizeStruct, FontDef_t* Font) {
	/* Fill settings */
	SizeStruct->Height = Font->FontHeight;
	SizeStruct->Length = Font->FontWidth * strlen(str);
	
	/* Return pointer */
	return str;
}
