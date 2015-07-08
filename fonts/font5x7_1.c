/*
 * font5x7.c
 *
 *  Created on: 17-06-2015
 *      Author: Mefiu
 */
#include <avr/io.h>
#include <avr/pgmspace.h>

const uint8_t font_5x7[] PROGMEM =
{
		0x00, 0x00, 0x00, 0x00, 0x00, 	//0 null
			0x3E, 0x5B, 0x4F, 0x5B, 0x3E,	//1
			0x3E, 0x6B, 0x4F, 0x6B, 0x3E,	//2
			0x1C, 0x3E, 0x7C, 0x3E, 0x1C,	//3
			0x18, 0x3C, 0x7E, 0x3C, 0x18,	//4
			0x1C, 0x57, 0x7D, 0x57, 0x1C,	//5
			0x1C, 0x5E, 0x7F, 0x5E, 0x1C,	//6
			0x00, 0x18, 0x3C, 0x18, 0x00,	//7
			0xFF, 0xE7, 0xC3, 0xE7, 0xFF,	//8
			0x00, 0x18, 0x24, 0x18, 0x00,	//9
			0xFF, 0xE7, 0xDB, 0xE7, 0xFF,	//10
			0x30, 0x48, 0x3A, 0x06, 0x0E,	//11
			0x26, 0x29, 0x79, 0x29, 0x26,	//12
			0x40, 0x7F, 0x05, 0x05, 0x07,	//13
			0x40, 0x7F, 0x05, 0x25, 0x3F,	//14
			0x5A, 0x3C, 0xE7, 0x3C, 0x5A,	//15
			0x7F, 0x3E, 0x1C, 0x1C, 0x08,	//16
			0x08, 0x1C, 0x1C, 0x3E, 0x7F,	//17
			0x14, 0x22, 0x7F, 0x22, 0x14,	//18
			0x5F, 0x5F, 0x00, 0x5F, 0x5F,	//19
			0x06, 0x09, 0x7F, 0x01, 0x7F,	//20
			0x00, 0x66, 0x89, 0x95, 0x6A,	//21
			0x60, 0x60, 0x60, 0x60, 0x60,	//22
			0x94, 0xA2, 0xFF, 0xA2, 0x94,	//23
			0x08, 0x04, 0x7E, 0x04, 0x08,	//24
			0x10, 0x20, 0x7E, 0x20, 0x10,	//25
			0x08, 0x08, 0x2A, 0x1C, 0x08,	//26
			0x08, 0x1C, 0x2A, 0x08, 0x08,	//27
			0x1E, 0x10, 0x10, 0x10, 0x10,	//28
			0x0C, 0x1E, 0x0C, 0x1E, 0x0C,	//29
			0x30, 0x38, 0x3E, 0x38, 0x30,	//30
			0x06, 0x0E, 0x3E, 0x0E, 0x06,	//31
			0x00, 0x00, 0x00, 0x00, 0x00,	//32 (space)
			0x00, 0x00, 0x5F, 0x00, 0x00,	//33 !
			0x00, 0x07, 0x00, 0x07, 0x00,	//34 "
			0x14, 0x7F, 0x14, 0x7F, 0x14,	//35 #
			0x24, 0x2A, 0x7F, 0x2A, 0x12,	//36 $
			0x23, 0x13, 0x08, 0x64, 0x62,	//37 %
			0x36, 0x49, 0x56, 0x20, 0x50,	//38 &
			0x00, 0x08, 0x07, 0x03, 0x00,	//39 â
			0x00, 0x1C, 0x22, 0x41, 0x00,	//40 (
			0x00, 0x41, 0x22, 0x1C, 0x00,	//41 )
			0x2A, 0x1C, 0x7F, 0x1C, 0x2A,	//42 *
			0x08, 0x08, 0x3E, 0x08, 0x08,	//43 +
			0x00, 0x80, 0x70, 0x30, 0x00,	//44 ,
			0x08, 0x08, 0x08, 0x08, 0x08,	//45 -
			0x00, 0x00, 0x60, 0x60, 0x00,	//46 .
			0x20, 0x10, 0x08, 0x04, 0x02,	//47 /
			0x3E, 0x51, 0x49, 0x45, 0x3E,	//48 0
			0x00, 0x42, 0x7F, 0x40, 0x00,	//49 1
			0x72, 0x49, 0x49, 0x49, 0x46,	//50 2
			0x21, 0x41, 0x49, 0x4D, 0x33,	//51 3
			0x18, 0x14, 0x12, 0x7F, 0x10,	//52 4
			0x27, 0x45, 0x45, 0x45, 0x39,	//53 5
			0x3C, 0x4A, 0x49, 0x49, 0x31,	//54 6
			0x41, 0x21, 0x11, 0x09, 0x07,	//55 7
			0x36, 0x49, 0x49, 0x49, 0x36,	//56 8
			0x46, 0x49, 0x49, 0x29, 0x1E,	//57 9
			0x00, 0x00, 0x14, 0x00, 0x00,	//58 :
			0x00, 0x40, 0x34, 0x00, 0x00,	//59 ;
			0x00, 0x08, 0x14, 0x22, 0x41,	//60 <
			0x14, 0x14, 0x14, 0x14, 0x14,	//61 =
			0x00, 0x41, 0x22, 0x14, 0x08,	//62 >
			0x02, 0x01, 0x59, 0x09, 0x06,	//63 ?
			0x3E, 0x41, 0x5D, 0x59, 0x4E,	//64 @
			0x7E, 0x09, 0x09, 0x09, 0x7E,	//65 A
			0x7F, 0x49, 0x49, 0x49, 0x36,	//66 B
			0x3E, 0x41, 0x41, 0x41, 0x00,	//67 C
			0x7F, 0x41, 0x41, 0x22, 0x1C,	//68 D
			0x7F, 0x49, 0x49, 0x49, 0x41,	//69 E
			0x7F, 0x09, 0x09, 0x09, 0x01,	//70 F
			0x3E, 0x41, 0x49, 0x49, 0x38,	//71 G
			0x7F, 0x08, 0x08, 0x08, 0x7F,	//72 H
			0x00, 0x00, 0x7F, 0x00, 0x00,	//73 I
			0x20, 0x40, 0x40, 0x40, 0x3F,	//74 J
			0x7F, 0x08, 0x14, 0x22, 0x41,	//75 K
			0x7F, 0x40, 0x40, 0x40, 0x40,	//76 L
			0x7F, 0x02, 0x04, 0x02, 0x7F,	//77 M
			0x7F, 0x02, 0x04, 0x08, 0x7F,	//78 N
			0x3E, 0x41, 0x41, 0x41, 0x3E,	//79 O
			0x7F, 0x09, 0x09, 0x09, 0x06,	//80 P
			0x3E, 0x41, 0x41, 0x61, 0x7E,	//81 Q
			0x7F, 0x09, 0x19, 0x29, 0x46,	//82 R
			0x26, 0x49, 0x49, 0x49, 0x32,	//83 S
			0x01, 0x01, 0x7F, 0x01, 0x01,	//84 T
			0x3F, 0x40, 0x40, 0x40, 0x3F,	//85 U
			0x1F, 0x20, 0x40, 0x20, 0x1F,	//86 V
			0x7F, 0x20, 0x10, 0x20, 0x7F,	//87 W
			0x63, 0x14, 0x08, 0x14, 0x63,	//88 X
			0x07, 0x08, 0x78, 0x08, 0x07,	//89 Y
			0x61, 0x51, 0x49, 0x45, 0x43,	//90 Z
			0x00, 0x7F, 0x41, 0x41, 0x41,	//91
			0x02, 0x04, 0x08, 0x10, 0x20,	//92
			0x00, 0x41, 0x41, 0x41, 0x7F,	//93
			0x04, 0x02, 0x01, 0x02, 0x04,	//94
			0x40, 0x40, 0x40, 0x40, 0x40,	//95
			0x00, 0x03, 0x07, 0x08, 0x00,	//96
			0x20, 0x54, 0x54, 0x78, 0x40,	//97  a
			0x7F, 0x28, 0x44, 0x44, 0x38,	//98  b
			0x38, 0x44, 0x44, 0x44, 0x28,	//99  c
			0x38, 0x44, 0x44, 0x28, 0x7F,	//100 d
			0x38, 0x54, 0x54, 0x54, 0x18,	//101 e
			0x00, 0x08, 0x7E, 0x09, 0x02,	//102 f
			0x18, 0xA4, 0xA4, 0x9C, 0x78,	//103 g
			0x7F, 0x08, 0x04, 0x04, 0x78,	//104 h
			0x00, 0x44, 0x7D, 0x40, 0x00,	//105 i
			0x20, 0x40, 0x40, 0x3D, 0x00,	//106 j
			0x7F, 0x10, 0x28, 0x44, 0x00,	//107 k
			0x00, 0x41, 0x7F, 0x40, 0x00,	//108 l
			0x7C, 0x04, 0x78, 0x04, 0x78,	//109 m
			0x7C, 0x08, 0x04, 0x04, 0x78,	//110 n
			0x38, 0x44, 0x44, 0x44, 0x38,	//111 o
			0xFC, 0x18, 0x24, 0x24, 0x18,	//112 p
			0x18, 0x24, 0x24, 0x18, 0xFC,	//113 q
			0x7C, 0x08, 0x04, 0x04, 0x08,	//114 r
			0x48, 0x54, 0x54, 0x54, 0x24,	//115 s
			0x04, 0x04, 0x3F, 0x44, 0x24,	//116 t
			0x3C, 0x40, 0x40, 0x20, 0x7C,	//117 u
			0x1C, 0x20, 0x40, 0x20, 0x1C,	//118 v
			0x3C, 0x40, 0x30, 0x40, 0x3C,	//119 w
			0x44, 0x28, 0x10, 0x28, 0x44,	//120 x
			0x4C, 0x90, 0x90, 0x90, 0x7C,	//121 y
			0x44, 0x64, 0x54, 0x4C, 0x44,	//122 z
			0x00, 0x08, 0x36, 0x41, 0x00,	//123 {
			0x00, 0x00, 0x77, 0x00, 0x00,	//124 |
			0x00, 0x41, 0x36, 0x08, 0x00,	//125 }
			0x02, 0x01, 0x02, 0x04, 0x02,	//126 ~
};
