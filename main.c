/*
 * main.c
 *
 *  Created on: 26-02-2015
 *      Author: Mefiu
 */

#include <avr/io.h>
#include <util/delay.h>

#include "PCD8544_LCD/PCD8544_LCD.h"

/************DEBUG LED*********************/
#define LED_PORT PORTD
#define LED_DDRX DDRD
#define LED_PIN (1<<PD7)
#define LED_ON LED_PORT &= ~LED_PIN
#define LED_OFF LED_PORT |= LED_PIN
#define LED_TOGGLE LED_PORT ^= LED_PIN
/******************************************/

//extern const uint8_t picture[];
int main(void)
{
/*****INIT LED*****/
	LED_DDRX |= LED_PIN;
	LED_OFF;
/******************/

	pcd8544_init();
	font = &font_5x7;

//	for(int i=1;i<20;i++)
//
//	pcd8544_drawPixel(i,1,1);
//	pcd8544_drawPixel(1,2,1);
//	pcd8544_drawPixel(1,3,1);
//	pcd8544_drawPixel(1,4,1);

//	pcd8544_drawFastHLine(3,5,40,1);
//	pcd8544_drawFastVLine(5,5,37,1);
//	pcd8544_drawLine(0,0,67,34,1);
//
//	pcd8544_drawChar(10,10,'A',1,0,1, font);

	pcd8544_puts(0,0,"Mateusz",1,1,0,font);
	pcd8544_puts(0,8,"   Salamon",1,1,0,font);
	pcd8544_puts(0,24," Junior",1,1,0,font);
	pcd8544_puts(0,32,"  embedded",1,1,0,font);
	pcd8544_puts(0,40,"   developer",1,1,0,font);

//	pcd8544_drawBitmap_P(0,0,picture,84,48,1);





	pcd8544_display();
//	int i = 0;
//	while(1)
//	{
//
//		LED_ON;
////		pcd8544_invertedMode();
//		pcd8544_drawPixel(i,5,1);
//		pcd8544_display();
//		_delay_ms(500);
//		LED_OFF;
//		pcd8544_drawPixel(5,i,1);
//		pcd8544_display();
////		pcd8544_normalMode();
//		_delay_ms(500);
//		i++;
//	}
}

