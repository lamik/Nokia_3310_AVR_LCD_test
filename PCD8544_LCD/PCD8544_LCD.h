/*
 * PCD8544_LCD.h
 *
 *  Created on: 11-02-2015
 *      Author: Mefiu
 */

#ifndef PCD8544_LCD_H_
#define PCD8544_LCD_H_

#define BLACK 1
#define WHITE 0

#define LCDWIDTH 84
#define LCDHEIGHT 48
#define BUF_SIZE LCDWIDTH*LCDHEIGHT/8

#define PCD8544_POWERDOWN 0x04
#define PCD8544_ENTRYMODE 0x02
#define PCD8544_EXTENDEDINSTRUCTION 0x01

#define PCD8544_DISPLAYBLANK 0x0
#define PCD8544_DISPLAYNORMAL 0x4
#define PCD8544_DISPLAYALLON 0x1
#define PCD8544_DISPLAYINVERTED 0x5

#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_DISPLAYCONTROL 0x08
#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80

#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

#define LCD_NOP asm volatile("nop\n\t""nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" ::);


//========================SPI===============================

#define SPI_HW_SW 0  // 1 - Hardware'owy, 0 - Software'owy
#define USE_CS 1 // 1 - CS podlaczony do procesora, 0 - na stale do masy

#if USE_CS ==1
#define SCE      (1<<PD2)  //CS w programowym i sprzetowym SPI
#define SCE_PORT PORTD
#define SCE_DDR  DDRD

#define SCE_LO SCE_PORT &= ~SCE
#define SCE_HI SCE_PORT |= SCE
#endif

#define DC		(1<<PD3)  //D/C w programowym i sprzetowym SPI
#define DC_PORT PORTD
#define DC_DDR  DDRD

#define DC_LO DC_PORT &= ~DC
#define DC_HI DC_PORT |= DC

#define RES		(1<<PD4)	//RST w programowym i sprzetowym SPI
#define RES_PORT PORTD
#define RES_DDR  DDRD

#define RES_LO RES_PORT &= ~RES
#define RES_HI RES_PORT |= RES

#if SPI_HW_SW == 0
//Makra programowego SPI
#define SCLK     (1<<PD5) //SCK
#define SDIN    (1<<PD6)  //MOSI

#define SCLK_PORT PORTD
#define SCLK_DDR  DDRD

#define SDIN_PORT PORTD
#define SDIN_DDR  DDRD

#define SCLK_LO SCLK_PORT &= ~SCLK
#define SCLK_HI SCLK_PORT |= SCLK

#define SDIN_LO SDIN_PORT &= ~SDIN
#define SDIN_HI SDIN_PORT |= SDIN

#else

#define SPI_DDR DDRB
#define MOSI	PB5
#define SCK		PB7

#endif
//==================================================

#define swap(a, b) {uint16_t t = a; a = b; b = t;}

//Deklaracje zmiennych
const uint8_t *font;
extern int cursor_x, cursor_y;
extern const uint8_t font_5x7;

//Funkcje uzytkownika
void pcd8544_init(); // inicjalizacja sterownika
void pcd8544_setContrast(uint8_t val); // ustawienie kontrastu
void pcd8544_cls(); // czyszczenie bufora
void pcd8544_display(); // wyslanie bufora do wyswietlacza
void pcd8544_invertedMode(); // odwrocone kolory
void pcd8544_normalMode(); //normalne kolory

//Funkcje graficzne
void pcd8544_drawPixel(int x, int y, uint8_t color); // zapalenie piksela w buforze na pozycji (x,y), color 1-czarny, 0-bialy
void pcd8544_drawBitmap_P(int x, int y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color); // rysowanie bitmapy z pamieci flash
void pcd8544_fillRect(int x, int y, int w, int h, uint8_t color); // rysowanie wypelnionego prostokata
void pcd8544_drawLine(int x0, int y0, int x1, int y1, uint8_t color); // rysowanie dowolnej linii
void pcd8544_drawFastHLine(int x, int y, int w, uint8_t color); // rysowanie linii poziomej
void pcd8544_drawFastVLine(int x, int y, int h, uint8_t color); // rysowanie linii pionowej

//Funkcje znakowe i napisow
void pcd8544_drawChar(int x, int y, char c, uint8_t color, uint8_t bg, uint8_t size, const uint8_t *font); // wyswietlenie pojedynczego znaku
void pcd8544_puts(int x, int y, char *str, uint8_t txt_size, uint8_t color, uint8_t bg, const uint8_t *font); // wyswietlanie napisow
void pcd8544_put_int(int x, int y, int data, uint8_t txt_size, uint8_t color, uint8_t bg, const uint8_t *font); // wyswietlenie liczby int

#endif /* PCD8544_LCD_H_ */
