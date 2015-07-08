/*
 * PCD8544_LCD.c
 *
 *  Created on: 11-02-2015
 *      Author: Mefiu
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>

#include "PCD8544_LCD.h"


static uint8_t pcd8544_bufor[BUF_SIZE];// = {}; //bufor wyswietlacza, zajmuje ok 1kB RAM
const uint8_t *font;
int cursor_x, cursor_y;

static void Delay ( void );

//inicjalizacja pinow SPI
void pcd8544_initSPI()
{
	DC_DDR  |= DC;
	DC_LO;

#if USE_CS == 1
	SCE_DDR  |= SCE;
	SCE_HI;
#endif

	RES_DDR  |= RES;
	RES_HI;

#if SPI_HW_SW == 0
	SCLK_DDR  |= SCLK;
	SCLK_LO;

	SDIN_DDR  |= SDIN;
	SDIN_LO;
#else
	/* Set MOSI and SCK output, all others input */
	SPI_DDR = (1<<MOSI)|(1<<SCK);
	/* Enable SPI, Master, set clock rate fck/4 */

	SPCR = (1<<SPE)|(1<<MSTR);  //|(1<<SPR0); fck/16 , Wyswietlacz toleruje max 4Mhz
#endif
}

//funkcja wysylania po SPI
void SPIwrite(uint8_t a)
{
#if SPI_HW_SW == 0
	uint8_t x;
	SCLK_LO;

#if USE_CS == 1
	SCE_LO;
#endif
	for(x=0x80; x; x>>=1)
	{

		if(a & x)
			SDIN_HI;
		else
			SDIN_LO;

		SCLK_HI;
	//	LCD_NOP;
		SCLK_LO;
	}

#else
	/* Start transmission */
	SPDR = a;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
#endif

#if USE_CS == 1
	SCE_HI;
#endif
}

//przeslanie komendy
void pcd8544_cmd( uint8_t cmd )
{
	DC_LO;
	SPIwrite(cmd);
}

//przeslanie bajtu danych
void pcd8544_data( uint8_t dat )
{
	DC_HI;
	SPIwrite(dat);
}

//ustawienie kontrastu
void pcd8544_setContrast(uint8_t val)
{
  if (val > 0x7f) val = 0x7f;

  pcd8544_cmd(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );
  pcd8544_cmd( PCD8544_SETVOP | val);
  pcd8544_cmd(PCD8544_FUNCTIONSET);

}

void pcd8544_invertedMode()
{
	pcd8544_cmd(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYINVERTED);
}

void pcd8544_normalMode()
{
	pcd8544_cmd(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

//inicjalizacja sterownika
void pcd8544_init()
{
	pcd8544_initSPI(); //inicjacja SPI

	RES_LO;
	Delay();
	RES_HI;

//	pcd8544_cmd(0x21); //komendy inicjalizujace
//	pcd8544_cmd(0xc8);
//	pcd8544_cmd(0x06);
//	pcd8544_cmd(0x13);
//	pcd8544_cmd(0xa5);
//	pcd8544_cmd(0x20);
//	pcd8544_cmd(0x0c);

	pcd8544_cmd(0x21);
	pcd8544_cmd(0xC0);
	pcd8544_cmd(0x06);
	pcd8544_cmd(0x13);
	pcd8544_cmd(0x20);
	pcd8544_cmd(0x0C);

//	 Cos tam dzialalo
//	pcd8544_cmd(0x21);
//	pcd8544_cmd(0xc8);
//	pcd8544_cmd(0x13);
//	pcd8544_cmd(0x20);
//	pcd8544_cmd(0x09);
//	_delay_ms(50);
//	pcd8544_cmd(0x08);
//	_delay_ms(10);
//	pcd8544_cmd(0x0C);


//	pcd8544_cmd(0x21);
//	pcd8544_cmd(0x90);
//	pcd8544_cmd(0x20);
//	pcd8544_cmd(0x0C);

	pcd8544_setContrast(0x3f);
	pcd8544_cls();
	pcd8544_display();
}

//wyczyszczenie bufora
void pcd8544_cls()
{
	memset(pcd8544_bufor, 0x00, BUF_SIZE);
	//cursor_y = cursor_x = 0;
}

//przeslanie bufora do wyswietlacza
void pcd8544_display()
{
	pcd8544_cmd(0x40); // Y = 0
	pcd8544_cmd(0x80); // X = 0

	for(uint16_t i=0; i<BUF_SIZE; i++)
	{
		pcd8544_data(pcd8544_bufor[i]); //przeslanie calego bufora
	}
}

//podstawowa funkcja zapalania pojedynczego piksela
void pcd8544_drawPixel(int x, int y, uint8_t color)
{
	if ((x < 0) || (x >= LCDWIDTH) || (y < 0) || (y >= LCDHEIGHT))
	    return;

	if(color) pcd8544_bufor[x+ (y/8)*LCDWIDTH] |= (1<<(y%8));
	else pcd8544_bufor[x+ (y/8)*LCDWIDTH] &= ~(1<<(y%8));
}

// rysowanie bitmapy z pamieci flash
void pcd8544_drawBitmap_P(int x, int y, const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color)
{
	int i, j, byteWidth = (w + 7)/8;

	for(j=0; j<h; j++) {
	    for(i=0; i<w; i++ ) {
	      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
	        pcd8544_drawPixel(x+i, y+j, color);
	      }
	    }
	  }
}

//rysowanie pojedynczego znaku
void pcd8544_drawChar(int x, int y, char c, uint8_t color, uint8_t bg, uint8_t size, const uint8_t *font)
{
	  if((x >= LCDWIDTH) || (y >= LCDHEIGHT) || ((x + 6 * size - 1) < 0) || ((y + 8 * size - 1) < 0))
	    return;

	  uint8_t line;

	  for (int8_t i=0; i<6; i++ )
	  {
	      if (i == 5) line = 0x0;
	      else line = pgm_read_byte(font+(c*5)+i);

	      for (int8_t j = 0; j<8; j++)
	      {
	        if (line & 0x1)
	        {
	          if (size == 1)
	        	  pcd8544_drawPixel(x+i, y+j, color);
	          else
	          {
	        	  pcd8544_fillRect(x+(i*size), y+(j*size), size, size, color);
	          }
	        }
	        else if (bg != color)
	        {
	          if (size == 1)
	        	  pcd8544_drawPixel(x+i, y+j, bg);
	          else
	          {
	        	  pcd8544_fillRect(x+i*size, y+j*size, size, size, bg);
	          }
	        }
	        line >>= 1;
	      }
	   }
}

// rysowanie wypelnionego prostokata
void pcd8544_fillRect(int x, int y, int w, int h, uint8_t color)
{
	  for (int8_t i=x; i<x+w; i++)
	    pcd8544_drawFastVLine(i, y, h, color);
}

// rysowanie linii pionowej
void pcd8544_drawFastVLine(int x, int y, int h, uint8_t color)
{
	pcd8544_drawLine(x, y, x, y+h-1, color);
}

// rysowanie pojedynczej linii, algorytm Bresenhama
void pcd8544_drawLine(int x0, int y0, int x1, int y1, uint8_t color)
{
	int steep = abs(y1 -y0) > abs(x1 - x0);
	        if (steep)
	        {
	        	swap(x0, y0);
	        	swap(x1, y1);
	        }

	        if (x0 > x1)
	        {
	        	swap(x0, x1);
	        	swap(y0, y1);
	        }

	        int dx, dy;
	        dx = x1 - x0;
	        dy = abs(y1 - y0);

	        int err = dx / 2;
	        int ystep;

	        if (y0 < y1)
	        {
	        	ystep = 1;
	        }
	        else
	        {
	        	ystep = -1;
	        }

	        for (; x0 <= x1; x0++)
	        {
	        	if (steep)
	        	{
	        		pcd8544_drawPixel(y0, x0, color);
	        	}
	        	else
	        	{
	        		pcd8544_drawPixel(x0, y0, color);
	        	}
	            err -= dy;
	            if (err < 0)
	            {
	            	y0 += ystep;
	            	err += dx;
	            }
	        }
}

void pcd8544_drawFastHLine(int x, int y, int w, uint8_t color)
{
  pcd8544_drawLine(x, y, x+w-1, y, color);
}

void pcd8544_puts(int x, int y, char *str, uint8_t txt_size, uint8_t color, uint8_t bg, const uint8_t *font)
{
	cursor_x = x; cursor_y = y;

	while( *str )
	{
		pcd8544_drawChar(cursor_x, cursor_y, *str++, color, bg, txt_size, font);
		cursor_x += 6*txt_size;
	}
}

void pcd8544_put_int(int x, int y, int data, uint8_t txt_size, uint8_t color, uint8_t bg, const uint8_t *font)
{
	char buf[16];
	pcd8544_puts(x, y, itoa(data, buf, 10), txt_size, color, bg, font);
}

static void Delay ( void )
{
    int i;

    for ( i = -32000; i < 32000; i++ );
}
