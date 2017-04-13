/*
 *	LCD interface example
 *	Uses routines from delay.c
 *	This code will interface to a standard LCD controller
 *	like the Hitachi HD44780. It uses it in 4 bit mode.
 *  Copywrite Craig Lee 1998
 *	
 *	Ported to AVR-GCC by Alex Shepherd and modified to be compatible with previous LCD API
 */

#include <avr/io.h>
#include <string.h>
#include <stdlib.h>

#include	"lcd.h"

/*** [DH040609] Made enable and strobe more general */
#define LCD_E_INIT sbi(LCD_E_DDR, LCD_E_PIN);
#define LCD_E_ON LCD_E_PORT |= _BV(LCD_E_PIN)
#define LCD_E_OFF LCD_E_PORT &= ~_BV(LCD_E_PIN)
#define	LCD_STROBE	LCD_E_ON; LCD_E_OFF  			
// The above definitions were changed to allow the use of multiple bits to enable the LCD, 
// so, for example, #define LCD_E_ON LCD_E_PORT |= _BV(LCD_E_PIN1 | _BV(LCD_E_PIN2

/* previous definition 
#define	LCD_STROBE	sbi(LCD_E_PORT, LCD_E_PIN); cbi(LCD_E_PORT, LCD_E_PIN)
*/


// Note: this requires data bits to be in the same register
#define LCD_DATA_BITS (_BV(LCD_DATA_D7_PIN) | _BV(LCD_DATA_D6_PIN) | _BV(LCD_DATA_D5_PIN) | _BV(LCD_DATA_D4_PIN))

#define TIME_S1_MS    ( 1*(F_CPU/4000) ) 
#define delay_us(us)  delay_short( ((TIME_S1_MS*us)/1000) )

void delay_short(unsigned short number_of_loops) 
{
/* 4 cpu cycles per loop + 12 cycles overhead when a constant is passed. */
  __asm__ volatile ( "cp  %A0,__zero_reg__ \n\t"  \
                     "cpc %B0,__zero_reg__ \n\t"  \
                     "breq L_EXIT_%=       \n\t"  \
                     "L_%=:                \n\t"  \
                     "sbiw r24,1           \n\t"  \
                     "brne L_%=            \n\t"  \
                     "L_EXIT_%=:           \n\t"  \
                     : /* NO OUTPUT */            \
                     : "w" (number_of_loops)      \
                   );                            

}

/* write a byte to the LCD in 4 bit mode */

void lcd_write(unsigned char c)
{
	unsigned char dataBits ;

	dataBits = LCD_DATA_PORT & ~LCD_DATA_BITS ;
	if(c & 0x80) dataBits |= _BV(LCD_DATA_D7_PIN) ;
	if(c & 0x40) dataBits |= _BV(LCD_DATA_D6_PIN) ;
	if(c & 0x20) dataBits |= _BV(LCD_DATA_D5_PIN) ;
	if(c & 0x10) dataBits |= _BV(LCD_DATA_D4_PIN) ;
  LCD_DATA_PORT = dataBits ;
	LCD_STROBE ;

	dataBits &= ~LCD_DATA_BITS ;
	if(c & 0x08) dataBits |= _BV(LCD_DATA_D7_PIN) ;
	if(c & 0x04) dataBits |= _BV(LCD_DATA_D6_PIN) ;
	if(c & 0x02) dataBits |= _BV(LCD_DATA_D5_PIN) ;
	if(c & 0x01) dataBits |= _BV(LCD_DATA_D4_PIN) ;
  LCD_DATA_PORT = dataBits ;
	LCD_STROBE ;
	delay_us(160) ;
}

/*
 * 	Clear and home the LCD
 */

void lcd_clear(void)
{
	cbi(LCD_RS_PORT, LCD_RS_PIN);

	lcd_write(0x1);
	delay_us(4100);
}

/* write a string of chars to the LCD */

void lcd_puts(const char * s)
{
  sbi(LCD_RS_PORT, LCD_RS_PIN);

	while(*s) lcd_write(*s++);
}

/* write one character to the LCD */

void lcd_putc(char c)
{
  sbi(LCD_RS_PORT, LCD_RS_PIN);

	lcd_write(c);
}


/*
 * Go to the specified position
 */

void lcd_goto(unsigned char x, unsigned char y)
{
  cbi(LCD_RS_PORT, LCD_RS_PIN);

	if( y )
		x += LCD_START_LINE2 ;

	lcd_write( _BV(LCD_DDRAM) | x);
}
	
void lcd_clrxy(unsigned char x, unsigned char y, unsigned char count)
{
	lcd_goto( x, y ) ;

	while( count-- )
		lcd_putc( ' ' ) ;

	lcd_goto( x, y ) ;
}

void lcd_word(unsigned short value, unsigned char width)
{
  unsigned char outstr[11] ;
  unsigned char len ;
	unsigned char fill = ' ' ;

	if( width > 5 )
	{
		width -= 10 ;
		fill = '0' ;
	}

  memset( outstr, fill, 11 ) ;

  len = strlen( utoa( value, outstr + 5, 10 ) ) ;

  lcd_puts( outstr + len + 5 - width) ;
}

/* initialise the LCD - put into 4 bit mode */

void lcd_init(unsigned char dispAttr)
{
  sbi(LCD_RS_DDR, LCD_RS_PIN);
  LCD_E_INIT;
  LCD_DATA_DDR |= LCD_DATA_BITS ;
  cbi(LCD_RS_PORT, LCD_RS_PIN);
  LCD_E_OFF;

		// power on delay
	delay_us(16000);

		// Initially configure for 8 bit mode
	sbi(LCD_DATA_PORT, LCD_DATA_D4_PIN ) ;
	sbi(LCD_DATA_PORT, LCD_DATA_D5_PIN ) ;
	LCD_STROBE;
	delay_us(5000);

		// Repeat last command
	LCD_STROBE;
	delay_us(160);

		// Repeat last command a third time
	LCD_STROBE;
	delay_us(160);

		// Now configure for 4 bit mode
	cbi(LCD_DATA_PORT, LCD_DATA_D4_PIN ) ;
	LCD_STROBE;
	delay_us(160);

  lcd_write(LCD_FUNCTION_4BIT_2LINES);
  lcd_write(LCD_DISP_OFF);
  lcd_clear();
	lcd_write(LCD_MODE_DEFAULT);

		// entry mode advance cursor
	lcd_write(LCD_ENTRY_INC_);

		// set passed display attributes
	lcd_write(dispAttr);
}
