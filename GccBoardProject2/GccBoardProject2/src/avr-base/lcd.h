/*
 * LCD interface header file
 * See lcd.c for more info
 *
 * Copywrite Craig Lee 1998
 *
 * Modified by Alex Shepherd to be compatible with previous LCD API
 */

#include "sysdef.h"

/*** [BA040321] added ifdef for different PCB. Value defined in sysdef.h ****/
#if defined BOARD_LOCO_DEV //EmbeddedLocoNet 
#define LCD_DATA_PORT    PORTC     /* port for RS line */
#define LCD_DATA_DDR     DDRC      /* port for RS line */

#define LCD_DATA_D4_PIN  1
#define LCD_DATA_D5_PIN  0
#define LCD_DATA_D6_PIN  3
#define LCD_DATA_D7_PIN  2

#define LCD_RS_PORT      PORTD     /* port for RS line */
#define LCD_RS_DDR       DDRD      /* port for RS line */
#define LCD_RS_PIN       5

#define LCD_E_PORT       PORTD  /* port for Enable line */
#define LCD_E_DDR        DDRD      /* port for RS line */
#define LCD_E_PIN        4

#elif defined BOARD_PROTO_128 //ProtoBoardMega128
#define LCD_DATA_PORT    PORTC     /* port for RS line */
#define LCD_DATA_DDR     DDRC      /* port for RS line */

#define LCD_DATA_D4_PIN  0
#define LCD_DATA_D5_PIN  1
#define LCD_DATA_D6_PIN  2
#define LCD_DATA_D7_PIN  3

#define LCD_RS_PORT      PORTC     /* port for RS line */
#define LCD_RS_DDR       DDRC      /* port for RS line */
#define LCD_RS_PIN       5

#define LCD_E_PORT       PORTC  /* port for Enable line */
#define LCD_E_DDR        DDRC      /* port for RS line */
#define LCD_E_PIN        4

/*** [DH040609] added OmniPort Board definitions for OmniLinx Project */
#elif defined BOARD_OMNIPORT_OMNILINX // OmniPort OmniLinx Project
#define LCD_DATA_PORT    PORTB     /* port for Data lines */
#define LCD_DATA_DDR     DDRB

#define LCD_DATA_D4_PIN  0
#define LCD_DATA_D5_PIN  1
#define LCD_DATA_D6_PIN  2
#define LCD_DATA_D7_PIN  3

#define LCD_RS_PORT      PORTB    /* port for RS line */
#define LCD_RS_DDR       DDRB
#define LCD_RS_PIN       5

#define LCD_E_PORT       PORTB    /* port for Enable line */
#define LCD_E_DDR        DDRB
#define LCD_E_PIN        4

#elif defined BOARD_DT006_MEGA16 //Dontronics DT006 + SIMM100 with Mega16
#define LCD_DATA_PORT    PORTC     /* port for RS line */
#define LCD_DATA_DDR     DDRC      /* port for RS line */

#define LCD_DATA_D4_PIN  2
#define LCD_DATA_D5_PIN  3
#define LCD_DATA_D6_PIN  4
#define LCD_DATA_D7_PIN  5

#define LCD_RS_PORT      PORTC     /* port for RS line */
#define LCD_RS_DDR       DDRC      /* port for RS line */
#define LCD_RS_PIN       1

#define LCD_E_PORT       PORTC  /* port for Enable line */
#define LCD_E_DDR        DDRC      /* port for RS line */
#define LCD_E_PIN        0

#elif defined BOARD_DEFINED_IN_SYSDEF

#else //No Board defined (Error)
#    warning "Board not defined"
#endif //Boardtype

/* instruction register bit positions */
#define LCD_CLR             0      /* DB0: clear display */
#define LCD_HOME            1      /* DB1: return to home position */
#define LCD_ENTRY_MODE      2      /* DB2: set entry mode */
#define LCD_ENTRY_INC       1      /* DB1: 1=increment, 0=decrement  */
#define LCD_ENTRY_SHIFT     2      /* DB2: 1=display shift on        */
#define LCD_ON              3      /* DB3: turn lcd/cursor on */
#define LCD_ON_DISPLAY      2      /* DB2: turn display on */
#define LCD_ON_CURSOR       1      /* DB1: turn cursor on */
#define LCD_ON_BLINK        0      /* DB0: blinking cursor ? */
#define LCD_MOVE            4      /* DB4: move cursor/display */
#define LCD_MOVE_DISP       3      /* DB3: move display (0-> cursor) ? */
#define LCD_MOVE_RIGHT      2      /* DB2: move right (0-> left) ? */
#define LCD_FUNCTION        5      /* DB5: function set */
#define LCD_FUNCTION_8BIT   4      /* DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_FUNCTION_2LINES 3      /* DB3: two lines (0->one line) */
#define LCD_FUNCTION_10DOTS 2      /* DB2: 5x10 font (0->5x7 font) */
#define LCD_CGRAM           6      /* DB6: set CG RAM address */
#define LCD_DDRAM           7      /* DB7: set DD RAM address */
#define LCD_BUSY            7      /* DB7: LCD is busy */

/* set entry mode: display shift on/off, dec/inc cursor move direction */
#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */
#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */
#define LCD_ENTRY_INC_           0x06   /* display shift off, inc cursor move dir */
#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */

/* display on/off, cursor on/off, blinking char at cursor position */
#define LCD_DISP_OFF             0x08   /* display off                            */
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */

/* move cursor/shift display */
#define LCD_MOVE_CURSOR_LEFT     0x10   /* move cursor left  (decrement)          */
#define LCD_MOVE_CURSOR_RIGHT    0x14   /* move cursor right (increment)          */
#define LCD_MOVE_DISP_LEFT       0x18   /* shift display left                     */
#define LCD_MOVE_DISP_RIGHT      0x1C   /* shift display right                    */

/* function set: set interface data length and number of display lines */
#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */
#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */

#define LCD_START_LINE1  0x00     /* DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /* DDRAM address of first char of line 2 */

#define LCD_MODE_DEFAULT     ((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )

/* write a byte to the LCD in 4 bit mode */

extern void lcd_write(unsigned char);

/* Clear and home the LCD */

extern void lcd_clear(void);

/* write a string of characters to the LCD */

extern void lcd_puts(const char * s);

/* Go to the specified position */

extern void lcd_goto(unsigned char x, unsigned char y);
 
extern void lcd_clrxy(unsigned char x, unsigned char y, unsigned char count);

/* write a character to the LCD */

extern void lcd_putc(char);
extern void lcd_word(unsigned short value, unsigned char width);

/* intialize the LCD - call before anything else */

extern void lcd_init(unsigned char dispAttr);

