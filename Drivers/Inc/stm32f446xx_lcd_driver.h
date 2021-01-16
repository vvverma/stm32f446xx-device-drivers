/*
 * stm32f446xx_lcd_driver.h
 *
 *  Created on: Dec 2, 2019
 *      Author: vverma
 */

#ifndef INC_STM32F446XX_LCD_DRIVER_H_
#define INC_STM32F446XX_LCD_DRIVER_H_

#include "stm32f446xx.h"

typedef struct {
	/*Control Pins*/
	gpio_handle_t RS; 		/*Register Select*/
	//gpio_handle_t RW;		/*Read active high / Write active low*/
	gpio_handle_t EN;		/*Enable to latch the data*/

	/*Data pins*/
	gpio_handle_t D0;		/*Data bit 0*/
	gpio_handle_t D1;		/*Data bit 1*/
	gpio_handle_t D2;		/*Data bit 2*/
	gpio_handle_t D3;		/*Data bit 3*/
	gpio_handle_t D4;		/*Data bit 4*/
	gpio_handle_t D5;		/*Data bit 5*/
	gpio_handle_t D6;		/*Data bit 6*/
	gpio_handle_t D7;		/*Data bit 7*/

	uint8_t function_set;		/*	*/
	uint8_t entry_set;			/**/
	uint8_t display_set;			/**/

}lcd_handle_t;


#define COMMAND_MODE       		 RESET
#define DATA_MODE           	 SET

/*************************************LCD COMMAND SETS **********************************************/

/*Clears entire display and sets DDRAM address 0 in
address counter .. clears display fills up DDRAM with 20H ..
thats the ASCII value for black character or "space"*/
#define LCD_CMD_CLEAR_DISPLAY  0x01

/*Sets DDRAM address 0 in address counter. Also returns display from being shifted to original position.
DDRAM contents remain unchanged. */
#define LCD_CMD_RETURN_HOME 0x02


/* Sets cursor move direction and specifies display shift. These operations are performed during data write
and read. */
#define LCD_CMD_ENTRY_MODESET   			 0X04

#define INC_CURSOR     						( 1 << 1)
#define DEC_CURSOR    						 (LCD_CMD_ENTRY_MODESET & ~(INC_CURSOR))
#define ACCOMPANY_DISPLAY_SHIFT            	( 1 << 0)
#define DO_NOT_ACCOMPANY_DISPLAY_SHIFT      (LCD_CMD_ENTRY_MODESET & ~(ACCOMPANY_DISPLAY_SHIFT))


/*Sets entire display (D) on/off, cursor on/off (C), and blinking of cursor position character (B).*/
#define LCD_CMD_DISPLAY_CURSOR_ONOFF_CONTROL  0x08

#define DISPLAY_ON    						(1 << 2)
#define DISPLAY_OFF   						(LCD_CMD_DISPLAY_CURSOR_ONOFF_CONTROL & ~(DISPLAY_ON))

#define CURSOR_ON    						(1 << 1)
#define CURSOR_OFF  						(LCD_CMD_DISPLAY_CURSOR_ONOFF_CONTROL & ~(CURSOR_ON))

#define BLINK_CURSOR_ON 	 				(1 << 0)
#define BLINK_CURSOR_OFF  					(LCD_CMD_DISPLAY_CURSOR_ONOFF_CONTROL & ~(BLINK_CURSOR_ON))


/* Moves cursor and shifts display without changing DDRAM contents*/
#define LCD_CMD_CURSOR_DISPLAY_SHIFT_CONTROL  0x10

#define DISPLAY_SHIFT   					  ( 1 << 3)
#define CURSOR_MOVE     					  (LCD_CMD_CURSOR_DISPLAY_SHIFT_CONTROL & ~(DISPLAY_SHIFT))

#define SHIFT_TO_RIGHT  					( 1 << 2)
#define SHIFT_TO_LEFT   					 (LCD_CMD_CURSOR_DISPLAY_SHIFT_CONTROL & ~(SHIFT_TO_RIGHT))


/*Sets interface data length (DL), number of display lines (N), and character font (F). */
#define LCD_CMD_FUNCTION_SET  				0x20

#define DATA_LEN_8  						( 1 << 4)
#define DATA_LEN_4  						(LCD_CMD_FUNCTION_SET & ~(DATA_LEN_8))
#define DISPLAY_2_LINES 				 	( 1 << 3)
#define DISPLAY_1_LINE  					(LCD_CMD_FUNCTION_SET & ~(DISPLAY_2_LINES))
#define MATRIX_5_X_10 						( 1 << 2)
#define MATRIX_5_X_8 						(LCD_CMD_FUNCTION_SET & ~(MATRIX_5_X_10))


/*Sets CGRAM address. CGRAM data is sent and received after this setting. */
#define LCD_CMD_SET_CGRAM_ADDRESS  			0x40

/* Sets DDRAM address. DDRAM data is sent and received after this setting. */
#define LCD_CMD_SET_DDRAM_ADDRESS  			0x80

#define DDRAM_SECOND_LINE_BASE_ADDR         (LCD_CMD_SET_DDRAM_ADDRESS | 0x40 )
#define DDRAM_FIRST_LINE_BASE_ADDR          LCD_CMD_SET_DDRAM_ADDRESS


#define LCD_ENABLE 1
#define LCD_DISABLE 0

/* HD44780 CGRAM address start */
#define CGRAM_address_start 0x40

#define INS_WAIT_TIME ( 8 * 10000)


/*
 * LCD Driver API
 */

void lcd_init(lcd_handle_t *plcd);
void lcd_command(lcd_handle_t *plcd, uint8_t command);
void lcd_print_char(lcd_handle_t *plcd, uint8_t value);
void lcd_print_str(lcd_handle_t *plcd, char *message);
void lcd_data(lcd_handle_t* plcd, uint8_t data);
void lcd_enable(lcd_handle_t *plcd);
void lcd_cursor_move(lcd_handle_t *plcd, uint8_t row , uint8_t columns);
void lcd_load_cgram(lcd_handle_t *plcd, char tab[], uint8_t charnum);


#endif /* INC_STM32F446XX_LCD_DRIVER_H_ */
