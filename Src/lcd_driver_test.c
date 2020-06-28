/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f446xx_lcd_driver.h"
#include <string.h>
void delay(int time) {
	for(int i = 0 ; i < time ; i++);
}

int main(void){
	lcd_handle_t lcd;
	memset(&lcd, 0, sizeof(lcd_handle_t));
	//gpio_peripheral_clk(GPIOA, ENABLE);
	//gpio_peripheral_clk(GPIOH, ENABLE);
	//gpio_peripheral_clk(GPIOC, ENABLE);
	
    lcd.D4.pgpiox = GPIOC;
	lcd.D4.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
	lcd.D4.gpio_pinconfig.pin_number = GPIO_PIN_12;
	lcd.D4.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_HI;
	lcd.D4.gpio_pinconfig.pin_optype =  GPIO_OP_PUSH_PULL;
	lcd.D4.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;

	lcd.D5.pgpiox = GPIOC;
	lcd.D5.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
	lcd.D5.gpio_pinconfig.pin_number = GPIO_PIN_13;
	lcd.D5.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_HI;
	lcd.D5.gpio_pinconfig.pin_optype =  GPIO_OP_PUSH_PULL;
	lcd.D5.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;

	lcd.D6.pgpiox = GPIOC;
	lcd.D6.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
	lcd.D6.gpio_pinconfig.pin_number = GPIO_PIN_14;
	lcd.D6.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_HI;
	lcd.D6.gpio_pinconfig.pin_optype =  GPIO_OP_PUSH_PULL;
	lcd.D6.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;

	lcd.D7.pgpiox = GPIOC;
	lcd.D7.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
	lcd.D7.gpio_pinconfig.pin_number = GPIO_PIN_15;
	lcd.D7.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_HI;
	lcd.D7.gpio_pinconfig.pin_optype =  GPIO_OP_PUSH_PULL;
	lcd.D7.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;


	lcd.RS.pgpiox = GPIOC;
	lcd.RS.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
	lcd.RS.gpio_pinconfig.pin_number = GPIO_PIN_10;
	lcd.RS.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_HI;
	lcd.RS.gpio_pinconfig.pin_optype =  GPIO_OP_PUSH_PULL;
	lcd.RS.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;


	lcd.EN.pgpiox = GPIOC;
	lcd.EN.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
	lcd.EN.gpio_pinconfig.pin_number = GPIO_PIN_11;
	lcd.EN.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_HI;
	lcd.EN.gpio_pinconfig.pin_optype =  GPIO_OP_PUSH_PULL;
	lcd.EN.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;



	//lcd.function_set = LCD_CMD_FUNCTION_SET | DATA_LEN_4 | DISPLAY_1_LINE | MATRIX_5_X_8;
	//lcd.display_set = LCD_CMD_DISPLAY_CURSOR_ONOFF_CONTROL | DISPLAY_ON | BLINK_CURSOR_ON;
	//lcd.entry_set = LCD_CMD_ENTRY_MODESET|INC_CURSOR;
	lcd_init(&lcd);
	//lcd_cursor_move(&lcd,0,0);
	lcd_data(&lcd,(uint8_t)'A');

	while(1){
		//for (int i = 0 ; i < 4 ; i++){
			//lcd_cursor_move(&lcd, 0,0);
			//lcd_data(&lcd,(uint8_t)'B');
			lcd_command(&lcd, 0x38);
			lcd_command(&lcd, 0x0E);
			lcd_command(&lcd, 0x01); //clear display
			//delay(500000);
		//}
		delay(500000);
	}
	return 0;
}
