/*
 * stm32f446xx_lcd_driver.c
 *
 *  Created on: Dec 2, 2019
 *      Author: vverma
 */


#include "stm32f446xx_lcd_driver.h"

void usleep(int time){
    for(int i = 0 ; i < time; i++){}
}

void lcd_init(lcd_handle_t *plcd){
    if((plcd->function_set & DATA_LEN_8)){
        gpio_init(&plcd->D0);
        gpio_init(&plcd->D1);
        gpio_init(&plcd->D2);
        gpio_init(&plcd->D3);
    }

    gpio_init(&(plcd->RS));
    gpio_init(&(plcd->D4));
    gpio_init(&(plcd->D5));
    gpio_init(&(plcd->D6));
    gpio_init(&(plcd->D7));

    //gpio_init(&(plcd->RW)); needs to be ground
    gpio_init(&(plcd->EN));

    //lcd init
   // lcd_command(plcd, plcd->entry_set);
    //lcd_command(plcd, 0x01); //clear display
    lcd_command(plcd, 0x38); //
    lcd_command(plcd, 0x0E);// display on cursor on
    lcd_command(plcd, 0x01); //clear display
    // usleep(INS_WAIT_TIME);
    //lcd_command(plcd, plcd->function_set);
   // lcd_command(plcd, 0x06);
   // usleep(INS_WAIT_TIME);
   // lcd_command(plcd, plcd->display_set);

}



void lcd_command(lcd_handle_t* plcd, uint8_t ascii_Value){
    //set into command mode
    uint8_t data_msb = ((ascii_Value >> 4) & 0X0f ); // d7 d6 d5 d4
    gpio_write_pin(plcd->D4.pgpiox, plcd->D4.gpio_pinconfig.pin_number,((data_msb>>0) &0x01));
    gpio_write_pin(plcd->D5.pgpiox, plcd->D5.gpio_pinconfig.pin_number,((data_msb>>1) &0x01));
    gpio_write_pin(plcd->D6.pgpiox, plcd->D6.gpio_pinconfig.pin_number,((data_msb>>2) &0x01));
    gpio_write_pin(plcd->D7.pgpiox, plcd->D7.gpio_pinconfig.pin_number,((data_msb>>3) &0x01));
    gpio_write_pin(plcd->RS.pgpiox, plcd->RS.gpio_pinconfig.pin_number, COMMAND_MODE);

    lcd_enable(plcd);

    uint8_t data_lsb = (( ascii_Value & 0x0f )); // d3 d2 d1 d0
    gpio_write_pin(plcd->D4.pgpiox, plcd->D4.gpio_pinconfig.pin_number,((data_lsb>>0) &0x01));
    gpio_write_pin(plcd->D5.pgpiox, plcd->D5.gpio_pinconfig.pin_number,((data_lsb>>1) &0x01));
    gpio_write_pin(plcd->D6.pgpiox, plcd->D6.gpio_pinconfig.pin_number,((data_lsb>>2) &0x01));
    gpio_write_pin(plcd->D7.pgpiox, plcd->D7.gpio_pinconfig.pin_number,((data_lsb>>3) &0x01));
    gpio_write_pin(plcd->RS.pgpiox, plcd->RS.gpio_pinconfig.pin_number, COMMAND_MODE);
    lcd_enable(plcd);

    /*
    if((plcd->function_set & DATA_LEN_8)){
        lcd_write8(plcd, command);
    }else{
        lcd_write(plcd, command);
    }
    //usleep(5 * 1000);
     *
     */

}

void lcd_data(lcd_handle_t* plcd, uint8_t ascii_Value){
    uint8_t data_msb = ((ascii_Value >> 4) & 0X0f ); // d7 d7 d5 d4
    gpio_write_pin(plcd->D4.pgpiox, plcd->D4.gpio_pinconfig.pin_number,((data_msb>>0) &0x01));
    gpio_write_pin(plcd->D5.pgpiox, plcd->D5.gpio_pinconfig.pin_number,((data_msb>>1) &0x01));
    gpio_write_pin(plcd->D6.pgpiox, plcd->D6.gpio_pinconfig.pin_number,((data_msb>>2) &0x01));
    gpio_write_pin(plcd->D7.pgpiox, plcd->D7.gpio_pinconfig.pin_number,((data_msb>>3) &0x01));
    gpio_write_pin(plcd->RS.pgpiox, plcd->RS.gpio_pinconfig.pin_number, DATA_MODE);

    lcd_enable(plcd);

        uint8_t data_lsb = (( ascii_Value & 0x0f )); // d3 d2 d1 d0
        gpio_write_pin(plcd->D4.pgpiox, plcd->D4.gpio_pinconfig.pin_number,((data_lsb>>0) &0x01));
        gpio_write_pin(plcd->D5.pgpiox, plcd->D5.gpio_pinconfig.pin_number,((data_lsb>>1) &0x01));
        gpio_write_pin(plcd->D6.pgpiox, plcd->D6.gpio_pinconfig.pin_number,((data_lsb>>2) &0x01));
        gpio_write_pin(plcd->D7.pgpiox, plcd->D7.gpio_pinconfig.pin_number,((data_lsb>>3) &0x01));
        gpio_write_pin(plcd->RS.pgpiox, plcd->RS.gpio_pinconfig.pin_number, DATA_MODE);
        lcd_enable(plcd);
        /*

        if((plcd->function_set & DATA_LEN_8)){
            lcd_write8(plcd, data);
        }else{
            lcd_write(plcd, data);
        }
        //usleep(5 * 1000);

            */
}

/*
 *
 * */
void lcd_print_char(lcd_handle_t* plcd, uint8_t value){
    lcd_data(plcd, value);
}

/*
 *
 * */
void lcd_print_str(lcd_handle_t* plcd, char *message){
    while (*message != '\0') {
         lcd_print_char(plcd,(uint8_t)*message++);
      }
}

/*
 *
 * */
void lcd_enable(lcd_handle_t* plcd){
    gpio_write_pin(plcd->EN.pgpiox, plcd->EN.gpio_pinconfig.pin_number, LCD_ENABLE);
    usleep(4*100000); //2ms delay
    gpio_write_pin(plcd->EN.pgpiox, plcd->EN.gpio_pinconfig.pin_number, LCD_DISABLE);
    usleep(4*100000);
}
/*
 *
 * */
void lcd_cursor_move(lcd_handle_t* plcd, uint8_t row , uint8_t column){
    column--;
      switch (row)
      {
        case 1:
          /* Set cursor to 1st row address and add index*/
          lcd_command(plcd,column |= DDRAM_FIRST_LINE_BASE_ADDR);
          break;
        case 2:
          /* Set cursor to 2nd row address and add index*/
            lcd_command(plcd, column |= DDRAM_SECOND_LINE_BASE_ADDR);
          break;
        default:
          break;
      }

}


/*
 *
 *
 * */

void lcd_load_cgram(lcd_handle_t* plcd, char tab[], uint8_t charnum){

}
