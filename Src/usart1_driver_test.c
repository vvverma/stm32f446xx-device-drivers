/*
 * usart_driver_test.c
 *
 * Created on: Apr 9, 2020
 * Author: vvverma
 */

#include<stdio.h>
#include<string.h>
#include "stm32f446xx.h"

usart_handle_t usart1_handle;
char tx_msg[1024] = "USART1 PB7,PA10: Receive(RX) PB6,PA9: Transmit(TX) Testing...\n\r";
char msg_start[1] = "#";
char rx_msg[1024];

void usart1_init(void) {
    usart1_handle.pusartx = USART1;
    usart1_handle.usart_config.baudrate= USART_STD_BAUD_115200;
    usart1_handle.usart_config.hw_flow_ctrl= USART_HW_FLOW_CTRL_NONE;
    usart1_handle.usart_config.mode= USART_MODE_TXRX;
    usart1_handle.usart_config.stopbit_len=USART_STOPBIT_ONE;
    usart1_handle.usart_config.word_len= USART_DATA_LEN_8_BIT;
    usart1_handle.usart_config.parity_ctrl = USART_PARITY_DISABLE;
    usart_init(&usart1_handle);
}

void usart1_gpio_init(void) {
    gpio_handle_t usart_gpios;

    usart_gpios.pgpiox = GPIOA;
    usart_gpios.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_ALT_FUN;
    usart_gpios.gpio_pinconfig.pin_optype = GPIO_OP_PUSH_PULL;
    usart_gpios.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_PU; //NO_PUPD;
    usart_gpios.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_FAST;;
    usart_gpios.gpio_pinconfig.pin_altfun_mode=7;

    //USART2 TX PA9
    usart_gpios.gpio_pinconfig.pin_number = GPIO_PIN_9;
    //USART2 TX PB6
    //usart_gpios.gpio_pinconfig.pin_number = GPIO_PIN_6;
    gpio_init(&usart_gpios);

    //USART2 RX PA9
    usart_gpios.gpio_pinconfig.pin_number = GPIO_PIN_10;
    //USART2 RX PB7
    //usart_gpios.gpio_pinconfig.pin_number = GPIO_PIN_7;
    gpio_init(&usart_gpios);

}

void gpio_button_init(void)
{
    gpio_handle_t gpio_btn,gpio_led;

    //this is btn gpio configuration
    gpio_btn.pgpiox = GPIOC;
    gpio_btn.gpio_pinconfig.pin_number = GPIO_PIN_13;
    gpio_btn.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_IN;
    gpio_btn.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_FAST;
    gpio_btn.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;

    gpio_init(&gpio_btn);

    //this is led gpio configuration
    gpio_led.pgpiox = GPIOA;
    gpio_led.gpio_pinconfig.pin_number = GPIO_PIN_5;
    gpio_led.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
    gpio_led.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_MED;
    gpio_led.gpio_pinconfig.pin_optype = GPIO_OP_PUSH_PULL;
    gpio_led.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;
    gpio_peripheral_clk(GPIOA,ENABLE);
    gpio_init(&gpio_led);

}

void delay(void) {
    for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void) {
    char test;
    gpio_button_init();

    usart1_gpio_init();

    usart1_init();

    usart_peripheral_control(USART1,ENABLE);
    usart_tx(&usart1_handle,(uint8_t*)tx_msg,strlen(tx_msg));
    while(1)
    {
        //wait till button is pressed
        while (gpio_read_pin(GPIOC,GPIO_PIN_13));
        //to avoid button de-bouncing related issues 200ms of delay
        //delay();
        //gpio_toggle_pin(GPIOA, 5);
        //usart_tx(&usart1_handle,(uint8_t*)msg_start,strlen(msg_start));
        usart_rx(&usart1_handle,(uint8_t*)rx_msg,1);
        test = *rx_msg;
        usart_tx(&usart1_handle,(uint8_t*)rx_msg,1);
        if(test == '\b'){
            *rx_msg = ' ';
            usart_tx(&usart1_handle,(uint8_t*)rx_msg,1);
            *rx_msg = '\b';
            usart_tx(&usart1_handle,(uint8_t*)rx_msg,1);
        }
        else if (test == '\r') {
            *rx_msg = '\n';
            usart_tx(&usart1_handle,(uint8_t*)rx_msg,1);
        }
    }

    return 0;
}
