/*
 * usart3_driver_test.c
 *
 *  Created on: Apr 10, 2020
 *      Author: vvverma
 */

#include<stdio.h>
#include<string.h>
#include "stm32f446xx.h"

usart_handle_t usart3_handle;
char msg[1024] = "USART3 PC11,PB11: Receive(RX) PC10,PB10: Transmit(TX) Testing...\n\r";


void USART2_Init(void) {
	usart3_handle.pusartx = USART3;
	usart3_handle.usart_config.baudrate= USART_STD_BAUD_115200;
	usart3_handle.usart_config.hw_flow_ctrl= USART_HW_FLOW_CTRL_NONE;
	usart3_handle.usart_config.mode= USART_MODE_TX_ONLY;
	usart3_handle.usart_config.stopbit_len=USART_STOPBIT_ONE;
	usart3_handle.usart_config.word_len= USART_DATA_LEN_8_BIT;
	usart3_handle.usart_config.parity_ctrl = USART_PARITY_DISABLE;
	usart_init(&usart3_handle);
}

void USART2_GPIOInit(void) {
	gpio_handle_t usart_gpios;

	usart_gpios.pgpiox = GPIOB;
	usart_gpios.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_ALT_FUN;
	usart_gpios.gpio_pinconfig.pin_optype = GPIO_OP_PUSH_PULL;
	usart_gpios.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_PU; //NO_PUPD;
	usart_gpios.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_FAST;;
	usart_gpios.gpio_pinconfig.pin_altfun_mode=7;

	/*USART2 TX PC10,PB10*/
	usart_gpios.gpio_pinconfig.pin_number = GPIO_PIN_10;
	gpio_init(&usart_gpios);

	/*USART2 RX PC11,PB11*/
	usart_gpios.gpio_pinconfig.pin_number = GPIO_PIN_11;
	gpio_init(&usart_gpios);
}

void GPIO_ButtonInit(void) {
	gpio_handle_t GPIOBtn, GpioLed;

	//this is Button gpio configuration
	GPIOBtn.pgpiox = GPIOC;
	GPIOBtn.gpio_pinconfig.pin_number = GPIO_PIN_13;
	GPIOBtn.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_IN;
	GPIOBtn.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_FAST;
	GPIOBtn.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;
	gpio_init(&GPIOBtn);

	//this is led gpio configuration COMMENT WHEN CONNECTED TO GPIOA TXRX
	GpioLed.pgpiox = GPIOA;
	GpioLed.gpio_pinconfig.pin_number = GPIO_PIN_5;
	GpioLed.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_OUT;
	GpioLed.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_MED;
	GpioLed.gpio_pinconfig.pin_optype = GPIO_OP_PUSH_PULL;
	GpioLed.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;
	gpio_peripheral_clk(GPIOA,ENABLE);
	gpio_init(&GpioLed);

}

void delay(void) {
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void) {

	GPIO_ButtonInit();
	USART2_GPIOInit();
	USART2_Init();
	usart_peripheral_control(USART3,ENABLE);

    while(1) {
		while (gpio_read_pin(GPIOC,GPIO_PIN_13));
		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		gpio_toggle_pin(GPIOA, 5);
		usart_tx(&usart3_handle,(uint8_t*)msg,strlen(msg));
    }

	return 0;
}


