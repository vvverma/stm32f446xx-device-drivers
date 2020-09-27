/*
 * console.c
 *
 *  Created on: Sep 27, 2020
 *      Author: vvverma
 */



#include <doubly_link_list.h>
#include <string.h>
#include <stdio.h>
#include <console.h>

usart_handle_t usart1_handle;

char tx_msg[1024] = "USART1 PB7,PA10: Receive(RX) PB6,PA9: Transmit(TX) Testing...\n\r";
char msg_start[1] = "#";
char rx_msg[1024];


void conv_list_string(node* list,char* rstring){
	int i = 0;
	node* mover;
	mover = list->next;

	while(mover!=NULL){
		rstring[i] = mover->value;
		i++;
		mover = mover->next;
	}
	rstring[i] = '\0';
}

void putchars(char value){
	char buff[1];
	buff[0] = value;
	usart_tx(&usart1_handle,(uint8_t*)buff,1);
}
char getchars(){
	char rvalue[1];
    usart_rx(&usart1_handle,(uint8_t*)rvalue,1);
    return rvalue[0];
}
void putstring(char* str_value){
	usart_tx(&usart1_handle,(uint8_t*)str_value,strlen(str_value));
	putchars('\n');
    putchars('\r');

}
char* getstring(int* num_bytes){
	return 0;
}


//Fix error to avoid 16byte
void getline(char* rstring){
	node* list, *listPtr, *temp;
	int count;
	char curr_char;
	list = create_list();
	listPtr = list;
	count = 0;

	while((curr_char = getchars())!= '\r'){
		putchars(curr_char);
        if(curr_char == '\b'){
            curr_char = ' ';
            putchars(curr_char);
            curr_char = '\b';
            putchars(curr_char);
            if (listPtr->value != '\0'){
            	temp = listPtr->prev;
            	del_node(&listPtr);
            	listPtr = temp;
            }
        }
        else {
        	add_node(&list, curr_char);
        	listPtr = listPtr->next;
        }
    }
	putchars('\n');
	putchars('\r');

	count = get_list_length(&list);
	//print_list(list);
	//putchars('\n');
	//putchars('\r');
	char number[5];
	itoa(count, number,10);
	putstring(number);
	putchars('\n');
	putchars('\r');
	conv_list_string(list, rstring);
	del_list(&list);

}

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



void delay(void) {
    for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void) {

	char buff[256];
    usart1_gpio_init();

    usart1_init();

    usart_peripheral_control(USART1,ENABLE);
    putstring(tx_msg);

    while(1)
    {
    	getline(buff);
    	putstring(buff);
    }

    return 0;
}

