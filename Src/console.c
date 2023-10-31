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


char* supported_command[] = {"gpio", "help"};



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

void parse_command(char* command){
	if (strncmp("help", command, 4) == 0)
	{
		putstring("Help Command Used");
	}
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
        if(curr_char == '\b'){
        	putchars(curr_char);
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
        else if (curr_char == '\e') {
        	//add functionality in driver to peek or to check if more characters on bus to fix ESC
        	//if (usart_get_status_flag(usart1_handle.pusartx ,USART_FLAG_RXNE)){
        	    curr_char = getchars();
        	    curr_char = getchars();
        	    if (curr_char == 'A') { //Up Arrow
        	    	usart_tx(&usart1_handle,(uint8_t*)"\e[A",3);
				}
				else if (curr_char == 'B') { //Down Arrow
					usart_tx(&usart1_handle,(uint8_t*)"\e[B",3);
				}
				else if (curr_char == 'C') { //Right Arrow
					usart_tx(&usart1_handle,(uint8_t*)"\e[C",3);
				}
				else if (curr_char == 'D') { //Left Arrow
					usart_tx(&usart1_handle,(uint8_t*)"\e[D",3);
				}
				else if (curr_char == 'H') { //Home
					usart_tx(&usart1_handle,(uint8_t*)"\e[H",3);
				}
				else if (curr_char == 'F') { //End
					usart_tx(&usart1_handle,(uint8_t*)"\e[F",3);
				}
				else if (curr_char == '2') { //Insert                 /*Add support to insert*/
					 curr_char = getchars();
            	     usart_tx(&usart1_handle,(uint8_t*)"\e[2~",4);
    			}
				else if (curr_char == '3') { //Delete                 /*Add support to delete*/
					curr_char = getchars();
					usart_tx(&usart1_handle,(uint8_t*)"\e[3~",4);
				}
				else if (curr_char == '5') { //Page Up
					curr_char = getchars();
					usart_tx(&usart1_handle,(uint8_t*)"\e[5~",4);
				}
				else if (curr_char == '6') {    //Page Down
					curr_char = getchars();
					usart_tx(&usart1_handle,(uint8_t*)"\e[6~",4);
				}

        }
        else {
        	putchars(curr_char);
        	add_node(&list, curr_char);
        	listPtr = listPtr->next;
        }
    }
	putchars('\n');
	putchars('\r');

	count = get_list_length(&list);
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


void init_i2c_gpio(){

		gpio_handle_t I2CPins;
		I2CPins.pgpiox = GPIOB;
		I2CPins.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_ALT_FUN;
		I2CPins.gpio_pinconfig.pin_optype = GPIO_OP_OPEN_DRAIN;

		I2CPins.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_PU;
		I2CPins.gpio_pinconfig.pin_altfun_mode= 4;
		I2CPins.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_FAST;

		//scl
		I2CPins.gpio_pinconfig.pin_number = GPIO_PIN_8;
		gpio_init(&I2CPins);


		//sda
		//Note : since we found a glitch on PB9 , you can also try with PB7
		I2CPins.gpio_pinconfig.pin_number = GPIO_PIN_9;

		gpio_init(&I2CPins);
}


void GPIO_ButtonInit(void)
{
gpio_handle_t GPIOBtn;

//this is btn gpio configuration
GPIOBtn.pgpiox = GPIOC;
GPIOBtn.gpio_pinconfig.pin_number = GPIO_PIN_13;
GPIOBtn.gpio_pinconfig.pin_mode = GPIO_PIN_MODE_IN;
GPIOBtn.gpio_pinconfig.pin_speed = GPIO_PIN_SPEED_FAST;
GPIOBtn.gpio_pinconfig.pin_pupd_ctrl = GPIO_PIN_NO_PUPD;

gpio_init(&GPIOBtn);

}

i2c_handle_t i2c_test;

//#define SSD1306_WRITECOMMAND(data) 	i2c_writeByte(0x3c,0x00,data)
#define SSD1306_WRITECOMMAND(data) i2c_master_send(&i2c_test,(uint8_t)0x00,(uint8_t)0x3c,data);
//#define SSD1306_WRITEDATA(data) 	i2c_writeByte(0x3c,0x40,data)
#define SSD1306_WRITEDATA(data) i2c_master_send(&i2c_test,(uint8_t)0x40,(uint8_t)0x3c,data);



int main(void) {
	//uint8_t buffer [] = {0x00,0xAE,
	//		              0x00,0xAF,
	//					 0x40,0xFF,0x40,0x00,0x40,0xFF,0x40,0x00,0x40,0xFF,0x40,0x00};
	dma_handle_t dma_test;
	i2c_test.pi2cx = I2C1;
	i2c_test.i2c_pinconfig.ack_en=1;
	i2c_test.i2c_pinconfig.speed_mode = I2C_SCL_SPEED_SM;
	i2c_test.i2c_pinconfig.duty = 0;

	GPIO_ButtonInit();
    init_i2c_gpio();

	i2c_init(&i2c_test);

	i2c_peripheral_control(i2c_test.pi2cx,ENABLE);

	//wait till button is pressed
    //while(gpio_read_pin(GPIOC,GPIO_PIN_13)==0x1);
    delay();

    delay();
	SSD1306_WRITECOMMAND(0xAE); //display off
	SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode
	SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	SSD1306_WRITECOMMAND(0x00); //---set low column address
	SSD1306_WRITECOMMAND(0x10); //---set high column address
	SSD1306_WRITECOMMAND(0x40); //--set start line address
	SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	SSD1306_WRITECOMMAND(0xFF);
	SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	SSD1306_WRITECOMMAND(0xA6); //--set normal display
	SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	SSD1306_WRITECOMMAND(0x3F); //
	SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WRITECOMMAND(0xD3); //-set display offset
	SSD1306_WRITECOMMAND(0x00); //-not offset
	SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	SSD1306_WRITECOMMAND(0x22); //
	SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	SSD1306_WRITECOMMAND(0x14); //
	SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel

	for (int i = 0; i < 8; i++) {
		for(int j = 0 ; i <128 ; j++) {
		  SSD1306_WRITEDATA(0xAA); //--turn on SSD1306 panel
		  SSD1306_WRITEDATA(0x55); //--turn on SSD1306 panel
		}
	}
	//i2c_master_send(&i2c_test,(uint8_t*)buffer,16, 0x3c);


	volatile int i = 1;
	while(i);

	dma_test.dma_config.stream_number = 0;
	dma_test.pdmax = DMA1;
    dma_peripheral_clk(DMA1, ENABLE);
    dma_init(&dma_test);
	char command[256];
    usart1_gpio_init();

    usart1_init();

    usart_peripheral_control(USART1,ENABLE);
    putstring(tx_msg);

    while(1)
    {
    	putchars('#');
    	getline(command);
        parse_command(command);
    	//putstring(command);
    }

    return 0;
}

