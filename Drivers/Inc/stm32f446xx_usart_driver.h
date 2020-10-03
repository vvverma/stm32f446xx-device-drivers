/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Feb 12, 2020
 *      Author: vvverma
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_
#include "stm32f446xx.h"

typedef struct{
	uint32_t baudrate;
	uint8_t mode;
	uint8_t word_len;
	uint8_t stopbit_len;
	uint8_t parity_ctrl;
	uint8_t hw_flow_ctrl;
}usart_config_t;

typedef struct{
	usartx_reg_def_t* pusartx;
	usart_config_t usart_config;
	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
	uint32_t tx_bufflen;
	uint32_t rx_bufflen;
	uint8_t tx_busystate;
	uint8_t rx_busystate;
}usart_handle_t;

/*TODO Add support for SYNC and ASYNC MODE*/
/*TODO add more baud support*/
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

#define USART_MODE_TX_ONLY	0
#define USART_MODE_RX_ONLY	1
#define USART_MODE_TXRX		2

#define USART_DATA_LEN_8_BIT	RESET
#define USART_DATA_LEN_9_BIT	SET

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_DISABLE  0
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_EN_ODD   2

#define USART_STOPBIT_ONE      0
#define USART_STOPBIT_HALF	   1
#define USART_STOPBIT_TWO	   2
#define USART_STOPBIT_ONE_HALF 3


#define USART_HW_FLOW_CTRL_NONE    0
#define USART_HW_FLOW_CTRL_CTS     1
#define USART_HW_FLOW_CTRL_RTS     2
#define USART_HW_FLOW_CTRL_CTS_RTS 3


/*
 * Application states
 */
#define USART_READY      0
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2


#define 	USART_EVENT_TX_CMPLT 0
#define		USART_EVENT_RX_CMPLT 1
#define		USART_EVENT_IDLE     2
#define		USART_EVENT_CTS      3
#define		USART_EVENT_PE       4
#define		USART_ERR_FE         5
#define		USART_ERR_NE         6
#define		USART_ERR_ORE        7

/*
 * USART flags
 */

#define USART_FLAG_TXE  ( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE ( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 	( 1 << USART_SR_TC)


void usart_peripheral_clk(usartx_reg_def_t* pusartx, uint8_t enordi);
void usart_init(usart_handle_t* pusart_handle);
void usart_deinit(usart_handle_t* pusart_handle);

void usart_tx(usart_handle_t* pusart_handle, uint8_t *ptxbuffer, uint32_t len);
void usart_rx(usart_handle_t* pusart_handle, uint8_t *prxbuffer, uint32_t len);
uint8_t usart_tx_it(usart_handle_t* pusart_handle, uint8_t *ptxbuffer, uint32_t len);
uint8_t usart_rx_it(usart_handle_t* pusart_handle, uint8_t *prxbuffer, uint32_t len);


/*
 * IRQ Configuration and ISR handling
 */
void usart_irq_interrup_config(uint8_t irq_number, uint8_t enordi);
void usart_irq_priority_config(uint8_t irq_number, uint32_t irq_priority);
void usart_irq_handling(usart_handle_t *pusart_handle);

/*
 * Other Peripheral Control APIs
 */

uint8_t usart_get_status_flag(usartx_reg_def_t *pusartx, uint8_t status_flag_name);
void usart_clear_flag(usartx_reg_def_t *pusartx, uint16_t status_flag_name);
void usart_peripheral_control(usartx_reg_def_t *pusartx, uint8_t enOrdi);
void usart_set_baud_rate(usartx_reg_def_t *pusartx, uint32_t baudrate);


/*
 * Application Callbacks
 */
void usart_application_event_callback(usart_handle_t *pusart_handle,uint8_t app_ev);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
