/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Apr 8, 2020
 *      Author: vvverma
 */

#include "stm32f446xx_usart_driver.h"

void usart_peripheral_clk(usartx_reg_def_t* pusartx, uint8_t enordi) {
    if (enordi == ENABLE) {
        if (pusartx == USART1) {
            USART1_PCLK_EN();
        }
        else if (pusartx == USART2) {
            USART2_PCLK_EN();
        }
        else if (pusartx == USART3) {
            USART3_PCLK_EN();
        }
        else if (pusartx == UART4) {
            UART4_PCLK_EN();
        }
        else if (pusartx == UART5) {
            UART5_PCLK_EN();
        }
        else if (pusartx == USART6) {
            USART6_PCLK_EN();
        }
    }
    else {
        if (pusartx == USART1) {
            USART1_PCLK_DI();
        }
        else if (pusartx == USART2) {
            USART2_PCLK_DI();
        }
        else if (pusartx == USART3) {
            USART3_PCLK_DI();
        }
        else if (pusartx == UART4) {
            UART4_PCLK_DI();
        }
        else if (pusartx == UART5) {
            UART5_PCLK_DI();
        }
        else if (pusartx == USART6) {
            USART6_PCLK_DI();
        }
    }
}


void usart_init(usart_handle_t* pusart_handle) {
    uint32_t tempreg=0;

    /******************************** Configuration of CR1******************************************/

    //Step 1: Enable the Clock for given USART peripheral
    usart_peripheral_clk(pusart_handle->pusartx,ENABLE);

    //Step 2: Enable USART Tx and Rx engines according to the USART_Mode configuration
    if (pusart_handle->usart_config.mode == USART_MODE_RX_ONLY) {
        //Implement the code to enable the Receiver bit field
        tempreg |= (1<<USART_CR1_RE);
    }
    else if (pusart_handle->usart_config.mode == USART_MODE_TX_ONLY) {
        //Implement the code to enable the Transmitter bit field
        tempreg |= (1<<USART_CR1_TE);

    }
    else if (pusart_handle->usart_config.mode == USART_MODE_TXRX) {
        //Implement the code to enable the both Transmitter and Receiver bit fields
        tempreg |= ((1<<USART_CR1_RE)|(1<<USART_CR1_TE));
    }

    //Step 3: Implement Word length configuration
    tempreg |= pusart_handle->usart_config.word_len << USART_CR1_M;

    //Step 4: Configuration of parity control bit fields
    if (pusart_handle->usart_config.parity_ctrl == USART_PARITY_EN_EVEN) {
        //Implement the code to enable the parity control
        tempreg |= (1<<USART_CR1_PCE);
        //Implement the code to enable EVEN parity
        //Not required because by default EVEN parity will be selected once you enable the parity control
    }
    else if (pusart_handle->usart_config.parity_ctrl == USART_PARITY_EN_ODD) {
        //Implement the code to enable the parity control
        tempreg |= ( 1 << USART_CR1_PCE);

        //Implement the code to enable ODD parity
        tempreg |= ( 1 << USART_CR1_PS);
    }

   //Program the CR1 register
    pusart_handle->pusartx->cr1 = tempreg;

/******************************** Configuration of CR2******************************************/

    tempreg=0;

    //Implement the code to configure the number of stop bits inserted during USART frame transmission
    tempreg |= pusart_handle->usart_config.stopbit_len<<USART_CR2_STOP;

    //Program the CR2 register
    pusart_handle->pusartx->cr2 = tempreg;

/******************************** Configuration of CR3******************************************/

    tempreg=0;

    //Configuration of USART hardware flow control
    if ( pusart_handle->usart_config.hw_flow_ctrl == USART_HW_FLOW_CTRL_CTS) {
        //Implement the code to enable CTS flow control
        tempreg |= (1<<USART_CR3_CTSE);
    }
    else if (pusart_handle->usart_config.hw_flow_ctrl == USART_HW_FLOW_CTRL_RTS) {
        //Implement the code to enable RTS flow control
        tempreg |= (1<<USART_CR3_RTSE);
    }
    else if (pusart_handle->usart_config.hw_flow_ctrl == USART_HW_FLOW_CTRL_CTS_RTS) {
        //Implement the code to enable both CTS and RTS Flow control
        tempreg |= (1<<USART_CR3_CTSE);
        tempreg |= (1<<USART_CR3_RTSE);
    }

    pusart_handle->pusartx->cr3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

    //Implement the code to configure the baud rate
    //We will cover this in the lecture. No action required here
    usart_set_baud_rate(pusart_handle->pusartx, pusart_handle->usart_config.baudrate);

}

void usart_deinit(usart_handle_t* pusart_handle) {

}

void usart_tx(usart_handle_t* pusart_handle, uint8_t *ptxbuffer, uint32_t len) {
    uint16_t *pdata;
    //uint8_t *mover_ptxbuffer = ptxbuffer;

   //Loop over until "Len" number of bytes are transferred
    for(uint32_t i = 0 ; i < len; i++) {
        //Implement the code to wait until TXE flag is set in the SR
        while(!usart_get_status_flag(pusart_handle->pusartx, USART_FLAG_TXE));

        //Check the word_len item for 9BIT or 8BIT in a frame
        if(pusart_handle->usart_config.word_len == USART_DATA_LEN_9_BIT) {
            //if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
            pdata = (uint16_t*) ptxbuffer;
            pusart_handle->pusartx->dr = (*pdata & (uint16_t)0x01FF);

            //check for parity_ctrl
            if(pusart_handle->usart_config.parity_ctrl == USART_PARITY_DISABLE) {
                //No parity is used in this transfer , so 9bits of user data will be sent
                //Implement the code to increment ptxbuffer twice
                ptxbuffer++;
                ptxbuffer++;
            }
            else {
                //Parity bit is used in this transfer . so 8bits of user data will be sent
                //The 9th bit will be replaced by parity bit by the hardware
                ptxbuffer++;
            }
        }
        else {
            //This is 8bit data transfer
            //pdata2 = (uint8_t*) ptxbuffer;
            pusart_handle->pusartx->dr = (*ptxbuffer & (uint8_t) 0xFF);//*ptxbuffer; //& (uint8_t) 0xFF;
            //Implement the code to increment the buffer address
            ptxbuffer++;
        }
    }

    //Implement the code to wait till TC flag is set in the SR
    while(!usart_get_status_flag(pusart_handle->pusartx,USART_FLAG_TC));
}


void usart_rx(usart_handle_t* pusart_handle, uint8_t *prxbuffer, uint32_t len) {
    //Loop over until "Len" number of bytes are transferred
	//char check_character;
	//memset(prxbuffer, 0, 1024);
    for(uint32_t i = 0 ; i < len; i++) {
	//do {
        //Implement the code to wait until RXNE flag is set in the SR
        while(! usart_get_status_flag(pusart_handle->pusartx,USART_FLAG_RXNE));

        //Check the word_len to decide whether we are going to receive 9bit of data in a frame or 8 bit
        if(pusart_handle->usart_config.word_len == USART_DATA_LEN_9_BIT) {
            //We are going to receive 9bit data in a frame
            //Now, check are we using parity_ctrl control or not
            if(pusart_handle->usart_config.parity_ctrl == USART_PARITY_DISABLE) {
                //No parity is used , so all 9bits will be of user data
                //read only first 9 bits so mask the DR with 0x01FF
                *((uint16_t*) prxbuffer) = (pusart_handle->pusartx->dr  & (uint16_t)0x01FF);
                //Now increment the prxbuffer two times
                prxbuffer++;
                prxbuffer++;
            }
            else {
                //Parity is used, so 8bits will be of user data and 1 bit is parity
                 *prxbuffer = (pusart_handle->pusartx->dr  & (uint8_t)0xFF);
                 prxbuffer++;
            }
        }
        else {
            //We are going to receive 8bit data in a frame
            //Now, check are we using parity_ctrl control or not
            if(pusart_handle->usart_config.parity_ctrl == USART_PARITY_DISABLE) {
                //No parity is used , so all 8bits will be of user data
                //read 8 bits from DR
                 *prxbuffer = (uint8_t) (pusart_handle->pusartx->dr  & (uint8_t)0xFF);

            }
            else {
                //Parity is used, so , 7 bits will be of user data and 1 bit is parity
                //read only 7 bits , hence mask the DR with 0X7F
                 *prxbuffer = (uint8_t) (pusart_handle->pusartx->dr  & (uint8_t)0x7F);

            }
            //Now , increment the prxbuffer
            prxbuffer++;
        }
    }
}


uint8_t usart_tx_it(usart_handle_t* pusart_handle, uint8_t *ptxbuffer, uint32_t len){
    uint8_t txstate = pusart_handle->tx_busystate;
    if(txstate != USART_BUSY_IN_TX) {
        pusart_handle->tx_bufflen = len;
        pusart_handle->tx_buffer = ptxbuffer;
        pusart_handle->tx_busystate = USART_BUSY_IN_TX;

        //Implement the code to enable interrupt for TXE
        pusart_handle->pusartx->cr1 |= ( 1 << USART_CR1_TXEIE);
        //Implement the code to enable interrupt for TC
        pusart_handle->pusartx->cr1 |= ( 1 << USART_CR1_TCIE);
    }

    return txstate;
}


uint8_t usart_rx_it(usart_handle_t* pusart_handle, uint8_t *prxbuffer, uint32_t len){
    uint8_t rxstate = pusart_handle->rx_busystate;

    if(rxstate != USART_BUSY_IN_RX) {
        pusart_handle->rx_bufflen = len;
        pusart_handle->rx_buffer = prxbuffer;
        pusart_handle->rx_busystate = USART_BUSY_IN_RX;

        (void)pusart_handle->pusartx->dr;
        //Implement the code to enable interrupt for RXNE
        pusart_handle->pusartx->cr1 |= ( 1 << USART_CR1_RXNEIE);
    }

    return rxstate;
}


/*
 * IRQ Configuration and ISR handling
 */
void usart_irq_interrup_config(uint8_t irq_number, uint8_t enordi) {
    if(enordi == ENABLE) {
        if(irq_number <= 31) {
            //program ISER0 register
            *NVIC_ISER0 |= ( 1 << irq_number );
        }
        else if(irq_number > 31 && irq_number < 64 ) { //32 to 63{
            //program ISER1 register
            *NVIC_ISER1 |= ( 1 << (irq_number % 32) );
        }
        else if(irq_number >= 64 && irq_number < 96 ) {
            //program ISER2 register //64 to 95
            *NVIC_ISER3 |= ( 1 << (irq_number % 64) );
        }
    }
    else {
        if(irq_number <= 31) {
            //program ICER0 register
            *NVIC_ICER0 |= ( 1 << irq_number );
        }else if(irq_number > 31 && irq_number < 64 ) {
            //program ICER1 register
            *NVIC_ICER1 |= ( 1 << (irq_number % 32) );
        }
        else if(irq_number >= 6 && irq_number < 96 ) {
            //program ICER2 register
            *NVIC_ICER3 |= ( 1 << (irq_number % 64) );
        }
    }
}


void usart_irq_priority_config(uint8_t irq_number, uint32_t irq_priority) {
    //1. first lets find out the ipr register
    uint8_t iprx = irq_number / 4;
    uint8_t iprx_section  = irq_number %4 ;
    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

    *(  NVIC_PR_BASE_ADDR + iprx ) |=  ( irq_priority << shift_amount );
}


void usart_irq_handling(usart_handle_t *pusart_handle) {

    uint32_t temp1 , temp2, temp3;

    uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
    temp1 = pusart_handle->pusartx->sr & ( 1 << USART_SR_TC);

     //Implement the code to check the state of TCEIE bit
    temp2 = pusart_handle->pusartx->cr1 & ( 1 << USART_CR1_TCIE);

    if(temp1 && temp2 )
    {
        //this interrupt is because of TC

        //close transmission and call application callback if TxLen is zero
        if ( pusart_handle->tx_busystate == USART_BUSY_IN_TX)
        {
            //Check the TxLen . If it is zero then close the data transmission
            if(! pusart_handle->tx_bufflen )
            {
                //Implement the code to clear the TC flag
                pusart_handle->pusartx->sr &= ~( 1 << USART_SR_TC);

                //Implement the code to clear the TCIE control bit

                //Reset the application state
                pusart_handle->tx_busystate = USART_READY;

                //Reset Buffer address to NULL
                pusart_handle->tx_buffer = 0; //TODO:NULL;

                //Reset the length to zero
                pusart_handle->tx_bufflen = 0;

                //Call the application call back with event USART_EVENT_TX_CMPLT
                usart_application_event_callback(pusart_handle,USART_EVENT_TX_CMPLT);
            }
        }
    }

/*************************Check for TXE flag ********************************************/

    //Implement the code to check the state of TXE bit in the SR
    temp1 = pusart_handle->pusartx->sr & ( 1 << USART_SR_TXE);
    //Implement the code to check the state of TXEIE bit in CR1
    temp2 = pusart_handle->pusartx->cr1 & ( 1 << USART_CR1_TXEIE);

    if(temp1 && temp2 ) {
        //this interrupt is because of TXE
        if(pusart_handle->tx_busystate == USART_BUSY_IN_TX) {
            //Keep sending data until Txlen reaches to zero
            if(pusart_handle->tx_bufflen > 0) {
                //Check the word_len item for 9BIT or 8BIT in a frame
                if(pusart_handle->usart_config.word_len == USART_DATA_LEN_9_BIT) {
                    //if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
                    pdata = (uint16_t*) pusart_handle->tx_buffer;
                    pusart_handle->pusartx->dr = (*pdata & (uint16_t)0x01FF);

                    //check for parity_ctrl
                    if(pusart_handle->usart_config.parity_ctrl == USART_PARITY_DISABLE) {
                        //No parity is used in this transfer , so 9bits of user data will be sent
                        //Implement the code to increment ptxbuffer twice
                        pusart_handle->tx_buffer++;
                        pusart_handle->tx_buffer++;
                        pusart_handle->tx_bufflen-=2;
                    }
                    else {
                        //Parity bit is used in this transfer . so 8bits of user data will be sent
                        //The 9th bit will be replaced by parity bit by the hardware
                        pusart_handle->tx_buffer++;
                        pusart_handle->tx_bufflen-=1;
                    }
                }
                else {
                    //This is 8bit data transfer
                    pusart_handle->pusartx->dr = (*pusart_handle->tx_buffer  & (uint8_t)0xFF);                    //Implement the code to increment the buffer address
                    pusart_handle->tx_buffer++;
                    pusart_handle->tx_bufflen-=1;
                }
            }
            if (pusart_handle->tx_bufflen == 0 ) {
                //TxLen is zero
                //Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
                pusart_handle->pusartx->cr1 &= ~( 1 << USART_CR1_TXEIE);
            }
        }
    }

/*************************Check for RXNE flag ********************************************/

    temp1 = pusart_handle->pusartx->sr & ( 1 << USART_SR_RXNE);
    temp2 = pusart_handle->pusartx->cr1 & ( 1 << USART_CR1_RXNEIE);
    if(temp1 && temp2 )
    {
        //this interrupt is because of rxne
        if(pusart_handle->rx_busystate == USART_BUSY_IN_RX) {
            if(pusart_handle->rx_bufflen > 0) {
                //Check the word_len to decide whether we are going to receive 9bit of data in a frame or 8 bit
                if(pusart_handle->usart_config.word_len == USART_DATA_LEN_9_BIT) {
                    //We are going to receive 9bit data in a frame
                    //Now, check are we using parity_ctrl control or not
                    if(pusart_handle->usart_config.parity_ctrl == USART_PARITY_DISABLE) {
                        //No parity is used , so all 9bits will be of user data
                        //read only first 9 bits so mask the DR with 0x01FF
                        *((uint16_t*) pusart_handle->rx_buffer) = (pusart_handle->pusartx->dr  & (uint16_t)0x01FF);
                        //Now increment the prxbuffer two times
                        pusart_handle->rx_buffer++;
                        pusart_handle->rx_buffer++;
                        pusart_handle->rx_bufflen-=2;
                    }
                    else {
                        //Parity is used, so 8bits will be of user data and 1 bit is parity
                        *pusart_handle->rx_buffer = (pusart_handle->pusartx->dr  & (uint8_t)0xFF);
                        pusart_handle->rx_buffer++;
                        pusart_handle->rx_bufflen-=1;
                    }
                }
                else {
                    //We are going to receive 8bit data in a frame
                    //Now, check are we using parity_ctrl control or not
                    if(pusart_handle->usart_config.parity_ctrl == USART_PARITY_DISABLE)
                    {
                        //No parity is used , so all 8bits will be of user data
                        //read 8 bits from DR
                         *pusart_handle->rx_buffer = (uint8_t) (pusart_handle->pusartx->dr  & (uint8_t)0xFF);
                    }
                    else {
                        //Parity is used, so , 7 bits will be of user data and 1 bit is parity
                        //read only 7 bits , hence mask the DR with 0X7F
                        *pusart_handle->rx_buffer = (uint8_t) (pusart_handle->pusartx->dr  & (uint8_t)0x7F);
                    }
                    //Now , increment the prxbuffer
                    pusart_handle->rx_buffer++;
                    pusart_handle->rx_bufflen-=1;
                }
            }//if of >0

            if(! pusart_handle->rx_bufflen) {
                //disable the rxne
                pusart_handle->pusartx->cr1 &= ~( 1 << USART_CR1_RXNEIE );
                pusart_handle->rx_busystate = USART_READY;
                usart_application_event_callback(pusart_handle,USART_EVENT_RX_CMPLT);
            }
        }
    }


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

    //Implement the code to check the status of CTS bit in the SR
    temp1 = pusart_handle->pusartx->sr & ( 1 << USART_SR_CTS);

    //Implement the code to check the state of CTSE bit in CR1
    temp2 = pusart_handle->pusartx->cr3 & ( 1 << USART_CR3_CTSE);

    //Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
    temp3 = pusart_handle->pusartx->cr3 & ( 1 << USART_CR3_CTSIE);


    if(temp1  && temp2 ) {
        //Implement the code to clear the CTS flag in SR
        pusart_handle->pusartx->sr &=  ~( 1 << USART_SR_CTS);

        //this interrupt is because of cts
        usart_application_event_callback(pusart_handle,USART_EVENT_CTS);
    }

/*************************Check for IDLE detection flag ********************************************/

    //Implement the code to check the status of IDLE flag bit in the SR
    temp1 = pusart_handle->pusartx->sr & ( 1 << USART_SR_IDLE);

    //Implement the code to check the state of IDLEIE bit in CR1
    temp2 = pusart_handle->pusartx->cr1 & ( 1 << USART_CR1_IDLEIE);


    if(temp1 && temp2) {
        //Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
        temp1 = pusart_handle->pusartx->sr &= ~( 1 << USART_SR_IDLE);

        //this interrupt is because of idle
        usart_application_event_callback(pusart_handle,USART_EVENT_IDLE);
    }

/*************************Check for Overrun detection flag ********************************************/

    //Implement the code to check the status of ORE flag  in the SR
    temp1 = pusart_handle->pusartx->sr & USART_SR_ORE;

    //Implement the code to check the status of RXNEIE  bit in the CR1
    temp2 = pusart_handle->pusartx->cr1 & USART_CR1_RXNEIE;


    if(temp1  && temp2 ) {
        //Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

        //this interrupt is because of Overrun error
        usart_application_event_callback(pusart_handle,USART_ERR_ORE);
    }



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

    temp2 =  pusart_handle->pusartx->cr3 & ( 1 << USART_CR3_EIE) ;

    if(temp2 ) {
        temp1 = pusart_handle->pusartx->sr;
        if(temp1 & ( 1 << USART_SR_FE)) {
            /*
                This bit is set by hardware when a de-synchronization, excessive noise or a break character
                is detected. It is cleared by a software sequence (an read to the USART_SR register
                followed by a read to the USART_DR register).
            */
            usart_application_event_callback(pusart_handle,USART_ERR_FE);
        }

        if(temp1 & ( 1 << USART_SR_NE) ) {
            /*
                This bit is set by hardware when noise is detected on a received frame. It is cleared by a
                software sequence (an read to the USART_SR register followed by a read to the
                USART_DR register).
            */
            usart_application_event_callback(pusart_handle,USART_ERR_NE);
        }

        if(temp1 & ( 1 << USART_SR_ORE) ) {
            usart_application_event_callback(pusart_handle,USART_ERR_ORE);
        }
    }

}

/*
 * Other Peripheral Control APIs
 */


uint8_t usart_get_status_flag(usartx_reg_def_t *pusartx, uint8_t status_flag_name) {
    
    if(pusartx->sr & status_flag_name) {
            return SET;
    }
    return RESET;
}


void usart_clear_flag(usartx_reg_def_t *pusartx, uint16_t status_flag_name) {
    pusartx->sr &= ~( status_flag_name);
}


void usart_peripheral_control(usartx_reg_def_t *pusartx, uint8_t enOrdi) {

    if(enOrdi == ENABLE) {
        pusartx->cr1 |= (1 << 13);
    }
    else {
        pusartx->cr1 &= ~(1 << 13);
    }
}


void usart_set_baud_rate(usartx_reg_def_t *pusartx, uint32_t baudrate){

    //Variable to hold the APB clock
    uint32_t PCLKx;
    uint32_t usartdiv;

    //variables to hold Mantissa and Fraction values
    uint32_t M_part,F_part;

    uint32_t tempreg=0;

    //Get the value of APB bus clock in to the variable PCLKx
    if(pusartx == USART1 || pusartx == USART6) {
    //USART1 and USART6 are hanging on APB2 bus
        PCLKx = rcc_get_pclk2_value();
    }
    else {
        PCLKx = rcc_get_pclk1_value();
    }

    //Check for OVER8 configuration bit
    if(pusartx->cr1 & (1 << USART_CR1_OVER8)) {
    //OVER8 = 1 , over sampling by 8
        usartdiv = ((25 * PCLKx) / (2 *baudrate));
    }
    else {
        //over sampling by 16
        usartdiv = ((25 * PCLKx) / (4 *baudrate));
    }

    //Calculate the Mantissa part
    M_part = usartdiv/100;

    //Place the Mantissa part in appropriate bit position . refer USART_BRR
    tempreg |= M_part << 4;

    //Extract the fraction part
    F_part = (usartdiv - (M_part * 100));

    //Calculate the final fractional
    if(pusartx->cr1 & ( 1 << USART_CR1_OVER8)) {
        //OVER8 = 1 , over sampling by 8
        F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

    }
    else {
        //over sampling by 16
        F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
    }

    //Place the fractional part in appropriate bit position . refer USART_BRR
    tempreg |= F_part;

    //copy the value of tempreg in to BRR register
    pusartx->brr = tempreg;

}


/*
 * Application Callbacks
 */
__weak void usart_application_event_callback(usart_handle_t *pusart_handle,uint8_t app_ev){

}

