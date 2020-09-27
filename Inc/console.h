/*
 * console.h
 *
 *  Created on: Sep 27, 2020
 *      Author: vvverma
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include "stm32f446xx.h"

extern usart_handle_t usart1_handle;

void putchars(char value);
char getchars();
void putstring(char* str_value);
char* getstring(int* num_bytes);


#endif /* CONSOLE_H_ */
