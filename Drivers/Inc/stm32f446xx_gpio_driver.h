/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Nov 27, 2019
 *      Author: vverma
 */

#ifndef INC_STM32F446XX_gpio_DRIVER_H_
#define INC_STM32F446XX_gpio_DRIVER_H_

#include "stm32f446xx.h"

typedef struct {
	uint8_t pin_number;
	uint8_t pin_mode;
	uint8_t pin_speed;
	uint8_t pin_pupd_ctrl;
	uint8_t pin_optype;
	uint8_t pin_altfun_mode;
} gpio_pin_config_t;


typedef struct {
	gpiox_reg_def_t* pgpiox;
	gpio_pin_config_t gpio_pinconfig;
} gpio_handle_t;


#define GPIO_PIN_MODE_IN			0
#define GPIO_PIN_MODE_OUT			1
#define GPIO_PIN_MODE_ALT_FUN		2
#define GPIO_PIN_MODE_ANALOG		3

#define GPIO_PIN_SPEED_LOW			0
#define GPIO_PIN_SPEED_MED			1
#define GPIO_PIN_SPEED_FAST			2
#define GPIO_PIN_SPEED_HI			3

#define GPIO_PIN_NO_PUPD			0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2

#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

#define GPIO_OP_PUSH_PULL			0
#define GPIO_OP_OPEN_DRAIN			1

#define GPIO_MODE_IT_FT				4
#define GPIO_MODE_IT_RT				5
#define GPIO_MODE_IT_RFT			6


/*
 * *****************************************************************
 * GPIO Driver API
 *******************************************************************
*/


/*
 * Control Peripheral Clock to enable or Disable
**/
void gpio_peripheral_clk(gpiox_reg_def_t* pgpiox, uint8_t enordi);


/*
 * Initialize gpio
**/
void gpio_init(gpio_handle_t* pgpio_handle);


/*
 * Deintialize gpio
**/
void gpio_deinit(gpiox_reg_def_t* pgpiox);


/*
 * Read from Input Pin
**/
uint8_t gpio_read_pin(gpiox_reg_def_t* pgpiox, uint8_t pin_number);


/*
 *  Write to OutputPin
**/
void gpio_write_pin(gpiox_reg_def_t* pgpiox, uint8_t pin_number, uint8_t value);


/*
 * Write to Output Port
 **/
void gpio_write_port(gpiox_reg_def_t* pgpiox, uint16_t value);


/*
 * Read from Input Port
 **/
uint16_t gpio_read_port(gpiox_reg_def_t* pgpiox);


/*
 * Toggle Output Pin
**/
void gpio_toggle_pin(gpiox_reg_def_t* pgpiox, uint8_t pin_number);


/*
 * TODO
 *
void gpio_IRQConfig(uint8_t IRQNUmber, uint8_t IRQPriority, uint8_t ENorDI);
void gpio_IRQHandling(uint8_t PinNumber);
*/

#endif /* INC_STM32F446XX_gpio_DRIVER_H_ */



