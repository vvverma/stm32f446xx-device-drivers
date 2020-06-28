/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: March 10, 2020
 *      Author: vvverma
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_
#include "stm32f446xx.h"

/*
 * What are the different clock source in stm32f446re?
 *      1) LSE and LSI
 *      2) HSE (Crystal Oscillator) and HSI(RC Oscillator)
 *      3) PLL (Phase Lock Loop) -> Mostly to step up the clock 
 * What this driver is about? 
 *      To prove with clock values to the use depending on the peripheral.
 *      Future support would be to provide ability to modify clck source
 *
 *  Created on: March 10, 2020
 *      Author: vvverma
 */
//This returns the APB1 clock value
uint32_t rcc_get_pclk1_value(void);

//This returns the APB2 clock value
uint32_t rcc_get_pclk2_value(void);

uint32_t rcc_get_pll_output_clk(void);


#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
