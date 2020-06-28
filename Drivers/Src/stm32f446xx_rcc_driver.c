/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Apr 8, 2020
 *      Author: vvverma
 */

#include "stm32f446xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};


uint32_t rcc_get_pclk1_value(void) {
    uint32_t pclk1, system_clock;
    uint8_t clksrc,temp, ahbp, apb1p;

    clksrc = ((RCC->cfgr >> 2) & 0x3);

    if (clksrc == 0 ) {
        system_clock = 16000000;
    } 
    else if(clksrc == 1) {
        system_clock = 8000000;
    }
    else if (clksrc == 2) {
        system_clock = rcc_get_pll_output_clk();
    }

    //for ahb
    temp = ((RCC->cfgr >> 4 ) & 0xF);

    if (temp < 8) {
        ahbp = 1;
    }
    else {
        ahbp = AHB_PreScaler[temp-8];
    }

    //apb1
    temp = ((RCC->cfgr >> 10 ) & 0x7);

    if (temp < 4) {
        apb1p = 1;
    }
    else {
        apb1p = APB1_PreScaler[temp-4];
    }

    pclk1 =  (system_clock / ahbp) /apb1p;

    return pclk1;
}



/*********************************************************************
 * @fn                - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint32_t rcc_get_pclk2_value(void) {
    uint32_t system_clock, tmp, pclk2;
    uint8_t ahbp,apb2p, clk_src;

    clk_src = (RCC->cfgr>>2) & 0X3;
    system_clock = 0;

    if (clk_src == 0) {
        system_clock = 16000000;
    }
    else {
        system_clock = 8000000;
    }

    tmp = (RCC->cfgr>>4) & 0xF;

    if (tmp < 0x08) {
        ahbp = 1;
    }
    else {
       ahbp = AHB_PreScaler[tmp-8];
    }

    tmp = (RCC->cfgr>>13) & 0x7;
    if(tmp < 0x04) {
        apb2p = 1;
    }
    else {
        apb2p = APB1_PreScaler[tmp-4];
    }

    pclk2 = (system_clock / ahbp )/ apb2p;

    return pclk2;
}

uint32_t  rcc_get_pll_output_clk(void) {
    return 0;
}
