/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Nov 27, 2019
 *      Author: vverma
 */
#include "stm32f446xx_gpio_driver.h"

void gpio_peripheral_clk(gpiox_reg_def_t* pgpiox, uint8_t enordi) {

    if (enordi == ENABLE) {

        if (pgpiox == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pgpiox == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pgpiox == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pgpiox == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pgpiox == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (pgpiox == GPIOF) {
            GPIOF_PCLK_EN();
        } else if (pgpiox == GPIOG) {
            GPIOG_PCLK_EN();
        } else if (pgpiox == GPIOH) {
            GPIOH_PCLK_EN();
        } else {
            //Report error or print something
        }

    } else if (enordi == DISABLE) {
        if (pgpiox == GPIOA) {
            GPIOA_PCLK_DI();
        } else if (pgpiox == GPIOB) {
            GPIOB_PCLK_DI();
        } else if (pgpiox == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pgpiox == GPIOD) {
            GPIOD_PCLK_DI();
        } else if (pgpiox == GPIOE) {
            GPIOE_PCLK_DI();
        } else if (pgpiox == GPIOF) {
            GPIOF_PCLK_DI();
        } else if (pgpiox == GPIOG) {
            GPIOG_PCLK_DI();
        } else if (pgpiox == GPIOH) {
            GPIOH_PCLK_DI();
        } else {
            //Report error or print something
        }
    } else {
        //Report error or print something
    }
}


void gpio_init(gpio_handle_t* pgpio_handle) {

    uint32_t temp = 0;
    //uint32_t test = 0;

    //enable the clock
    gpio_peripheral_clk(pgpio_handle->pgpiox, ENABLE);

    if (pgpio_handle->gpio_pinconfig.pin_mode <= GPIO_PIN_MODE_ANALOG) {
        temp = (pgpio_handle->gpio_pinconfig.pin_mode <<
                                (2*pgpio_handle->gpio_pinconfig.pin_number));
        //test =  ~(0x3 << (2*pgpio_handle->gpio_pinconfig.pin_number));
        pgpio_handle->pgpiox->moder &= ~(0x3 <<
                                (2*pgpio_handle->gpio_pinconfig.pin_number));
        pgpio_handle->pgpiox->moder |= temp;
    } else {
        //interrupt side
    }

    //clear and set speed register
    temp = 0;
    temp = (pgpio_handle->gpio_pinconfig.pin_speed <<
                                (2*pgpio_handle->gpio_pinconfig.pin_number));
    pgpio_handle->pgpiox->ospeedr &= ~(0x3 <<
                                (2*pgpio_handle->gpio_pinconfig.pin_number)); //fix this for gpio A and B which have diff reset

    pgpio_handle->pgpiox->ospeedr |= temp;

    //clear and set pullup and pulldown
    temp = 0;
    temp = (pgpio_handle->gpio_pinconfig.pin_pupd_ctrl <<
                                (2*pgpio_handle->gpio_pinconfig.pin_number));
    pgpio_handle->pgpiox->pupdr &= ~(0x3 <<
                                (2*pgpio_handle->gpio_pinconfig.pin_number));
    pgpio_handle->pgpiox->pupdr |= temp;

    //clear and set pin alt func mode
    if (pgpio_handle->gpio_pinconfig.pin_mode == GPIO_PIN_MODE_ALT_FUN) {
        temp = 0;
        if (pgpio_handle->gpio_pinconfig.pin_number <= GPIO_PIN_7) {
            temp = (pgpio_handle->gpio_pinconfig.pin_altfun_mode <<
                                (4*pgpio_handle->gpio_pinconfig.pin_number));
            pgpio_handle->pgpiox->afrl &= ~(0xF <<
                                (4*pgpio_handle->gpio_pinconfig.pin_number));
            pgpio_handle->pgpiox->afrl |= temp;

        } else {
            temp = (pgpio_handle->gpio_pinconfig.pin_altfun_mode <<
                            (4*(pgpio_handle->gpio_pinconfig.pin_number%8)));
            pgpio_handle->pgpiox->afrh &= ~(0xF <<
                            (4*(pgpio_handle->gpio_pinconfig.pin_number%8)));
            pgpio_handle->pgpiox->afrh|= temp;
        }
    }

    //clear and set pin output type
    temp = 0;
    temp = (pgpio_handle->gpio_pinconfig.pin_optype <<
                                    (pgpio_handle->gpio_pinconfig.pin_number));
    pgpio_handle->pgpiox->otyper &= ~(0x1 <<
                                    (pgpio_handle->gpio_pinconfig.pin_number));
    pgpio_handle->pgpiox->otyper |= temp;

}


void gpio_deinit(gpiox_reg_def_t* pgpiox) {
    if (pgpiox == GPIOA) {
            GPIOA_RESET();
        } else if (pgpiox == GPIOB) {
            GPIOB_RESET();
        } else if(pgpiox == GPIOC) {
            GPIOC_RESET();
        } else if (pgpiox == GPIOD) {
            GPIOD_RESET();
        } else if (pgpiox == GPIOE) {
            GPIOE_RESET();
        } else if (pgpiox == GPIOF) {
            GPIOF_RESET();
        } else if (pgpiox == GPIOG) {
            GPIOG_RESET();
        } else if (pgpiox == GPIOH) {
            GPIOH_RESET();
        } else {
            //Report error or print something
        }
}


uint8_t gpio_read_pin(gpiox_reg_def_t* pgpiox, uint8_t pin_number){
    uint8_t data;
    data = (uint8_t) (pgpiox->idr >> pin_number)& 0x00000001;
    return data;
}


void gpio_write_pin(gpiox_reg_def_t* pgpiox, uint8_t pin_number, uint8_t value){
    if (value == GPIO_PIN_SET) {
        pgpiox->odr |= (1<<pin_number);
    } else {
        pgpiox->odr &= ~(1<<pin_number);
    }
}


void gpio_write_port(gpiox_reg_def_t* pgpiox, uint16_t value) {
    pgpiox->odr |= value;
}


uint16_t gpio_read_port(gpiox_reg_def_t* pgpiox) {
    return (uint16_t)pgpiox->odr;
}


void gpio_toggle_pin(gpiox_reg_def_t* pgpiox, uint8_t pin_number) {
    pgpiox->odr ^= (1 << pin_number);
}
