/*
 * arm_cortex_m4_nvic_driver.c
 *
 *  Created on: Apr 20, 2020
 *      Author: vvverma
 */


#include "arm_cortex_m4_nvic_driver.h"


/*
 * Please verify before activating clear the pending register for that interrupt line
 * */

void nvic_enable_irq(uint32_t irq_number) {
    uint32_t shift_by = 0;

    if(irq_number <= 31) {
        *NVIC_ISER0 |= (1<<irq_number);
    }
    else if(irq_number <= 63) {
        shift_by = irq_number % 32;
        *NVIC_ISER1 |= (1<<shift_by);
    }
    else if(irq_number <= 95) {
        shift_by = irq_number % 32;
        *NVIC_ISER2 |= (1<<shift_by);
    }
    else if(irq_number <= 127) {
        shift_by = irq_number % 32;
        *NVIC_ISER3 |= (1<<shift_by);
    }
    else if(irq_number <= 159) {
        shift_by = irq_number % 32;
        *NVIC_ISER4 |= (1<<shift_by);
    }
    else if(irq_number <=191) {
        shift_by = irq_number % 32;
        *NVIC_ISER5 |= (1<<shift_by);
    }
    else if(irq_number <= 223) {
        shift_by = irq_number % 32;
        *NVIC_ISER6 |= (1<<shift_by);
    }
    else if(irq_number <= 239) {
        //Only first 0 to 15 bits are used remaining are reserved
        shift_by = irq_number % 32;
        *NVIC_ISER7 |= (1<<shift_by);
    }
}

/*
 * Disables an interrupt or irq
 * */
void nvic_disable_irq(uint32_t irq_number){

    uint32_t shift_by = 0;

    if(irq_number <= 31) {
        *NVIC_ICER0 |= (1<<irq_number);
    }
    else if(irq_number <= 63) {
        shift_by = irq_number % 32;
        *NVIC_ICER1 |= (1<<shift_by);
    }
    else if(irq_number <= 95) {
        shift_by = irq_number % 32;
        *NVIC_ICER2 |= (1<<shift_by);
    }
    else if(irq_number <= 127) {
        shift_by = irq_number % 32;
        *NVIC_ICER3 |= (1<<shift_by);
    }
    else if(irq_number <= 159) {
        shift_by = irq_number % 32;
        *NVIC_ICER4 |= (1<<shift_by);
    }
    else if(irq_number <=191) {
        shift_by = irq_number % 32;
        *NVIC_ICER5 |= (1<<shift_by);
    }
    else if(irq_number <= 223) {
        shift_by = irq_number % 32;
        *NVIC_ICER6 |= (1<<shift_by);
    }
    else if(irq_number <= 239) {
        //Only first 0 to 15 bits are used remaining are reserved
        shift_by = irq_number % 32;
        *NVIC_ICER7 |= (1<<shift_by);
    }

}

/*
 * Sets pending status of interrupt or exception to 1
 * Set bit to 1 only useful of that line is currently disabled
 * */
void nvic_set_pending_irq(uint32_t irq_number){
uint32_t shift_by = 0;

    if(irq_number <= 31) {
        *NVIC_ISPR0 |= (1<<irq_number);
    }
    else if(irq_number <= 63) {
        shift_by = irq_number % 32;
        *NVIC_ISPR1 |= (1<<shift_by);
    }
    else if(irq_number <= 95) {
        shift_by = irq_number % 32;
        *NVIC_ISPR2 |= (1<<shift_by);
    }
    else if(irq_number <= 127) {
        shift_by = irq_number % 32;
        *NVIC_ISPR3 |= (1<<shift_by);
    }
    else if(irq_number <= 159) {
        shift_by = irq_number % 32;
        *NVIC_ISPR4 |= (1<<shift_by);
    }
    else if(irq_number <=191) {
        shift_by = irq_number % 32;
        *NVIC_ISPR5 |= (1<<shift_by);
    }
    else if(irq_number <= 223) {
        shift_by = irq_number % 32;
        *NVIC_ISPR6 |= (1<<shift_by);
    }
    else if(irq_number <= 239) {
        //Only first 0 to 15 bits are used remaining are reserved
        shift_by = irq_number % 32;
        *NVIC_ISPR7 |= (1<<shift_by);
    }

}

/*
 * Clears the pending status of interrupt or exception to 0
 * */
void nvic_clear_pending_irq(uint32_t irq_number){
    uint32_t shift_by = 0;

    if(irq_number <= 31) {
        *NVIC_ICPR0 |= (1<<irq_number);
    }
    else if(irq_number <= 63) {
        shift_by = irq_number % 32;
        *NVIC_ICPR1 |= (1<<shift_by);
    }
    else if(irq_number <= 95) {
        shift_by = irq_number % 32;
        *NVIC_ICPR2 |= (1<<shift_by);
    }
    else if(irq_number <= 127) {
        shift_by = irq_number % 32;
        *NVIC_ICPR3 |= (1<<shift_by);
    }
    else if(irq_number <= 159) {
        shift_by = irq_number % 32;
        *NVIC_ICPR4 |= (1<<shift_by);
    }
    else if(irq_number <=191) {
        shift_by = irq_number % 32;
        *NVIC_ICPR5 |= (1<<shift_by);
    }
    else if(irq_number <= 223) {
        shift_by = irq_number % 32;
        *NVIC_ICPR6 |= (1<<shift_by);
    }
    else if(irq_number <= 239) {
        //Only first 0 to 15 bits are used remaining are reserved
        shift_by = irq_number % 32;
        *NVIC_ICPR7 |= (1<<shift_by);
    }
}

/*
 * Reads the pending status of interrupt or exception
 * Returns non-zero value if pending status is set to 0
 * */
uint32_t nvic_get_pending_irq(uint32_t irq_number){
    return 0;
}

/*
 * Set the priority of interrupt or exception with a configurable
 * priority level
 * */
void nvic_set_priority_irq(uint32_t irq_number, uint32_t priority){
    uint8_t iprx = irq_number / 4;
    uint8_t iprx_section  = irq_number %4 ;
    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

    *(  NVIC_PR_BASE_ADDR + iprx ) |=  ( priority << shift_amount );
}

/*
 * Reads the priority of interrupt or exception
 * Returns current priority level
 * */
uint32_t nvic_get_priority_irq(uint32_t irq_number){
    return 0;
}
