/*
 * arm_cortex_m4_nvic_driver.h
 *
 *  Created on: Apr 20, 2020
 *      Author: vvverma
 */

#ifndef INC_ARM_CORTEX_M4_NVIC_DRIVER_H_
#define INC_ARM_CORTEX_M4_NVIC_DRIVER_H_
#include "arm_cortex_m4.h"
/*
 * Enables an interrupt or exception
 * */
void nvic_enable_irq(uint32_t irq_number);

/*
 * Disables an interrupt or irq
 * */
void nvic_disable_irq(uint32_t irq_number);

/*
 * Sets pending status of interrupt or exception to 1
 * */
void nvic_set_pending_irq(uint32_t irq_number);

/*
 * Clears the pending status of interrupt or exception to 0
 * */
void nvic_clear_pending_irq(uint32_t irq_number);

/*
 * Reads the pending status of interrupt or exception
 * Returns non-zero value if pending status is set to 0
 * */
uint32_t nvic_get_pending_irq(uint32_t irq_number);

/*
 * Set the priority of interrupt or exception with a configurable
 * priority level
 * */
void nvic_set_priority_irq(uint32_t irq_number, uint32_t priority);

/*
 * Reads the priority of interrupt or exception
 * Returns current priority level
 * */
uint32_t nvic_get_priority_irq(uint32_t irq_number);



#endif /* INC_ARM_CORTEX_M4_NVIC_DRIVER_H_ */
