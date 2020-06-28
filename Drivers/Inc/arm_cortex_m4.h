/*
 * arm_cortex_m4.h
 *
 *  Created on: Apr 21, 2020
 *      Author: vvverma
 */

#ifndef INC_ARM_CORTEX_M4_H_
#define INC_ARM_CORTEX_M4_H_

#include <stdint.h>

#define _IO volatile


/**********************************START:Processor Specific Details **********************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 * Interrupt Set Enable Register
 * Permission: RW
 */
#define NVIC_ISER0    ((_IO uint32_t*)0xE000E100) /*IRQ Line: 0  <-> IRQ Line: 31  */
#define NVIC_ISER1    ((_IO uint32_t*)0xE000E104) /*IRQ Line: 32 <-> IRQ Line: 63  */
#define NVIC_ISER2    ((_IO uint32_t*)0xE000E108) /*IRQ Line: 64 <-> IRQ Line: 95  */
#define NVIC_ISER3    ((_IO uint32_t*)0xE000E10C) /*IRQ Line: 96 <-> IRQ Line: 127 */
#define NVIC_ISER4    ((_IO uint32_t*)0xE000E110) /*IRQ Line: 128 <-> IRQ Line: 159*/
#define NVIC_ISER5    ((_IO uint32_t*)0xE000E114) /*IRQ Line: 160 <-> IRQ Line: 191*/
#define NVIC_ISER6    ((_IO uint32_t*)0xE000E118) /*IRQ Line: 192 <-> IRQ Line: 223*/
#define NVIC_ISER7    ((_IO uint32_t*)0xE000E11C) /*IRQ Line: 224 <-> IRQ Line: 239*/


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 * Interrupt Clear Enable Register
 * Permission: RW
 */
#define NVIC_ICER0    ((_IO uint32_t*)0xE000E180) /*IRQ Line: 0  <-> IRQ Line: 31  */
#define NVIC_ICER1    ((_IO uint32_t*)0xE000E184) /*IRQ Line: 32 <-> IRQ Line: 63  */
#define NVIC_ICER2    ((_IO uint32_t*)0xE000E188) /*IRQ Line: 64 <-> IRQ Line: 95  */
#define NVIC_ICER3    ((_IO uint32_t*)0xE000E18C) /*IRQ Line: 96 <-> IRQ Line: 127 */
#define NVIC_ICER4    ((_IO uint32_t*)0xE000E190) /*IRQ Line: 128 <-> IRQ Line: 159*/
#define NVIC_ICER5    ((_IO uint32_t*)0xE000E194) /*IRQ Line: 160 <-> IRQ Line: 191*/
#define NVIC_ICER6    ((_IO uint32_t*)0xE000E198) /*IRQ Line: 192 <-> IRQ Line: 223*/
#define NVIC_ICER7    ((_IO uint32_t*)0xE000E19C) /*IRQ Line: 224 <-> IRQ Line: 239*/


/*
 * ARM Cortex Mx Processor NVIC ISPRx register Addresses
 * Interrupt Set Pending Register
 * Permission: RW
 */
#define NVIC_ISPR0    ((_IO uint32_t*)0xE000E200) /*IRQ Line: 0  <-> IRQ Line: 31  */
#define NVIC_ISPR1    ((_IO uint32_t*)0xE000E204) /*IRQ Line: 32 <-> IRQ Line: 63  */
#define NVIC_ISPR2    ((_IO uint32_t*)0xE000E208) /*IRQ Line: 64 <-> IRQ Line: 95  */
#define NVIC_ISPR3    ((_IO uint32_t*)0xE000E20C) /*IRQ Line: 96 <-> IRQ Line: 127 */
#define NVIC_ISPR4    ((_IO uint32_t*)0xE000E210) /*IRQ Line: 128 <-> IRQ Line: 159*/
#define NVIC_ISPR5    ((_IO uint32_t*)0xE000E214) /*IRQ Line: 160 <-> IRQ Line: 191*/
#define NVIC_ISPR6    ((_IO uint32_t*)0xE000E218) /*IRQ Line: 192 <-> IRQ Line: 223*/
#define NVIC_ISPR7    ((_IO uint32_t*)0xE000E21C) /*IRQ Line: 224 <-> IRQ Line: 239*/

/*
 * ARM Cortex Mx Processor NVIC ICPRx register Addresses
 * Interrupt Clear Pending Register
 * Permission: RW
 */
#define NVIC_ICPR0    ((_IO uint32_t*)0xE000E280) /*IRQ Line: 0  <-> IRQ Line: 31  */
#define NVIC_ICPR1    ((_IO uint32_t*)0xE000E284) /*IRQ Line: 32 <-> IRQ Line: 63  */
#define NVIC_ICPR2    ((_IO uint32_t*)0xE000E288) /*IRQ Line: 64 <-> IRQ Line: 95  */
#define NVIC_ICPR3    ((_IO uint32_t*)0xE000E28C) /*IRQ Line: 96 <-> IRQ Line: 127 */
#define NVIC_ICPR4    ((_IO uint32_t*)0xE000E290) /*IRQ Line: 128 <-> IRQ Line: 159*/
#define NVIC_ICPR5    ((_IO uint32_t*)0xE000E294) /*IRQ Line: 160 <-> IRQ Line: 191*/
#define NVIC_ICPR6    ((_IO uint32_t*)0xE000E298) /*IRQ Line: 192 <-> IRQ Line: 223*/
#define NVIC_ICPR7    ((_IO uint32_t*)0xE000E29C) /*IRQ Line: 224 <-> IRQ Line: 239*/


/*
 * ARM Cortex Mx Processor NVIC IABRx register Addresses
 * Interrupt Active Bit Register
 * Permission: RO
 */
#define NVIC_IABR0    ((_IO uint32_t*)0xE000E300) /*IRQ Line: 0  <-> IRQ Line: 31  */
#define NVIC_IABR1    ((_IO uint32_t*)0xE000E304) /*IRQ Line: 32 <-> IRQ Line: 63  */
#define NVIC_IABR2    ((_IO uint32_t*)0xE000E308) /*IRQ Line: 64 <-> IRQ Line: 95  */
#define NVIC_IABR3    ((_IO uint32_t*)0xE000E30C) /*IRQ Line: 96 <-> IRQ Line: 127 */
#define NVIC_IABR4    ((_IO uint32_t*)0xE000E310) /*IRQ Line: 128 <-> IRQ Line: 159*/
#define NVIC_IABR5    ((_IO uint32_t*)0xE000E314) /*IRQ Line: 160 <-> IRQ Line: 191*/
#define NVIC_IABR6    ((_IO uint32_t*)0xE000E318) /*IRQ Line: 192 <-> IRQ Line: 223*/
#define NVIC_IABR7    ((_IO uint32_t*)0xE000E31C) /*IRQ Line: 224 <-> IRQ Line: 239*/

/*
 * Interrupt Controller Type Register ICTR
 * Permission: RO
 */
#define NVIC_ICTR    ((_IO uint32_t*)0xE000E31C) /*IRQ Line: 224 <-> IRQ Line: 239*/

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 * Permission: RW
 */
#define NVIC_PR_BASE_ADDR    ((_IO uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor Software Trigger Interrupt Register
 * Permission: WO
 */
#define NVIC_STIR_BASE_ADDR    ((_IO uint32_t*)0xE000EF00)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED    4


#endif /* INC_ARM_CORTEX_M4_H_ */
