/*
 * stm32f446xx.h
 *
 *  Created on: Nov 26, 2019
 *      Author: vverma
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include "arm_cortex_m4.h"


/*
 * Embedded Flash Memory and SRAM Base Address
 *  */
#define FLASH_BASEADDR			0x08000000U	/*Flash 512 Kbytes Sector0 - Sector7*/
#define SRAM1_BASEADDR			0x20000000U /*SRAM1 112 Kbytes*/
#define SRAM2_BASEADDR			0x2001C000U /*SRAM2 16 Kbytes*/
#define ROM_BASEADDR			0x1FFF0000U /*System Reserved Memory 30 Kbytes*/
#define OTP_BASEADDR			0x1FFF7800U	/*One time programmable 528 Kbytes*/
#define OPT_BYTES_BASEADDR		0x1FFFC000U /*Option bytes 16 bytes*/
#define SRAM					SRAM1_BASEADDR


/*
 * APBx and AHBx Peripheral Bus Base Address
 * */
#define PERIPH_BASEADDR			0x40000000U	/*Peripheral Base Address*/
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR	/* APB1 0x40000000 to 0x40007FFF*/
#define APB2PERIPH_BASEADDR		0x40010000U /* APB2 0X40010000 to 0x40016BFF*/
#define AHB1PERIPH_BASEADDR		0x40020000U	/* AHB1 0x40020000 to 0x4007FFFF*/
#define AHB2PERIPH_BASEADDR		0x50000000U	/* AHB2 0x50000000 to 0x50060BFF*/
#define AHB3PERIPH_BASEADDR		0x60000000U	/* AHB3 0x60000000 to 0xDFFFFFFF*/


/*
 * AHB1 Peripherals Base Address
 * */
#define GPIOA_BASEADDR			AHB1PERIPH_BASEADDR	/*GPIOA Base Address Reset Value: 0xA800 0000*/
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + (0x0400U)) /*GPIOB Base Address Reset Value: 0x0000 0280*/
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + (0x0800U)) /*GPIOC Base Address Reset Value: 0x0000 0000*/
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + (0x0C00U)) /*GPIOD Base Address Reset Value: 0x0000 0000*/
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + (0x1000U)) /*GPIOE Base Address Reset Value: 0x0000 0000*/
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + (0x1400U)) /*GPIOF Base Address Reset Value: 0x0000 0000*/
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + (0x1800U)) /*GPIOG Base Address Reset Value: 0x0000 0000*/
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + (0x1C00U)) /*GPIOH Base Address Reset Value: 0x0000 0000*/
#define CRC_BASEADDR			(AHB1PERIPH_BASEADDR + (0x3000U))
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + (0x3800U))
#define FIS_BASEADDR			(AHB1PERIPH_BASEADDR + (0x3C00U))
#define BKPSRAM_BASEADDR		(AHB1PERIPH_BASEADDR + (0x4000U))
#define DMA1_BASEADDR			(AHB1PERIPH_BASEADDR + (0x6000U))
#define DMA2_BASEADDR			(AHB1PERIPH_BASEADDR + (0x6400U))
#define USBOTG_HS_BASEADDR		(AHB1PERIPH_BASEADDR + (0x20000U))


/*
 * APB1 Peripherals Base Address
 * */
#define TIM2_BASEADDR			APB1PERIPH_BASEADDR
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + (0x0400U))
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + (0x0800U))
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + (0x0C00U))
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + (0x1000U))
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + (0x1400U))
#define TIM12_BASEADDR			(APB1PERIPH_BASEADDR + (0x1800U))
#define TIM13_BASEADDR			(APB1PERIPH_BASEADDR + (0x1C00U))
#define TIM14_BASEADDR			(APB1PERIPH_BASEADDR + (0x2000U))
#define RTC_BASEADDR			(APB1PERIPH_BASEADDR + (0x2800U))
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + (0x2C00U))
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + (0x3000U))
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + (0x3800U))
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + (0x3C00U))
#define SPDIF_BASEADDR			(APB1PERIPH_BASEADDR + (0x4000U))
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + (0x4400U))
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + (0x4800U))
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + (0x4C00U))
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + (0x5000U))
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + (0x5400U))
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + (0x5800U))
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + (0x5C00U))
#define CAN1_BASEADDR			(APB1PERIPH_BASEADDR + (0x6400U))
#define CAN2_BASEADDR			(APB1PERIPH_BASEADDR + (0x6800U))
#define HDMI_BASEADDR			(APB1PERIPH_BASEADDR + (0x6C00U))
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + (0x7000U))
#define DAC_BASEADDR			(APB1PERIPH_BASEADDR + (0x7400U))


/*
 * APB2 Peripherals Base Address
 * */
#define TIM1_BASEADDR			APB2PERIPH_BASEADDR
#define TIM8_BASEADDR			(APB2PERIPH_BASEADDR + (0x0400U))
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + (0x1000U))
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + (0x1400U))
#define ADCx_BASEADDR			(APB2PERIPH_BASEADDR + (0x2000U)) /*Verify this*/
#define SDMMC_BASEADDR			(APB2PERIPH_BASEADDR + (0x2C00U))
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + (0x3000U))
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + (0x3400U))
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + (0x3800U))
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + (0x3C00U))
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + (0x4000U))
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + (0x4400U))
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + (0x4800U))
#define SAI1_BASEADDR			(APB2PERIPH_BASEADDR + (0x5800U))
#define SAI2_BASEADDR			(APB2PERIPH_BASEADDR + (0x5C00U))

/*
 * GPIOx Peripheral Registers
 * */
typedef struct {
	_IO uint32_t moder; /* Control the pin mode of the GPIOx port ( Input/Output/Alternate/Analog ) */
	_IO uint32_t otyper; /* Output type Pin Configuration  ( Push Pull / Open Drain ) */
	_IO uint32_t ospeedr; /* Output type Speed Pin Configuration (Low/Med/Fast/High)  */
	_IO uint32_t pupdr; /* Internal Pull up Pull down Pin configuration */
	_IO uint32_t idr;
	_IO uint32_t odr;
	_IO uint32_t bsrr;
	_IO uint32_t lckr;
	_IO uint32_t afrl;
	_IO uint32_t afrh;
} gpiox_reg_def_t;

#define GPIOA	((gpiox_reg_def_t *) GPIOA_BASEADDR)
#define GPIOB	((gpiox_reg_def_t *) GPIOB_BASEADDR)
#define GPIOC	((gpiox_reg_def_t *) GPIOC_BASEADDR)
#define GPIOD	((gpiox_reg_def_t *) GPIOD_BASEADDR)
#define GPIOE	((gpiox_reg_def_t *) GPIOE_BASEADDR)
#define GPIOF	((gpiox_reg_def_t *) GPIOF_BASEADDR)
#define GPIOG	((gpiox_reg_def_t *) GPIOG_BASEADDR)
#define GPIOH	((gpiox_reg_def_t *) GPIOH_BASEADDR)


/*
 *  USART/UART Peripheral Register Map
 * */
typedef struct{
	_IO uint32_t sr;    /*status register*/
	_IO uint32_t dr;    /*data register r/w */
    _IO uint32_t brr;
	_IO uint32_t cr1;
	_IO uint32_t cr2;
	_IO uint32_t cr3;
	_IO uint32_t gtpr;
}usartx_reg_def_t;

#define USART1	((usartx_reg_def_t*) USART1_BASEADDR)
#define USART2	((usartx_reg_def_t*) USART2_BASEADDR)
#define USART3	((usartx_reg_def_t*) USART3_BASEADDR)
#define UART4	((usartx_reg_def_t*) UART4_BASEADDR)
#define UART5	((usartx_reg_def_t*) UART5_BASEADDR)
#define USART6	((usartx_reg_def_t*) USART6_BASEADDR)

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	_IO uint32_t mem_rmp;       /*!< Give a short description,                    Address offset: 0x00      */
	_IO uint32_t pmc;          /*!< TODO,     									  Address offset: 0x04      */
	_IO uint32_t exti_cr[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	    uint32_t reserved1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	_IO uint32_t cmpcr;        /*!< TODO         								  Address offset: 0x20      */
	    uint32_t reserved2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	_IO uint32_t cfgr;         /*!< TODO                                         Address offset: 0x2C   	*/
} syscfg_reg_def_t;

#define SYSCFG				((syscfg_reg_def_t*)SYSCFG_BASEADDR)

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	_IO uint32_t imr;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	_IO uint32_t emr;    /*!< TODO,                						Address offset: 0x04 */
	_IO uint32_t rtsr;   /*!< TODO,  									     Address offset: 0x08 */
	_IO uint32_t ftsr;   /*!< TODO, 										Address offset: 0x0C */
	_IO uint32_t swier;  /*!< TODO,  									   Address offset: 0x10 */
	_IO uint32_t pr;     /*!< TODO,                   					   Address offset: 0x14 */

}exti_reg_def_t;

#define EXTI				((exti_reg_def_t*)EXTI_BASEADDR)

/*
 *  RCC Peripheral Register Map
 * */
typedef struct {
	_IO uint32_t cr;
	_IO uint32_t pll_cfgr;
	_IO uint32_t cfgr;
	_IO uint32_t cir;
	_IO uint32_t ahb1_rstr;
	_IO uint32_t ahb2_rstr;
	_IO uint32_t ahb3_rstr;
	uint32_t  reserved_0;
	_IO uint32_t apb1_rstr;
	_IO uint32_t apb2_rstr;
	uint32_t reserved_1;
	uint32_t reserved_2;
	_IO uint32_t ahb1_enr;
	_IO uint32_t ahb2_enr;
	_IO uint32_t ahb3_enr;
	uint32_t reserved_3;
	_IO uint32_t apb1_enr;
	_IO uint32_t apb2_enr;
	uint32_t reserved_4;
	uint32_t reserved_5;
	_IO uint32_t ahb1_lpenr;
	_IO uint32_t ahb2_lpenr;
	_IO uint32_t ahb3_lpenr;
	uint32_t reserved_6;
	_IO uint32_t apb1_lpenr;
	_IO uint32_t apb2_lpenr;
	uint32_t reserved_7;
	uint32_t reserved_8;
	_IO uint32_t bdcr;
	_IO uint32_t csr;
	uint32_t reserved_9;
	uint32_t reserved_10;
	_IO uint32_t sscgr;
	_IO uint32_t PLLI2_SCFGR;
	_IO uint32_t PLL_SAI_CSFGR;
	_IO uint32_t DCK_CFGR;
	_IO uint32_t CK_GATE_NR;
	_IO uint32_t DCK_CFGR2;
} rcc_reg_def_t;

#define RCC		((rcc_reg_def_t* ) RCC_BASEADDR)



/* Direct Memory Access Controller Register Map*/
typedef struct {
	_IO uint32_t dma_sxcr;         /* DMA stream x configuration register      Address offset:0x0 0x10 */
	_IO uint32_t dma_sxndtr;    /* DMA stream x number of data register        Address offset:0x04 0x14 */
	_IO uint32_t dma_sxpar;     /* DMA stream x peripheral address register 	  Address offset:0x08 0x18 */
	_IO uint32_t dma_sxm0ar;    /* DMA stream x memory 0 address register  	  Address offset: C0x1C */
	_IO uint32_t dma_sxm1ar;    /* DMA stream x memory 1 address register      Address offset:10 0x20 */
	_IO uint32_t dma_sxfcr;     /* DMA stream x FIFO Control Register      	  Address offset: 14 0x24 */
}streamx;

typedef struct {
	_IO uint32_t dma_lisp;         /* DMA low interrupt status register           Address offset: 0x00 */
	_IO uint32_t dma_hisr;         /* DMA high interrupt status register          Address offset: 0x04 */
	_IO uint32_t dma_lifcr;        /* DMA low interrupt flag clear register       Address offset: 0x08 */
	_IO uint32_t dma_hifcr;        /* DMA high interrupt flag clear register      Address offset: 0x0C */
	streamx streams[8];
	//_IO uint32_t dma_sxcr[8];         /* DMA stream x configuration register      Address offset: 0x10 */
	//_IO uint32_t dma_sxndtr[8];    /* DMA stream x number of data register        Address offset: 0x14 */
	//_IO uint32_t dma_sxpar[8];     /* DMA stream x peripheral address register 	  Address offset: 0x18 */
	//_IO uint32_t dma_sxm0ar[8];    /* DMA stream x memory 0 address register  	  Address offset: 0x1C */
	//_IO uint32_t dma_sxm1ar[8];    /* DMA stream x memory 1 address register      Address offset: 0x20 */
	//_IO uint32_t dma_sxfcr[8];     /* DMA stream x FIFO Control Register      	  Address offset: 0x24 */
} dmax_reg_def_t;

#define DMA1	((dmax_reg_def_t* ) DMA1_BASEADDR)
#define DMA2	((dmax_reg_def_t* ) DMA2_BASEADDR)

/*
 *  GPIOx Peripheral clock Enable
 * */
#define GPIOA_PCLK_EN()			(RCC->ahb1_enr |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->ahb1_enr |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->ahb1_enr |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->ahb1_enr |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->ahb1_enr |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->ahb1_enr |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->ahb1_enr |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->ahb1_enr |= (1<<7))


/*
 *  GPIOx Peripheral clock Disable
 * */
#define GPIOA_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<4))
#define GPIOF_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<5))
#define GPIOG_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<6))
#define GPIOH_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<7))


/*
 *  CRC Peripheral clock Enable
 * */
#define CRC_PCLK_EN()			(RCC->ahb1_enr |= (1<<12))

/*
 *  CRC Peripheral clock Disable
 * */
#define CRC_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<12))


/*
 *  DMA1 Peripheral clock Enable
 * */
#define DMA1_PCLK_EN()			(RCC->ahb1_enr |= (1<<21))

/*
 *  DMA1 Peripheral clock Disable
 * */
#define DMA1_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<21))


/*
 *  DMA2 Peripheral clock Enable
 * */
#define DMA2_PCLK_EN()			(RCC->ahb1_enr |= (1<<22))

/*
 *  DMA1 Peripheral clock Disable
 * */
#define DMA2_PCLK_DI()			(RCC->ahb1_enr &= ~(1<<22))


/*
 *  CRC Peripheral clock Enable
 * */
#define CRC_PCLK_EN()			(RCC->ahb1_enr |= (1<<12))


/*
 *  BKP_SRAM Peripheral clock Enable
 * */

#define BKP_SRAM_PCLK_EN()		(RCC->ahb1_enr |= (1<<18))


/*
 *  BKP_SRAM Peripheral clock Disable
 * */

#define BKP_SRAM_PCLK_DI()		(RCC->ahb1_enr &= ~(1<<18))


/*
 *  I2Cx Peripheral clock Enable
 * */
#define I2C1_PCLK_EN()			(RCC->apb1_enr |= (1<<21))
#define I2C2_PCLK_EN()			(RCC->apb1_enr |= (1<<22))
#define I2C3_PCLK_EN()			(RCC->apb1_enr |= (1<<23))


/*
 *  I2Cx Peripheral clock Disable
 * */
#define I2C1_PCLK_DI()			(RCC->apb1_enr &= ~(1<<21))
#define I2C2_PCLK_DI()			(RCC->apb1_enr &= ~(1<<22))
#define I2C3_PCLK_DI()			(RCC->apb1_enr &= ~(1<<23))


/*
 *  SPIx Peripheral clock Enable
 * */
#define SPI1_PCLK_EN()			(RCC->apb2_enr |= (1<<12))
#define SPI2_PCLK_EN()			(RCC->apb1_enr |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->apb1_enr |= (1<<15))
#define SPI4_PCLK_EN()			(RCC->apb2_enr |= (1<<13))


/*
 *  SPIx Peripheral clock Disable
 * */
#define SPI1_PCLK_DI()			(RCC->apb2_enr &= ~(1<<12))
#define SPI2_PCLK_DI()			(RCC->apb1_enr &= ~(1<<14))
#define SPI3_PCLK_DI()			(RCC->apb1_enr &= ~(1<<15))
#define SPI4_PCLK_DI()			(RCC->apb2_enr &= ~(1<<13))


/*
 *  USARTx Peripheral clock Enable
 * */
#define USART1_PCLK_EN()		(RCC->apb2_enr |= (1<<4))
#define USART2_PCLK_EN()		(RCC->apb1_enr |= (1<<17))
#define USART3_PCLK_EN()		(RCC->apb1_enr |= (1<<18))
#define UART4_PCLK_EN()		(RCC->apb2_enr |= (1<<19))
#define UART5_PCLK_EN()		(RCC->apb1_enr |= (1<<20))
#define USART6_PCLK_EN()		(RCC->apb2_enr |= (1<<5))


/*
 *  USARTx Peripheral clock Disable
 * */
#define USART1_PCLK_DI()		(RCC->apb2_enr &= ~(1<<4))
#define USART2_PCLK_DI()		(RCC->apb1_enr &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->apb1_enr &= ~(1<<18))
#define UART4_PCLK_DI()		(RCC->apb2_enr &= ~(1<<19))
#define UART5_PCLK_DI()		(RCC->apb1_enr &= ~(1<<20))
#define USART6_PCLK_DI()		(RCC->apb2_enr &= ~(1<<5))


/*
 *  SYSCFG Peripheral clock Enable
 * */
#define SYSCFG_PCLK_EN()		(RCC->apb2_enr |= (1<<14))


/*
 *  SYSCFG Peripheral clock Disable
 * */
#define SYSCFG_PCLK_DI()			(RCC->apb2_enr &= ~(1<<14))


/*
 *  GPIOx Reset macro function
 * */
#define GPIOA_RESET()			do{ RCC->ahb1_rstr |= (1<<0); RCC->ahb1_rstr &= ~(1<<0);}while(0)
#define GPIOB_RESET()			do{ RCC->ahb1_rstr |= (1<<1); RCC->ahb1_rstr &= ~(1<<1);}while(0)
#define GPIOC_RESET()			do{ RCC->ahb1_rstr |= (1<<2); RCC->ahb1_rstr &= ~(1<<2);}while(0)
#define GPIOD_RESET()			do{ RCC->ahb1_rstr |= (1<<3); RCC->ahb1_rstr &= ~(1<<3);}while(0)
#define GPIOE_RESET()			do{ RCC->ahb1_rstr |= (1<<4); RCC->ahb1_rstr &= ~(1<<4);}while(0)
#define GPIOF_RESET()			do{ RCC->ahb1_rstr |= (1<<5); RCC->ahb1_rstr &= ~(1<<5);}while(0)
#define GPIOG_RESET()			do{ RCC->ahb1_rstr |= (1<<6); RCC->ahb1_rstr &= ~(1<<6);}while(0)
#define GPIOH_RESET()			do{ RCC->ahb1_rstr |= (1<<7); RCC->ahb1_rstr &= ~(1<<7);}while(0)


/*
 * Generic macro
 * */
#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */
#define IRQ_NO_WWDG               0
#define IRQ_NO_EXTI0 	          6
#define IRQ_NO_EXTI1 	          7
#define IRQ_NO_EXTI2 	          8
#define IRQ_NO_EXTI3 	          9
#define IRQ_NO_EXTI4 	         10
#define IRQ_NO_DMA1_STREAM0 	 11
#define IRQ_NO_DMA1_STREAM1 	 12
#define IRQ_NO_DMA1_STREAM2 	 13
#define IRQ_NO_DMA1_STREAM3 	 14
#define IRQ_NO_DMA1_STREAM4 	 15
#define IRQ_NO_DMA1_STREAM5 	 16
#define IRQ_NO_DMA1_STREAM6 	 17
#define IRQ_NO_EXTI9_5 	         23
#define IRQ_NO_I2C1_EV           31
#define IRQ_NO_I2C1_ER           32
#define IRQ_NO_SPI1		         35
#define IRQ_NO_SPI2              36
#define IRQ_NO_USART1	         37
#define IRQ_NO_USART2	         38
#define IRQ_NO_USART3	         39
#define IRQ_NO_EXTI15_10         40
#define IRQ_NO_DMA1_STREAM7 	 47
#define IRQ_NO_SPI3              51
#define IRQ_NO_UART4	         52
#define IRQ_NO_UART5	         53
#define IRQ_NO_DMA2_STREAM0 	 56
#define IRQ_NO_DMA2_STREAM1 	 57
#define IRQ_NO_DMA2_STREAM2 	 58
#define IRQ_NO_DMA2_STREAM3 	 59
#define IRQ_NO_DMA2_STREAM4 	 60
#define IRQ_NO_DMA2_STREAM5 	 68
#define IRQ_NO_DMA2_STREAM6 	 69
#define IRQ_NO_DMA3_STREAM7 	 70
#define IRQ_NO_USART6	         71
#define IRQ_NO_SPI4              84




#include "arm_cortex_m4_nvic_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx_dma_driver.h"



#endif /* INC_STM32F446XX_H_ */
