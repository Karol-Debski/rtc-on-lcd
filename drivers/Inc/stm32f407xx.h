/*
 * stm32f407xx.h
 *
 *  Created on: Oct 22, 2023
 *      Author: karol
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * base address of Flash and SRAM
 */

#define FLASH_BASE_ADDR				0x08000000U
#define SRAM1_BASE_ADDR				0x20000000U
#define SRAM2_BASE_ADDR				0x20001C00U
#define ROM_BASE_ADDR				0x1FFF0000U
#define SRAM 						SRAM1_BASE_ADDR

/*
 * AHBx and APBx Bus Peripheral
 */

#define PERIPH_BASEADDR 		0X40000000U
#define APB1PERIPH_BASEADDR		PERIPH__BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * GPIO base address
 */

#define GPIOA_BASEADDR		(AHB1PERIPH__BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH__BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH__BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH__BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH__BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH__BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH__BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH__BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH__BASEADDR + 0x2000)


/*
 * RCC base addr
 */
#define RCC_BASEADDR 		(AHB1PERIPH__BASEADDR + 0x3800)


/*
 * Clock enable macros for gpio
 */
#define GPIOA_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<1))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1 << 21))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PERI_CLOCK_EN() 	(RCC->APB2ENR |= (1 << 12))


/*
 * Clock enable macros for USARTx peripherals
 */

#define	USART2_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1 << 17))

/*
 * Clock enable macros for syscfg peripherals
 */

#define	SYSCFG_PERI_CLOCK_EN()	(RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLOCK_DI() 	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PERI_CLOCK_DI()	(RCC->AHB1ENR &= ~(1<<1))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PERI_CLOCK_DI()	(RCC->APB1ENR &= ~(1 << 21))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PERI_CLOCK_DI() 	(RCC->APB2ENR &= ~(1 << 12))


/*
 * Clock disable macros for USARTx peripherals
 */

#define	USART2_PERI_CLOCK_DI()	(RCC->APB1ENR &= ~(1 << 17))

/*
 * Clock disable macros for syscfg peripherals
 */

#define	SYSCFG_PERI_CLOCK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Base addr of APB1 bus periphals
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)


/*
 * Definitions of peripherals register structures
 */

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2;
	__vo uint32_t RESERVED3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t RESERVED6;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED8;
	__vo uint32_t RESERVED9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED10;
	__vo uint32_t RESERVED11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * Peripherals definitions - base address type casted to x_RegDef_t)
 */

#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 	((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)






#endif /* INC_STM32F407XX_H_ */
