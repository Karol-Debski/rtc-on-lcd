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

/************************************************** START: Processor specific details **************************************************/

#define NVIC_ISER0	((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1	((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2	((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3	((__vo uint32_t*)0xE000E10C)


#define NVIC_ICER0	((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1	((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2	((__vo uint32_t*)0XE000E18C)
#define NVIC_ICER3	((__vo uint32_t*)0XE000E190)

#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4

/************************************************** STOP: Processor specific details **************************************************/



/************************************************** START: MCU specific details **************************************************/

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
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * GPIOx base address
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000)

/*
 * RCC base addr
 */
#define RCC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * EXTI base addr
 */
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00)

/*
 * SYSCFG base addr
 */
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)



/*
 * Clock enable macros for GPIOx
 */
#define GPIOA_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PERI_CLOCK_EN()	(RCC->AHB1ENR |= (1<<8))

/*
 * Clock disable macros for GPIOx
 */
#define GPIOA_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PERI_CLOCK_DIS()	(RCC->AHB1ENR &= ~(1<<8))

/*
 * Reset GPIOx registers MACRO
 */
#define GPIOA_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REGISTERS_RESET() do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1 << 21))

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PERI_CLOCK_EN() 		(RCC->APB2ENR |= (1U << 12))
#define SPI2_I2S2_PERI_CLOCK_EN() 	(RCC->APB1ENR |= (1U << 14))
#define SPI3_I2S3_PERI_CLOCK_EN() 	(RCC->APB1ENR |= (1U << 15))

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PERI_CLOCK_DIS() 		(RCC->APB2ENR &= ~(1U << 12))
#define SPI2_I2S2_PERI_CLOCK_DIS() 	(RCC->APB1ENR &= ~(1U << 14))
#define SPI3_I2S3_PERI_CLOCK_DIS() 	(RCC->APB1ENR &= ~(1U << 15))

/*
 * Clock enable macros for USARTx peripherals
 */
#define	USART2_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1 << 17))

/*
 * Clock enable macros for SYSCFG peripherals
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
 * Clock disable macros for SYSCFG peripherals
 */
#define	SYSCFG_PERI_CLOCK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macro to get port code from a GPIO base address
 */
#define GPIO_BASEADDR_TO_PORT_CODE(addr)	((addr == GPIOA) ? 0 :\
											 (addr == GPIOB) ? 1 :\
										     (addr == GPIOC) ? 2 :\
										     (addr == GPIOD) ? 3 :\
										     (addr == GPIOE) ? 4 :\
										     (addr == GPIOF) ? 5 :\
										     (addr == GPIOG) ? 6 :\
										     (addr == GPIOH) ? 7 :\
										     (addr == GPIOI) ? 8 :0)

/*
 * Base addr of APB1 bus periphals
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0X3000U)
#define SPI2_I2S2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_I2S3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00U)





#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

/*
 * peripheral register definition structure for GPIO
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
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
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
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
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
}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t      RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t      RESERVED2[2];
	__vo uint32_t CFGR;
} SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


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


#define SPI1 		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	 	((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)
#define SPI3 		((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)







#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI 	((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



#define IRQ_NUM_EXTI0 		6
#define IRQ_NUM_EXTI1 		7
#define IRQ_NUM_EXTI2 		8
#define IRQ_NUM_EXTI3 		9
#define IRQ_NUM_EXTI4 		10
#define IRQ_NUM_EXTI9_5 	23
#define IRQ_NUM_EXTI15_10 	40

/************************************************** STOP: MCU specific details **************************************************/

/*
 * Generic macros
 */
#define ENABLE 	1
#define DISABLE 0
#define SET 	ENABLE
#define RESET 	DISABLE


#define BIT_MASK(bitNum)								((1U) << (bitNum))
#define BIT_CLEAR(reg, bitNum)							((reg) & ~BIT_MASK(bitNum))
#define BIT_SET_VAL(reg, val, bitNum)					(reg = (BIT_CLEAR(reg, bitNum) | (((val) << bitNum) & BIT_MASK(bitNum))))
#define BIT_READ(reg, bitNum)							(0U != ((reg) & BIT_MASK(bitNum)))

#define MULTI_BIT_MASK(bitNum, len)						(((1U << len) - 1U) << bitNum)
#define MULTI_BIT_CLEAR(reg, bitNum, len)				((reg) & ~(MULTI_BIT_MASK(bitNum, len)))
#define MULTI_BIT_SET_VAL(reg, val, bitNum, len)		(reg = (MULTI_BIT_CLEAR(reg, bitNum, len) | ((val << bitNum) & MULTI_BIT_MASK(bitNum, len))))
#define MULTI_BIT_READ(reg, bitNum, len)				(((reg) & MULTI_BIT_MASK(bitNum, len)) >> bitNum)

#include "stm32407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
