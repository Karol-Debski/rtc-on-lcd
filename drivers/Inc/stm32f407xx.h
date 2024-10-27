/*
 * stm32f407xx.h
 *
 *  Created on: Oct 22, 2023
 *      Author: karol
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/************************************************** START: Processor specific details **************************************************/

/*
 * Set interrupt registers
 */
#define NVIC_ISER0	((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1	((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2	((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3	((__vo uint32_t*)0xE000E10C)

/*
 * Clear interrupt registers
 */
#define NVIC_ICER0	((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1	((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2	((__vo uint32_t*)0XE000E18C)
#define NVIC_ICER3	((__vo uint32_t*)0XE000E190)

/*
 * Set pending registers
 */
#define NVIC_ISPR0	((__vo uint32_t*)0XE000E200)
#define NVIC_ISPR1	((__vo uint32_t*)0XE000E204)
#define NVIC_ISPR2	((__vo uint32_t*)0XE000E208)
#define NVIC_ISPR3	((__vo uint32_t*)0XE000E20C)
#define NVIC_ISPR4	((__vo uint32_t*)0XE000E210)
#define NVIC_ISPR5	((__vo uint32_t*)0XE000E214)
#define NVIC_ISPR6	((__vo uint32_t*)0XE000E218)
#define NVIC_ISPR7	((__vo uint32_t*)0XE000E21C)

/*
 * Clear pending registers
 */
#define NVIC_ICPR0	((__vo uint32_t*)0XE000E280)
#define NVIC_ICPR1	((__vo uint32_t*)0XE000E284)
#define NVIC_ICPR2	((__vo uint32_t*)0XE000E288)
#define NVIC_ICPR3	((__vo uint32_t*)0XE000E28C)
#define NVIC_ICPR4	((__vo uint32_t*)0XE000E290)
#define NVIC_ICPR5	((__vo uint32_t*)0XE000E294)
#define NVIC_ICPR6	((__vo uint32_t*)0XE000E298)
#define NVIC_ICPR7	((__vo uint32_t*)0XE000E29C)


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
#define I2C2_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1 << 23))

/*
 * Reset I2Cx registers MACRO
 */
#define I2C1_REGISTERS_RESET() do{(RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21));}while(0)
#define I2C2_REGISTERS_RESET() do{(RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REGISTERS_RESET() do{(RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23));}while(0)

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
 * Clock enable macros for USARTx/UARTx peripherals
 */
#define	USART1_PERI_CLOCK_EN()	(RCC->APB2ENR |= (1U << 4))
#define	USART2_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1U << 17))
#define	USART3_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1U << 18))
#define	UART4_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1U << 19))
#define	UART5_PERI_CLOCK_EN()	(RCC->APB1ENR |= (1U << 20))
#define	USART6_PERI_CLOCK_EN()	(RCC->APB2ENR |= (1U << 5))

/*
 * Clock disable macros for USARTx/UARTx peripherals
 */
#define	USART1_PERI_CLOCK_DIS()	(RCC->APB2ENR &= ~(1U << 4))
#define	USART2_PERI_CLOCK_DIS()	(RCC->APB1ENR &= ~(1U << 17))
#define	USART3_PERI_CLOCK_DIS()	(RCC->APB1ENR &= ~(1U << 18))
#define	UART4_PERI_CLOCK_DIS()	(RCC->APB1ENR &= ~(1U << 19))
#define	UART5_PERI_CLOCK_DIS()	(RCC->APB1ENR &= ~(1U << 20))
#define	USART6_PERI_CLOCK_DIS()	(RCC->APB2ENR &= ~(1U << 5))

/*
 * Reset USARTx/UARTx registers MACRO
 */
#define USART1_REGISTERS_RESET()	do{(RCC->APB2RSTR |= (1U<<4)); (RCC->APB1RSTR &= ~(1U<<4));}while(0)
#define USART2_REGISTERS_RESET() 	do{(RCC->APB1RSTR |= (1U<<17)); (RCC->APB1RSTR &= ~(1U<<17));}while(0)
#define USART3_REGISTERS_RESET() 	do{(RCC->APB1RSTR |= (1U<<18)); (RCC->APB1RSTR &= ~(1U<<18));}while(0)
#define UART4_REGISTERS_RESET() 	do{(RCC->APB1RSTR |= (1U<<19)); (RCC->APB1RSTR &= ~(1U<<19));}while(0)
#define UART5_REGISTERS_RESET() 	do{(RCC->APB1RSTR |= (1U<<20)); (RCC->APB1RSTR &= ~(1U<<20));}while(0)
#define USART6_REGISTERS_RESET() 	do{(RCC->APB2RSTR |= (1U<<5)); (RCC->APB1RSTR &= ~(1U<<5));}while(0)


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
#define I2C2_PERI_CLOCK_DI()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PERI_CLOCK_DI()	(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PERI_CLOCK_DI() 	(RCC->APB2ENR &= ~(1 << 12))

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


#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)


/*
 * peripheral register definition structure for NVIC
 */
typedef struct
{
	__vo uint32_t NVIC_ISER[8];
	__vo uint32_t NVIC_ICER[8];
	__vo uint32_t NVIC_ISPR[8];
	__vo uint32_t NVIC_ICPR[8];
	__vo uint32_t NVIC_IABR[8];
	__vo uint32_t NVIC_IPR[60];
	__vo uint32_t STIR;
} NVIC_RegDef_t;

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
 * peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;


/*
 * peripheral register definition structure for USARTx/UARTx
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;


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

#define I2C1 		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 		((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1 		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2 		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3 		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4 		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5 		((USART_RegDef_t*)UART5_BASEADDR)
#define USART6 		((USART_RegDef_t*)USART6_BASEADDR)



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
#define IRQ_NUM_SPI1		35
#define IRQ_NUM_SPI2		36
#define IRQ_NUM_SPI3		51
#define IRQ_NUM_I2C1_EVENT  31
#define IRQ_NUM_I2C1_ERROR  32
#define IRQ_NUM_I2C2_EVENT	33
#define IRQ_NUM_I2C2_ERROR	34
#define IRQ_NUM_I2C3_EVENT	72
#define IRQ_NUM_I2C3_ERROR	73
#define IRQ_NUM_USART1		37
#define IRQ_NUM_USART2		38
#define IRQ_NUM_USART3		39
#define IRQ_NUM_UART4		52
#define IRQ_NUM_UART5		53
#define IRQ_NUM_USART6		71

/************************************************** STOP: MCU specific details **************************************************/

/*
 * Generic macros
 */
#define ENABLE 	1
#define DISABLE 0
#define SET 	ENABLE
#define RESET 	DISABLE


#define BIT_MASK(bitNum)								((1U) << (bitNum))
#define BIT_CLEAR(reg, bitNum)							(reg = (reg) & ~BIT_MASK(bitNum))
#define BIT_SET(reg, bitNum)							(reg = (reg) | BIT_MASK(bitNum))
#define BIT_SET_VAL(reg, val, bitNum)					(reg = (BIT_CLEAR(reg, bitNum) | (((val) << bitNum) & BIT_MASK(bitNum))))
#define BIT_READ(reg, bitNum)							(0U != ((reg) & BIT_MASK(bitNum)))

#define MULTI_BIT_MASK(bitNum, len)						(((1U << len) - 1U) << bitNum)
#define MULTI_BIT_CLEAR(reg, bitNum, len)				(reg = (reg) & ~(MULTI_BIT_MASK(bitNum, len)))
#define MULTI_BIT_SET_VAL(reg, val, bitNum, len)		(reg = (MULTI_BIT_CLEAR(reg, bitNum, len) | ((val << bitNum) & MULTI_BIT_MASK(bitNum, len))))
#define MULTI_BIT_READ(reg, bitNum, len)				(((reg) & MULTI_BIT_MASK(bitNum, len)) >> bitNum)

#include "stm32f407xx_rcc_driver.h"
#include "stm32407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"

#endif /* INC_STM32F407XX_H_ */
