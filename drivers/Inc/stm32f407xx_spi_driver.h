/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Aug 20, 2024
 *      Author: karol
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	ESPI_DeviceMode 				SPI_DeviceMode;
	ESPI_BusConfig 					SPI_BusConfig;
	ESPI_SclkSpeed 					SPI_SclkSpeed;
	ESPI_DataFrameFormat 			SPI_DataFrameFormat;
	ESPI_ClockPolarity 				SPI_ClockPolarity;
	ESPI_ClockPhase 				SPI_ClockPhase;
	ESPI_SoftwareSlaveManagment 	SPI_SoftwareSlaveManagment;
}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t*	pSPIx;
	SPI_Config_t	SPIConfig;
}SPI_Handle_t;


typedef enum ESPI_BusConfig
{
	SPI_BUS_CONFIG_FULL_DUPLEX = 0x0,
	SPI_BUS_CONFIG_HALF_DUPLEX = 0x1,
	SPI_BUS_CONFIG_SIMPLEX_RX_ONLY = 0x2,
}ESPI_BusConfig;

typedef enum ESPI_DeviceMode
{
	SPI_DEVICE_MODE_SLAVE = 0x0,
	SPI_DEVICE_MODE_MASTER = 0x1,
}ESPI_DeviceMode;

typedef enum ESPI_SclkSpeed
{
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_2 = 0x0,
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_4 = 0x1,
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_8 = 0x2,
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_16 = 0x3,
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_32 = 0x4,
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_64 = 0x5,
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_128 = 0x6,
	SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_256 = 0x7,
}ESPI_SclkSpeed;

typedef enum ESPI_DataFrameFormat
{
	SPI_8_BIT = 0x0,
	SPI_16_BIT = 0x1,
}ESPI_DataFrameFormat;

typedef enum ESPI_ClockPolarity
{
	SPI_LOW_WHEN_IDLE = 0x0,
	SPI_HIGH_WHEN_IDLE = 0x1,
}ESPI_ClockPolarity;

typedef enum ESPI_ClockPhase
{
	SPI_SAMPLING_ON_THE_FIRST_EDGE = 0x0,
	SPI_SAMPLING_ON_THE_SECOND_EDGE = 0x1,
}ESPI_ClockPhase;


typedef enum ESPI_SoftwareSlaveManagment
{
	SPI_SOFTWARE_SLAVE_MANAGMENT_DISABLED = 0x0,
	SPI_SOFTWARE_SLAVE_MANAGMENT_ENABLED = 0x1,
}ESPI_SoftwareSlaveManagment;

typedef enum ESPI_ClockControlState
{
	SPI_CLOCK_DISABLE = 0x0,
	SPI_CLOCK_ENABLE = 0x1,
}ESPI_ClockControlState;

typedef enum ESPI_IRQControlState
{
	SPI_IRQ_DISABLE = 0x0,
	SPI_IRQ_ENABLE = 0x1,
}ESPI_IRQControlState;

/******************************** SPI Register Bit fields START ********************************/
typedef enum ESPI_CR1_BitFieldsPositions
{
	CPHA = 0,
	CPOL = 1,
	MSTR = 2,
	BR = 3,
	SPE = 6,
	LSBFIRST = 7,
	SSI = 8,
	SSM = 9,
	RXONLY = 10,
	DFF = 11,
	CRCNEXT = 12,
	CRCEN = 13,
	BIDIOE = 14,
	BIDIMODE = 15,
}ESPI_CR1_BitFieldsPositions;

typedef enum ESPI_SR_BitFieldsPositions
{
	RXNE = 0,
	TXE = 1,
	CHSIDE = 2,
	UDR = 3,
	CRCERR = 4,
	MODF = 5,
	OVR = 6,
	BSY = 7,
	FRE = 8,
}ESPI_SR_BitFieldsPositions;

/******************************** SPI Register Bit fields END ********************************/

/*
 * Clock control
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, ESPI_ClockControlState clockControlState);


/*
 * Init and deinit SPI
 */
void SPI_Init(SPI_Handle_t* pSPIHandle);

void SPI_DeInit(SPI_RegDef_t* pSPIx);


/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t length);

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t length);


/*
 * IRQ config and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, ESPI_IRQControlState state);

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t* pHandle);




#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
