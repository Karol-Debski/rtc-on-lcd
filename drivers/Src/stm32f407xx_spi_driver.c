/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 20, 2024
 *      Author: karol
 */
#include "stm32f407xx_spi_driver.h"




void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, ESPI_ClockControlState clockControlState)
{
	if(clockControlState == SPI_CLOCK_ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERI_CLOCK_EN();
		}
		else if (pSPIx == SPI2_I2S2)
		{
			SPI2_I2S2_PERI_CLOCK_EN();
		}
		else if (pSPIx == SPI3_I2S3)
		{
			SPI3_I2S3_PERI_CLOCK_EN();
		}
	}
	else if(clockControlState == SPI_CLOCK_DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERI_CLOCK_DIS();
		}
		else if (pSPIx == SPI2_I2S2)
		{
			SPI2_I2S2_PERI_CLOCK_DIS();
		}
		else if (pSPIx == SPI3_I2S3)
		{
			SPI3_I2S3_PERI_CLOCK_DIS();
		}
	}
}


void SPI_Init(SPI_Handle_t* pSPIHandle)
{

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		BIT_SET_VAL(pSPIHandle->pSPIx.CR1, 0, BIDIMODE);
		BIT_SET_VAL(pSPIHandle->pSPIx.CR1, 0, RXONLY);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		BIT_SET_VAL(pSPIHandle->pSPIx.CR1, 1, BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		BIT_SET_VAL(pSPIHandle->pSPIx.CR1, 0, BIDIMODE);
		BIT_SET_VAL(pSPIHandle->pSPIx.CR1, 1, RXONLY);
	}

	BIT_SET_VAL(pSPIHandle->pSPIx.CR1, pSPIHandle->SPIConfig.SPI_DeviceMode, MSTR);
	MULTI_BIT_SET_VAL(pSPIHandle->pSPIx.CR1, pSPIHandle->SPIConfig.SPI_SclkSpeed, BR, 3);
	BIT_SET_VAL(pSPIHandle->pSPIx.CR1, pSPIHandle->SPIConfig.SPI_DataFrameFormat, DFF);
	BIT_SET_VAL(pSPIHandle->pSPIx.CR1, pSPIHandle->SPIConfig.SPI_ClockPolarity, CPOL);
	BIT_SET_VAL(pSPIHandle->pSPIx.CR1, pSPIHandle->SPIConfig.SPI_ClockPhase, CPHA);
	BIT_SET_VAL(pSPIHandle->pSPIx.CR1, pSPIHandle->SPIConfig.SPI_SoftwareSlaveManagment, SSM);

}

void SPI_DeInit(SPI_RegDef_t* pSPIx)
{
	//todo
}

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t length)
{

}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t length);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, ESPI_IRQControlState state);

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t* pHandle);
