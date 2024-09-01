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
		else if (pSPIx == SPI2)
		{
			SPI2_I2S2_PERI_CLOCK_EN();
		}
		else if (pSPIx == SPI3)
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
		else if (pSPIx == SPI2)
		{
			SPI2_I2S2_PERI_CLOCK_DIS();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_I2S3_PERI_CLOCK_DIS();
		}
	}
}


void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx, SPI_CLOCK_ENABLE);

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		BIT_SET_VAL(pSPIHandle->pSPIx->CR1, 0, BIDIMODE);
		BIT_SET_VAL(pSPIHandle->pSPIx->CR1, 0, RXONLY);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		BIT_SET_VAL(pSPIHandle->pSPIx->CR1, 1, BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		BIT_SET_VAL(pSPIHandle->pSPIx->CR1, 0, BIDIMODE);
		BIT_SET_VAL(pSPIHandle->pSPIx->CR1, 1, RXONLY);
	}

	BIT_SET_VAL(pSPIHandle->pSPIx->CR1, pSPIHandle->SPIConfig.SPI_DeviceMode, MSTR);
	MULTI_BIT_SET_VAL(pSPIHandle->pSPIx->CR1, pSPIHandle->SPIConfig.SPI_SclkSpeed, BR, 3);
	BIT_SET_VAL(pSPIHandle->pSPIx->CR1, pSPIHandle->SPIConfig.SPI_DataFrameFormat, DFF);
	BIT_SET_VAL(pSPIHandle->pSPIx->CR1, pSPIHandle->SPIConfig.SPI_ClockPolarity, CPOL);
	BIT_SET_VAL(pSPIHandle->pSPIx->CR1, pSPIHandle->SPIConfig.SPI_ClockPhase, CPHA);
	BIT_SET_VAL(pSPIHandle->pSPIx->CR1, pSPIHandle->SPIConfig.SPI_SoftwareSlaveManagment, SSM);

}

void SPI_DeInit(SPI_RegDef_t* pSPIx)
{
	//todo
}

void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, ESPI_ControlState controlState)
{
	if(controlState == SPI_DISABLE)
	{
		while(BIT_READ(pSPIx->SR, BSY) == 1);
	}
	BIT_SET_VAL(pSPIx->CR1, controlState, SPE);
}

void SPI_SSIConfig(SPI_RegDef_t* pSPIx, ESPI_SSI_ControlState controlState)
{
	BIT_SET_VAL(pSPIx->CR1, controlState, SSI);
}

void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, ESPI_SSOE_ControlState controlState)
{
	BIT_SET_VAL(pSPIx->CR2, controlState, SSOE);
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - enable clock or disable
 *
 * @return            -  none
 *
 * @Note              -  This is blocking function

 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t size)
{
	while(size > 0)
	{
		while(BIT_READ(pSPIx->SR, TXE) == 0);

		if(BIT_READ(pSPIx->CR1, DFF) == 0)
		{
			//8 BITS
			pSPIx->DR = *pTxBuffer;
			pTxBuffer = pTxBuffer + 1;
			size = size - 1;
		}
		else if(BIT_READ(pSPIx->CR1, DFF) == 1)
		{
			//16 BITS
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			pTxBuffer = pTxBuffer + 2;
			size = size - 2;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t size)
{
	while(size > 0)
	{
		while(BIT_READ(pSPIx->SR, RXNE) == 0);

		if(BIT_READ(pSPIx->CR1, DFF) == 0)
		{
			//8 BITS
			*pRxBuffer=(uint8_t)(pSPIx->DR);
			pRxBuffer = pRxBuffer + 1;
			size = size - 1;
		}
		else if(BIT_READ(pSPIx->CR1, DFF) == 1)
		{
			//16 BITS
			*pRxBuffer=(uint16_t)(pSPIx->DR);
			pRxBuffer = pRxBuffer + 2;
			size = size - 2;
		}
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, ESPI_IRQControlState state);

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t* pHandle);
