/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Aug 20, 2024
 *      Author: karol
 */
#include "stm32f407xx_spi_driver.h"

/************************** Helper functions definitions START **********************************/

static void SPI_Write8Bits(SPI_Handle_t* pSPIHandle);
static void SPI_Write16Bits(SPI_Handle_t* pSPIHandle);
static void SPI_Read8Bits(SPI_Handle_t* pSPIHandle);
static void SPI_Read16Bits(SPI_Handle_t* pSPIHandle);

/************************** Helper functions definitions END **********************************/


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

ESPI_Status SPI_SendDataIT(SPI_Handle_t* pSPIHandlex, uint8_t* pTxBuffer, uint32_t size)
{
	if(pSPIHandlex->state == SPI_BUSY_IN_TX)
	{
		return SPI_BUSY;
	}

	pSPIHandlex->pTxBuffer=pTxBuffer;
	pSPIHandlex->txLen=size;

	pSPIHandlex->state = SPI_BUSY_IN_TX;

	BIT_SET_VAL(pSPIHandlex->pSPIx->CR2, 1, TXEIE);

	return SPI_OK;
}

ESPI_Status SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandlex, uint8_t* pRxBuffer, uint32_t size)
{
	if(pSPIHandlex->state == SPI_BUSY_IN_RX)
	{
		return SPI_BUSY;
	}

	pSPIHandlex->pRxBuffer=pRxBuffer;
	pSPIHandlex->rxLen=size;

	pSPIHandlex->state = SPI_BUSY_IN_RX;

	BIT_SET_VAL(pSPIHandlex->pSPIx->CR2, 1, RXNEIE);

	return SPI_OK;
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, ESPI_IRQControlState state)
{
	if(state == SPI_IRQ_ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else if(state == SPI_IRQ_DISABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= 1 << IRQNumber;
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= 1 << IRQNumber % 32;
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= 1 << IRQNumber % 64;
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t nvicIprRegNum = IRQNumber / 4;
	uint8_t nvicIprRegByteNum = IRQNumber % 4;

	uint8_t shiftValue = nvicIprRegByteNum * 8 +( 8 - NO_PR_BITS_IMPLEMENTED);
	// clear previous priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] &= ~(0xF << shiftValue);
	// set new priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] |= (IRQPriority << shiftValue);
}

static void SPI_IRQWriteSendData(SPI_Handle_t* pSPIHandle)
{
	if(BIT_READ(pSPIHandle->pSPIx->CR1, DFF) == 0)
	{
		//8 BITS
		SPI_Write8Bits(pSPIHandle);
	}
	else if(BIT_READ(pSPIHandle->pSPIx->CR1, DFF) == 1)
	{
		//16 BITS
		if((pSPIHandle->txLen % 2) == 0)
		{
			SPI_Write16Bits(pSPIHandle);
		}
		else
		{
			SPI_Write8Bits(pSPIHandle);
		}
	}

	if(pSPIHandle->txLen == 0)
	{
		SPI_IRQCloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_COMPLETE);
	}
}

static void SPI_IRQReadReceiveData(SPI_Handle_t* pSPIHandle)
{
	if(BIT_READ(pSPIHandle->pSPIx->CR1, DFF) == 0)
	{
		//8 BITS
		SPI_Read8Bits(pSPIHandle);
	}
	else if(BIT_READ(pSPIHandle->pSPIx->CR1, DFF) == 1)
	{
		//16 BITS
		if((pSPIHandle->rxLen % 2) == 0)
		{
			SPI_Read16Bits(pSPIHandle);
		}
		else
		{
		SPI_Read8Bits(pSPIHandle);
		}
	}

	if(pSPIHandle->rxLen == 0)
	{
		SPI_IRQCloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_COMPLETE);
	}
}

static void SPI_IRQ_OVRErrorHandle(SPI_Handle_t* pSPIHandle)
{
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERROR);
}

void SPI_IRQHandling(SPI_Handle_t* pSPIHandle)
{
	// what trigger isr
	if((BIT_READ(pSPIHandle->pSPIx->CR2, RXNEIE) == 1) && (BIT_READ(pSPIHandle->pSPIx->SR, RXNE) == 1))
	{
		SPI_IRQReadReceiveData(pSPIHandle);
	}

	if((BIT_READ(pSPIHandle->pSPIx->CR2, TXEIE) == 1) && (BIT_READ(pSPIHandle->pSPIx->SR, TXE) == 1))
	{
		SPI_IRQWriteSendData(pSPIHandle);
	}

	if((BIT_READ(pSPIHandle->pSPIx->CR2, ERRIE) == 1) && (BIT_READ(pSPIHandle->pSPIx->SR, OVR) == 1))
	{
		SPI_IRQ_OVRErrorHandle(pSPIHandle);
	}

}


/************************** Helper functions declarations START **********************************/

static void SPI_Write8Bits(SPI_Handle_t* pSPIHandle)
{
	pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
	pSPIHandle->pTxBuffer = pSPIHandle->pTxBuffer + 1;
	pSPIHandle->txLen = pSPIHandle->txLen - 1;
}

static void SPI_Write16Bits(SPI_Handle_t* pSPIHandle)
{
	pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
	pSPIHandle->pTxBuffer = pSPIHandle->pTxBuffer + 2;
	pSPIHandle->txLen = pSPIHandle->txLen - 2;
}

static void SPI_Read8Bits(SPI_Handle_t* pSPIHandle)
{
	*(pSPIHandle->pRxBuffer)=(uint8_t)(pSPIHandle->pSPIx->DR);
	pSPIHandle->pRxBuffer = pSPIHandle->pRxBuffer + 1;
	pSPIHandle->rxLen = pSPIHandle->rxLen - 1;
}

static void SPI_Read16Bits(SPI_Handle_t* pSPIHandle)
{
	*(pSPIHandle->pRxBuffer)=(uint16_t)(pSPIHandle->pSPIx->DR);
	pSPIHandle->pRxBuffer = pSPIHandle->pRxBuffer + 2;
	pSPIHandle->rxLen = pSPIHandle->rxLen - 2;
}

void SPI_IRQCloseTransmisson(SPI_Handle_t* pSPIHandle)
{
	BIT_SET_VAL(pSPIHandle->pSPIx->CR2, 0, TXEIE);
	pSPIHandle->pTxBuffer=NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->state = SPI_READY;
}

void SPI_IRQCloseReception(SPI_Handle_t* pSPIHandle)
{
	BIT_SET_VAL(pSPIHandle->pSPIx->CR2, 0, RXNEIE);
	pSPIHandle->pRxBuffer=NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->state = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_Handle_t* pSPIHandle)
{
	pSPIHandle->pSPIx->DR;
	pSPIHandle->pSPIx->SR;
}

/************************** Helper functions declarations END **********************************/


/************************** Applications callbacks START **********************************/

__weak void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandlex, ESPI_Event spiEvent)
{
	// application should override this function
}

/************************** Applications callbacks END **********************************/
