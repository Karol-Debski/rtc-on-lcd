/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 29, 2024
 *      Author: karol
 */
#include "stm32f407xx_i2c_driver.h"


static I2C_Handle_t I2C1_Handle = {.pI2Cx=I2C1};
static I2C_Handle_t I2C2_Handle = {.pI2Cx=I2C2};
static I2C_Handle_t I2C3_Handle = {.pI2Cx=I2C3};





static void I2C_DisableACK(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 0, I2C_CR1_ACK);
}

static void I2C_EnableACK(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 1, I2C_CR1_ACK);
}

static I2C_Handle_t* getI2C_Handle(I2C_RegDef_t* pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		return &I2C1_Handle;
	}
	else if(pI2Cx == I2C2)
	{
		return &I2C2_Handle;
	}
	else if(pI2Cx == I2C3)
	{
		return &I2C3_Handle;
	}
	else
	{
		return NULL;
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 1, I2C_CR1_START);
}

static void I2C_WaitUntilCompleteStartConditionGeneration(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_SB) );
	//SCL line gets pulled to LOW now
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t slaveAddr, EI2C_OperationType operationType)
{
	if(operationType == I2C_Operation_Write)
	{
		pI2Cx->DR = slaveAddr << 1;
	}
	else if(operationType == I2C_Operation_Read)
	{
		pI2Cx->DR = (slaveAddr << 1) | 1;
	}
}

static void I2C_ResumeTransmisionAfterAddressMatch(I2C_RegDef_t* pI2Cx)
{
	//steps to clear ADDR flag in SR1
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_WaitUntilCompleateAddressPhase(I2C_RegDef_t* pI2Cx)
{
	//possible infinite loop when there is not match with sent address
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_ADDR) );
}

static void I2C_WaitUntilDataRegisterIsEmpty(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_TxE) );
}

static void I2C_WaitUntilDataRegisterIsNotEmpty(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_RxNE) );
}

static void I2C_WaitUntilLastByteIsSent(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_TxE) );
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_BTF) );
}

static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 1, I2C_CR1_STOP);
}

void I2C_WaitUntilMasterSendNACK(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_AF) );
}

void I2C_ClearACKFailure(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->SR1, 0, I2C_SR1_AF);
}

void I2C_WaitUntilSTOPDetected(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, I2C_SR1_STOPF) );
}

void I2C_ClearSTOPFailure(I2C_RegDef_t* pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	(void)dummyRead;
	// dummy write
	pI2Cx->CR1 |= 0x0;
}

static void I2C_EnableErrorEventBufferIT(I2C_RegDef_t* pI2Cx)
{
	//error
	BIT_SET_VAL(pI2Cx->CR2, 1, I2C_CR2_ITERREN);
	//event
	BIT_SET_VAL(pI2Cx->CR2, 1, I2C_CR2_ITEVTEN);
	//buffer
	BIT_SET_VAL(pI2Cx->CR2, 1, I2C_CR2_ITBUFEN);
}

static void I2C_DisableErrorEventBufferIT(I2C_RegDef_t* pI2Cx)
{
	//error
	BIT_SET_VAL(pI2Cx->CR2, 0, I2C_CR2_ITERREN);
	//event
	BIT_SET_VAL(pI2Cx->CR2, 0, I2C_CR2_ITEVTEN);
	//buffer
	BIT_SET_VAL(pI2Cx->CR2, 0, I2C_CR2_ITBUFEN);
}


void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	I2C_PeriClockControl(pI2CHandle->pI2Cx, I2C_ClockPeriEnable);


	uint32_t speedMode = pI2CHandle->I2C_Config.I2C_SCLSpeed;

	//t_rise calculation
	if(speedMode == I2C_SCLSpeed_StandardMode)
	{
		uint8_t timeRiseRegValue;
		timeRiseRegValue = RCC_GetPCLK1Value()/(1000000U)+1;
		MULTI_BIT_SET_VAL(pI2CHandle->pI2Cx->TRISE, timeRiseRegValue, 0, 6);
	}
	else if(speedMode == I2C_SCLSpeed_FastMode2K || speedMode == I2C_SCLSpeed_FastMode4K)
	{
		uint8_t timeRiseRegValue;
		timeRiseRegValue = (RCC_GetPCLK1Value()*300)/(1000000000U)+1;
		MULTI_BIT_SET_VAL(pI2CHandle->pI2Cx->TRISE, timeRiseRegValue, 0, 6);
	}


	I2C_PeripheralControl(pI2CHandle->pI2Cx, I2C_Enable);


	I2C_EnableACK(pI2CHandle->pI2Cx);


	uint32_t pclk1MHzValue = RCC_GetPCLK1Value()/1000000U;
	MULTI_BIT_SET_VAL(pI2CHandle->pI2Cx->CR2, pclk1MHzValue, 0, 6);

	uint32_t ownSlaveAdrress = pI2CHandle->I2C_Config.I2C_DeviceAddress;
	MULTI_BIT_SET_VAL(pI2CHandle->pI2Cx->OAR1, ownSlaveAdrress, 1, 7);


	//keep 14 bit of OAR1 as 1
	BIT_SET_VAL(pI2CHandle->pI2Cx->OAR1, 1, 14);


	if(speedMode == I2C_SCLSpeed_StandardMode)
	{
		BIT_SET_VAL(pI2CHandle->pI2Cx->CCR, 0, 15);
		uint32_t CCRValue = (RCC_GetPCLK1Value())/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		MULTI_BIT_SET_VAL(pI2CHandle->pI2Cx->CCR, CCRValue, 0, 12);
	}
	else if(speedMode == I2C_SCLSpeed_FastMode2K || speedMode == I2C_SCLSpeed_FastMode4K)
	{
		BIT_SET_VAL(pI2CHandle->pI2Cx->CCR, 1, 15);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FMDuty_2)
		{
			BIT_SET_VAL(pI2CHandle->pI2Cx->CCR, 0, 14);
			uint32_t CCRValue = (RCC_GetPCLK1Value())/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed);
			MULTI_BIT_SET_VAL(pI2CHandle->pI2Cx->CCR, CCRValue, 0, 12);
		}
		else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FMDuty_16_9)
		{
			BIT_SET_VAL(pI2CHandle->pI2Cx->CCR, 1, 14);
			uint32_t CCRValue = (RCC_GetPCLK1Value())/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed);
			MULTI_BIT_SET_VAL(pI2CHandle->pI2Cx->CCR, CCRValue, 0, 12);
		}
	}

	I2C_Handle_t* pI2Cx_HandleInternal = getI2C_Handle(pI2CHandle->pI2Cx);

	pI2Cx_HandleInternal->I2C_Config.I2C_SCLSpeed = pI2CHandle->I2C_Config.I2C_SCLSpeed;
	pI2Cx_HandleInternal->I2C_Config.I2C_FMDutyCycle = pI2CHandle->I2C_Config.I2C_FMDutyCycle;
	pI2Cx_HandleInternal->I2C_Config.I2C_DeviceAddress = pI2CHandle->I2C_Config.I2C_DeviceAddress;

}


uint8_t I2C_MasterSendData(I2C_RegDef_t* pI2Cx, uint8_t* pTxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop)
{
	I2C_Handle_t* pI2Cx_Handle = getI2C_Handle(pI2Cx);

	if(pI2Cx_Handle->TxRxState != I2C_Ready)
	{
		return 1;
	}

	pI2Cx_Handle->TxRxState = I2C_BusyInTx;

	I2C_GenerateStartCondition(pI2Cx);

	I2C_WaitUntilCompleteStartConditionGeneration(pI2Cx);

	I2C_ExecuteAddressPhase(pI2Cx, slaveAddr, I2C_Operation_Write);

	I2C_WaitUntilCompleateAddressPhase(pI2Cx);

	I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);

	while(size > 0)
	{
		I2C_WaitUntilDataRegisterIsEmpty(pI2Cx);
		pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		size--;
	}

	I2C_WaitUntilLastByteIsSent(pI2Cx);

	if(endWithStop == ENABLE)
	{
		I2C_GenerateStopCondition(pI2Cx);
	}

	pI2Cx_Handle->TxRxState = I2C_Ready;

	return 0;
}


uint8_t I2C_MasterReceiveData(I2C_RegDef_t* pI2Cx, uint8_t* pRxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop)
{
	I2C_Handle_t* pI2Cx_Handle = getI2C_Handle(pI2Cx);

	if(pI2Cx_Handle->TxRxState != I2C_Ready)
	{
		return 1;
	}

	pI2Cx_Handle->TxRxState = I2C_BusyInRx;

	I2C_GenerateStartCondition(pI2Cx);

	I2C_WaitUntilCompleteStartConditionGeneration(pI2Cx);

	I2C_ExecuteAddressPhase(pI2Cx, slaveAddr, I2C_Operation_Read);

	I2C_WaitUntilCompleateAddressPhase(pI2Cx);

	if(size == 1)
	{
		I2C_DisableACK(pI2Cx);
		I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);
		// getting data...
		I2C_WaitUntilDataRegisterIsNotEmpty(pI2Cx);

		*pRxBuffer = (uint8_t)(pI2Cx->DR);
	}
	else
	{
		I2C_EnableACK(pI2Cx);
		I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);
		// getting data...
		while(size > 0)
		{
			I2C_WaitUntilDataRegisterIsNotEmpty(pI2Cx);
			if(size == 2)
			{
				I2C_DisableACK(pI2Cx);
			}
			*pRxBuffer = (uint8_t)(pI2Cx->DR);
			// getting data...
			pRxBuffer++;
			size--;
		}
	}

	I2C_EnableACK(pI2Cx);

	if(endWithStop == ENABLE)
	{
		I2C_GenerateStopCondition(pI2Cx);
	}

	pI2Cx_Handle->TxRxState = I2C_Ready;

	return 0;
}


uint8_t I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t* pTxBuffer, uint8_t size)
{
	I2C_Handle_t* pI2Cx_Handle = getI2C_Handle(pI2Cx);

	if(pI2Cx_Handle->TxRxState != I2C_Ready)
	{
		return 1;
	}

	pI2Cx_Handle->TxRxState = I2C_BusyInTx;

	I2C_WaitUntilCompleateAddressPhase(pI2Cx);
	I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);

	while(size > 0)
	{
		I2C_WaitUntilDataRegisterIsEmpty(pI2Cx);
		pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		size--;
	}


	I2C_WaitUntilMasterSendNACK(pI2Cx);
	I2C_ClearACKFailure(pI2Cx);


	pI2Cx_Handle->TxRxState = I2C_Ready;

	return 0;
}


uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx, uint8_t* pRxBuffer, uint8_t size, uint8_t expectEndWithStop)
{
	I2C_Handle_t* I2Cx_Handle = getI2C_Handle(pI2Cx);

	if(I2Cx_Handle->TxRxState != I2C_Ready)
	{
		return 1;
	}

	I2Cx_Handle->TxRxState = I2C_BusyInRx;

	I2C_WaitUntilCompleateAddressPhase(pI2Cx);
	I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);

	while(size > 0)
	{
		I2C_WaitUntilDataRegisterIsNotEmpty(pI2Cx);
		*pRxBuffer = (uint8_t)(pI2Cx->DR);
		// getting data...
		pRxBuffer++;
		size--;
	}

	if(expectEndWithStop == ENABLE)
	{
		I2C_WaitUntilSTOPDetected(pI2Cx);
		I2C_ClearSTOPFailure(pI2Cx);
	}

	I2Cx_Handle->TxRxState = I2C_Ready;

	return 0;
}


uint8_t I2C_MasterSendDataIT(I2C_RegDef_t* pI2Cx, uint8_t* pTxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop)
{
	I2C_Handle_t* pI2Cx_Handle = getI2C_Handle(pI2Cx);

	if(pI2Cx_Handle->TxRxState != I2C_Ready)
	{
		return 1;
	}


	pI2Cx_Handle->pTxBuffer = pTxBuffer;
	pI2Cx_Handle->TxSize = size;
	pI2Cx_Handle->slaveAddress = slaveAddr;
	pI2Cx_Handle->endWithStop = endWithStop;


	pI2Cx_Handle->TxRxState = I2C_BusyInTx;

	I2C_GenerateStartCondition(pI2Cx);

	I2C_EnableErrorEventBufferIT(pI2Cx);

	return 0;
}


uint8_t I2C_MasterReceiveDataIT(I2C_RegDef_t* pI2Cx, uint8_t* pRxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop)
{
	I2C_Handle_t* pI2Cx_Handle = getI2C_Handle(pI2Cx);

	if(pI2Cx_Handle->TxRxState != I2C_Ready)
	{
		return 1;
	}


	pI2Cx_Handle->pRxBuffer = pRxBuffer;
	pI2Cx_Handle->RxSize = size;
	pI2Cx_Handle->slaveAddress = slaveAddr;
	pI2Cx_Handle->endWithStop = endWithStop;




	pI2Cx_Handle->TxRxState = I2C_BusyInRx;

	I2C_GenerateStartCondition(pI2Cx);

	I2C_EnableErrorEventBufferIT(pI2Cx);

	return 0;
}


EI2C_IT_Flag I2C_GetActiveFlag(I2C_RegDef_t* pI2Cx)
{
	if(BIT_READ(pI2Cx->SR1, I2C_SR1_SB) == 1)
	{
		return I2C_IT_EVENT_Master_StartBitSend;
	}
	else if(BIT_READ(pI2Cx->SR1, I2C_SR1_ADDR) == 1)
	{
		return I2C_IT_EVENT_Master_AddressSent_Slave_AddressMatched;
	}
	else if(BIT_READ(pI2Cx->SR1, I2C_SR1_ADD10) == 1)
	{
		return I2C_IT_EVENT_Master_10BitHeaderSent;
	}
	else if(BIT_READ(pI2Cx->SR1, I2C_SR1_STOPF) == 1)
	{
		return I2C_IT_EVENT_Slave_StopReceived;
	}
	else if(BIT_READ(pI2Cx->SR1, I2C_SR1_BTF) == 1)
	{
		return I2C_IT_EVENT_DataByteTransferFinished;
	}
	else if(BIT_READ(pI2Cx->SR1, I2C_SR1_RxNE) == 1)
	{
		return I2C_IT_EVENT_ReceiveBufferNotEmpty;
	}
	else if(BIT_READ(pI2Cx->SR1, I2C_SR1_TxE) == 1)
	{
		return I2C_IT_EVENT_TransmitBufferEmpty;
	}
	//todo add error flags

	return I2C_IT_Unknown;
}


EI2C_OperationType I2C_DetermineOperationType(I2C_Handle_t* pI2CHandle)
{
	if(pI2CHandle->TxRxState == I2C_BusyInTx)
	{
		return I2C_Operation_Write;
	}
	else if(pI2CHandle->TxRxState == I2C_BusyInRx)
	{
		return I2C_Operation_Read;
	}

	return I2C_Operation_Unknown;
}


void I2C_IT_ExecuteTransmitDataPhase(I2C_RegDef_t* pI2Cx)
{
	I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);
}


void I2C_IT_ExecuteReceiveDataPhase(I2C_RegDef_t* pI2Cx, uint32_t size)
{
	if(size == 1)
	{
		I2C_DisableACK(pI2Cx);
		I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);
		// getting data...
	}
	else
	{
		I2C_EnableACK(pI2Cx);
		I2C_ResumeTransmisionAfterAddressMatch(pI2Cx);
		// getting data...
	}
}


void I2C_ProcessFlag(I2C_Handle_t* pI2CHandle, EI2C_IT_Flag activeITFlag)
{
	switch(activeITFlag)
	{
		case I2C_IT_EVENT_Master_StartBitSend:
		{
			EI2C_OperationType operationType = I2C_DetermineOperationType(pI2CHandle);
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->slaveAddress, operationType);
			break;
		}
		case I2C_IT_EVENT_Master_AddressSent_Slave_AddressMatched:
		{
			//Master
			if(pI2CHandle->TxRxState == I2C_BusyInTx)
			{
				I2C_IT_ExecuteTransmitDataPhase(pI2CHandle->pI2Cx);
			}
			else if(pI2CHandle->TxRxState == I2C_BusyInRx)
			{
				I2C_IT_ExecuteReceiveDataPhase(pI2CHandle->pI2Cx, pI2CHandle->RxSize);
			}
			//todo:Slave
			break;
		}
		case I2C_IT_EVENT_Master_10BitHeaderSent:
		{

			break;
		}
		case I2C_IT_EVENT_Slave_StopReceived:
		{

			break;
		}
		case I2C_IT_EVENT_DataByteTransferFinished:
		{
			if(pI2CHandle->TxRxState == I2C_BusyInTx && pI2CHandle->TxSize == 0)
			{
				pI2CHandle->TxRxState = I2C_Ready;

				I2C_EnableACK(pI2CHandle->pI2Cx);

				I2C_DisableErrorEventBufferIT(pI2CHandle->pI2Cx);

				if(pI2CHandle->endWithStop == 1)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			break;
		}
		case I2C_IT_EVENT_ReceiveBufferNotEmpty:
		{
			if(pI2CHandle->RxSize > 0)
			{
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = (uint8_t)(pI2CHandle->pI2Cx->DR);
					pI2CHandle->RxSize--;
					pI2CHandle->TxRxState = I2C_Ready;

					I2C_EnableACK(pI2CHandle->pI2Cx);

					I2C_DisableErrorEventBufferIT(pI2CHandle->pI2Cx);

					if(pI2CHandle->endWithStop == 1)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
				}
				else if(pI2CHandle->RxSize == 2)
				{
					I2C_DisableACK(pI2CHandle->pI2Cx);
					*pI2CHandle->pRxBuffer = (uint8_t)(pI2CHandle->pI2Cx->DR);
					// getting data
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxSize--;
				}
				else
				{
					*pI2CHandle->pRxBuffer = (uint8_t)(pI2CHandle->pI2Cx->DR);
					// getting data
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxSize--;
				}

			}
			break;
		}
		case I2C_IT_EVENT_TransmitBufferEmpty:
		{
			if(pI2CHandle->TxSize > 0)
			{
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
				pI2CHandle->pTxBuffer++;
				pI2CHandle->TxSize--;
			}
			break;
		}
        default:
        {
            break;
        }
	}
}


void I2C_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	EI2C_IT_Flag activeITFlag = I2C_GetActiveFlag(pI2CHandle->pI2Cx);

	I2C_ProcessFlag(pI2CHandle, activeITFlag);
}


void I2C1_EV_IRQHandler()
{
	I2C_IRQHandling(&I2C1_Handle);
}


void I2C1_ER_IRQHandler()
{
	I2C_IRQHandling(&I2C1_Handle);
}

void I2C2_EV_IRQHandler()
{
	I2C_IRQHandling(&I2C2_Handle);
}


void I2C2_ER_IRQHandler()
{
	I2C_IRQHandling(&I2C2_Handle);
}

void I2C3_EV_IRQHandler()
{
	I2C_IRQHandling(&I2C3_Handle);
}


void I2C3_ER_IRQHandler()
{
	I2C_IRQHandling(&I2C3_Handle);
}


void I2C_DeInit(I2C_RegDef_t* pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REGISTERS_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REGISTERS_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REGISTERS_RESET();
	}
}


void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, EI2C_ClockPeriControlState clockControlState)
{
	if(clockControlState == I2C_ClockPeriEnable)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PERI_CLOCK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PERI_CLOCK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PERI_CLOCK_EN();
		}
	}
	else if(clockControlState == I2C_ClockPeriDisable)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PERI_CLOCK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PERI_CLOCK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PERI_CLOCK_EN();
		}
	}
}

void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, EI2C_PeriControlState controlState)
{
	BIT_SET_VAL(pI2Cx->CR1, controlState, I2C_CR1_PE);
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, EI2C_IRQControlState state)
{
	if(state == I2C_IRQ_Enable)
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
	else if(state == I2C_IRQ_Disable)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t nvicIprRegNum = IRQNumber / 4;
	uint8_t nvicIprRegByteNum = IRQNumber % 4;

	uint8_t shiftValue = nvicIprRegByteNum * 8 +( 8 - NO_PR_BITS_IMPLEMENTED);
	// clear previous priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] &= ~(0xF << shiftValue);
	// set new priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] |= (IRQPriority << shiftValue);
}
