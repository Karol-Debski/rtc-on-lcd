/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 29, 2024
 *      Author: karol
 */
#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScalerValues [8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APBLowSpeed_PreScalerValues [4] = {2, 4, 8, 16};

static uint32_t RCC_GetPCLK1Value();
static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_WaitUntilCompleteStartConditionGeneration(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t slaveAddr, EI2C_OperationType operationType);
static void I2C_ResumeTransmisionAfterAddressSlaveMatch(I2C_RegDef_t* pI2Cx);
static void I2C_WaitUntilCompleateAddressPhase(I2C_RegDef_t* pI2Cx);
static void I2C_WaitUntilDataRegisterIsEmpty(I2C_RegDef_t* pI2Cx);
static void I2C_WaitUntilDataRegisterIsNotEmpty(I2C_RegDef_t* pI2Cx);
static void I2C_WaitUntilLastByteIsSent(I2C_RegDef_t* pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
static void I2C_DisableACK(I2C_RegDef_t* pI2Cx);


static uint32_t RCC_GetPCLK1Value()
{
	uint32_t systemClock=0;

	uint8_t clockSource = MULTI_BIT_READ(RCC->CFGR, 2, 2);

	if(clockSource == 0)
	{
		//HSI
		systemClock=16000000;
	}
	else if(clockSource == 1)
	{
		//HSE
		systemClock=8000000;
	}
	else if(clockSource == 2)
	{
		//PLL
		//todo systemClock =
	}

	uint32_t preScalerAHBValue = MULTI_BIT_READ(RCC->CFGR, 4, 4);
	uint32_t scalarAHB = 0;

	if(preScalerAHBValue < 8)
	{
		scalarAHB = 1;
	}
	else
	{
		scalarAHB = AHB_PreScalerValues[preScalerAHBValue-8];
	}

	uint32_t preScalerAPBLowSpeedValue = MULTI_BIT_READ(RCC->CFGR, 10, 3);
	uint32_t scalarAPBLowSpeed = 0;

	if(preScalerAPBLowSpeedValue < 8)
	{
		scalarAPBLowSpeed = 1;
	}
	else
	{
		scalarAPBLowSpeed = APBLowSpeed_PreScalerValues[preScalerAPBLowSpeedValue-4];
	}

	uint32_t PCLK1Value = (systemClock/scalarAHB)/(scalarAPBLowSpeed);


	return PCLK1Value;
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

}

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 1, 8);
}

static void I2C_WaitUntilCompleteStartConditionGeneration(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, 0) );
	//SCL line gets pulled to LOW now
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t slaveAddr, EI2C_OperationType operationType)
{
	if(operationType == I2C_Write)
	{
		pI2Cx->DR = slaveAddr << 1;
	}
	else if(operationType == I2C_Read)
	{
		pI2Cx->DR = (slaveAddr << 1) | 1;
	}
}

static void I2C_ResumeTransmisionAfterAddressSlaveMatch(I2C_RegDef_t* pI2Cx)
{
	//steps to clear ADDR flag in SR1
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_WaitUntilCompleateAddressPhase(I2C_RegDef_t* pI2Cx)
{
	//possible infinite loop when there is not match with sent address
	while( 0 == BIT_READ(pI2Cx->SR1, 1) );
}

static void I2C_WaitUntilDataRegisterIsEmpty(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, 7) );
}

static void I2C_WaitUntilDataRegisterIsNotEmpty(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, 6) );
}

static void I2C_WaitUntilLastByteIsSent(I2C_RegDef_t* pI2Cx)
{
	while( 0 == BIT_READ(pI2Cx->SR1, 7) );
	while( 0 == BIT_READ(pI2Cx->SR1, 2) );
}

static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 1, 9);
}

static void I2C_DisableACK(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 0, 10);
}

static void I2C_EnableACK(I2C_RegDef_t* pI2Cx)
{
	BIT_SET_VAL(pI2Cx->CR1, 1, 10);
}

void I2C_MasterSendData(I2C_RegDef_t* pI2Cx, uint8_t* pTxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop)
{
	I2C_GenerateStartCondition(pI2Cx);

	I2C_WaitUntilCompleteStartConditionGeneration(pI2Cx);

	I2C_ExecuteAddressPhase(pI2Cx, slaveAddr, I2C_Write);

	I2C_WaitUntilCompleateAddressPhase(pI2Cx);

	I2C_ResumeTransmisionAfterAddressSlaveMatch(pI2Cx);

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
}


void I2C_MasterReceiveData(I2C_RegDef_t* pI2Cx, uint8_t* pRxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop)
{
	I2C_GenerateStartCondition(pI2Cx);

	I2C_WaitUntilCompleteStartConditionGeneration(pI2Cx);

	I2C_ExecuteAddressPhase(pI2Cx, slaveAddr, I2C_Read);

	I2C_WaitUntilCompleateAddressPhase(pI2Cx);

	if(size == 1)
	{
		I2C_DisableACK(pI2Cx);
		I2C_ResumeTransmisionAfterAddressSlaveMatch(pI2Cx);
		I2C_WaitUntilDataRegisterIsNotEmpty(pI2Cx);

		*pRxBuffer = (uint8_t)(pI2Cx->DR);
	}
	else
	{
		I2C_EnableACK(pI2Cx);
		I2C_ResumeTransmisionAfterAddressSlaveMatch(pI2Cx);
		while(size > 0)
		{
			I2C_WaitUntilDataRegisterIsNotEmpty(pI2Cx);
			if(size == 2)
			{
				I2C_DisableACK(pI2Cx);
			}
			*pRxBuffer = (uint8_t)(pI2Cx->DR);
			pRxBuffer++;
			size--;
		}
	}

	if(endWithStop == ENABLE)
	{
		I2C_GenerateStopCondition(pI2Cx);
	}
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
