/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Oct 21, 2024
 *      Author: karol
 */

#include "stm32f407xx_rcc_driver.h"

static uint16_t AHB_PreScalerValues [8] = {2, 4, 8, 16, 64, 128, 256, 512};
static uint16_t APBLowSpeedHighSpeed_PreScalerValues [4] = {2, 4, 8, 16};


uint32_t RCC_GetPCLK1Value()
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
		scalarAPBLowSpeed = APBLowSpeedHighSpeed_PreScalerValues[preScalerAPBLowSpeedValue-4];
	}

	uint32_t PCLK1Value = (systemClock/scalarAHB)/(scalarAPBLowSpeed);


	return PCLK1Value;
}


uint32_t RCC_GetPCLK2Value()
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

	uint32_t preScalerAPBHighSpeedValue = MULTI_BIT_READ(RCC->CFGR, 13, 3);
	uint32_t scalarAPBHighSpeed = 0;

	if(preScalerAPBHighSpeedValue < 8)
	{
		scalarAPBHighSpeed = 1;
	}
	else
	{
		scalarAPBHighSpeed = APBLowSpeedHighSpeed_PreScalerValues[preScalerAPBHighSpeedValue-4];
	}

	uint32_t PCLK2Value = (systemClock/scalarAHB)/(scalarAPBHighSpeed);


	return PCLK2Value;
}
