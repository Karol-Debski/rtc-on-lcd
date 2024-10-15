/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 29, 2023
 *      Author: karol
 */

#include "stm32407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - enable clock or disable
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, EGPIO_ClockControlState clockControlState)
{
	if(clockControlState == GPIO_CLOCK_ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLOCK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PERI_CLOCK_EN();
		}
	}
	else if(clockControlState == GPIO_CLOCK_DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PERI_CLOCK_DIS();
		}
	}

}


void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,GPIO_CLOCK_ENABLE);

	uint32_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint32_t pinMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
	uint32_t pinOutputSpeed = pGPIOHandle->GPIO_PinConfig.GPIO_PinOutputSpeed;
	uint32_t pinOutputType = pGPIOHandle->GPIO_PinConfig.GPIO_PinOutputType;
	uint32_t pinPullUpPullDownMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinPullUpPullDownMode;
	uint32_t pinAltFunMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/*Pin mode*/
		pGPIOHandle->pGPIOx->MODER = (pGPIOHandle->pGPIOx->MODER & ~(pinMode << pinNumber*2)) | (pinMode<< pinNumber*2);
	}
	else if( (pinMode == GPIO_MODE_IT_FT) || (pinMode == GPIO_MODE_IT_RT) || (pinMode == GPIO_MODE_IT_RFT) )
	{
		if(pinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR &= ~(1 << pinNumber);
		}
		else if(pinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pinNumber);
			EXTI->FTSR &= ~(1 << pinNumber);
		}
		else if(pinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);
		}

		uint8_t extiControllRegNum = pinNumber / 4;
		uint8_t extiNumInControllReg = pinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_PORT_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PERI_CLOCK_EN();

		SYSCFG->EXTICR[extiControllRegNum] = (SYSCFG->EXTICR[extiControllRegNum] & ~(portCode << extiNumInControllReg*4)) | (portCode << extiNumInControllReg*4);

		/* enable delivery interrupt */
		EXTI->IMR |= (1 << pinNumber);
	}

	/*Pin output speed*/
	pGPIOHandle->pGPIOx->OSPEEDR = (pGPIOHandle->pGPIOx->OSPEEDR & ~(pinOutputSpeed << pinNumber*2)) | (pinOutputSpeed<< pinNumber*2);
	/*Pin output type*/
	pGPIOHandle->pGPIOx->OTYPER = (pGPIOHandle->pGPIOx->OTYPER & ~(pinOutputType << pinNumber)) | (pinOutputType<< pinNumber);
	/*Pin pull up pull down mode*/
	pGPIOHandle->pGPIOx->PUPDR = (pGPIOHandle->pGPIOx->PUPDR & ~(pinPullUpPullDownMode << pinNumber*2)) | (pinPullUpPullDownMode<< pinNumber*2);

	if(pinMode == GPIO_MODE_ALT_FN)
	{
		/*Pin mode*/
		pGPIOHandle->pGPIOx->MODER = (pGPIOHandle->pGPIOx->MODER & ~(pinMode << pinNumber*2)) | (pinMode<< pinNumber*2);

		uint8_t positionOfAFRegister = pinNumber / GPIO_PIN_NUM_8;
		uint8_t pinNumberInAFRegister = pinNumber % GPIO_PIN_NUM_8;

		MULTI_BIT_SET_VAL(pGPIOHandle->pGPIOx->AFR[positionOfAFRegister], pinAltFunMode, pinNumberInAFRegister*4, 4);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initialization gpio peripheral
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REGISTERS_RESET();
	}

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function return reading state from single gpio input type
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - pin number
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{
	uint8_t readValue;

	readValue=(uint8_t)((pGPIOx->IDR >> pinNumber) & (0x1));

	return readValue;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function return reading state from whole gpio port
 *
 * @param[in]         - base address of the gpio peripheral
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t readValue;

	readValue=(uint16_t)(pGPIOx->IDR);

	return readValue;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function sets the state of a gpio output pin
 *
 * @param[in]         - base address of a gpio peripheral
 * @param[in]         - pin number
 * @param[in]         - state low or high of a gpio pin
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, EGPIO_PinNumber pinNumber, EGPIO_PinState pinState)
{
	pGPIOx->ODR = (pGPIOx->ODR & ~(0x1 << pinNumber)) | (pinState << pinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function sets the state of a gpio port output
 *
 * @param[in]         - base address of a gpio peripheral
 * @param[in]         - state of a port to be written in register
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t portState)
{
	pGPIOx->ODR = (pGPIOx->ODR & ~portState) | (portState);
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles state of an output pin
 *
 * @param[in]         - base address of a gpio peripheral
 * @param[in]         - pin number
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (0x1 << pinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - This function enable or disable handling interrupt by NVIC for a specific irq number
 *
 * @param[in]         - irq number
 * @param[in]         - enable or disable state
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, GPIO_IRQControlState state)
{
	if(state == GPIO_IRQ_ENABLE)
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
	else if(state == GPIO_IRQ_DISABLE)
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

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - This function set priority for irq number
 *
 * @param[in]         - irq number
 * @param[in]         - priority value from 0 to 15
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t nvicIprRegNum = IRQNumber / 4;
	uint8_t nvicIprRegByteNum = IRQNumber % 4;

	uint8_t shiftValue = nvicIprRegByteNum * 8 +( 8 - NO_PR_BITS_IMPLEMENTED);
	// clear previous priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] &= ~(0xF << shiftValue);
	// set new priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] |= (IRQPriority << shiftValue);

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function disable pending state of interrupt associated with a pin
 *
 * @param[in]         - pin number
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(EGPIO_PinNumber pinNumber)
{
	if(EXTI->PR & (1 << pinNumber))
	{
		EXTI->PR |= (1 << pinNumber);
	}
}




