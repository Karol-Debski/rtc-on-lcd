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
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
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
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PERI_CLOCK_EN();
		}
		else if (pGPIOx == GPIOK)
		{
			GPIOK_PERI_CLOCK_EN();
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
		else if (pGPIOx == GPIOJ)
		{
			GPIOJ_PERI_CLOCK_DIS();
		}
		else if (pGPIOx == GPIOK)
		{
			GPIOK_PERI_CLOCK_DIS();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint32_t pinMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
	uint32_t pinOutputSpeed = pGPIOHandle->GPIO_PinConfig.GPIO_PinOutputSpeed;
	uint32_t pinOutputType = pGPIOHandle->GPIO_PinConfig.GPIO_PinOutputType;
	uint32_t pinPullUpPullDownMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinPullUpPullDownMode;


	/*Pin mode*/
	pGPIOHandle->pGPIOx->MODER = (pGPIOHandle->pGPIOx->MODER & ~(pinMode << pinNumber*2)) | (pinMode<< pinNumber*2);
	/*Pin output speed*/
	pGPIOHandle->pGPIOx->OSPEEDR = (pGPIOHandle->pGPIOx->OSPEEDR & ~(pinOutputSpeed << pinNumber*2)) | (pinOutputSpeed<< pinNumber*2);
	/*Pin output type*/
	pGPIOHandle->pGPIOx->OTYPER = (pGPIOHandle->pGPIOx->OSPEEDR & ~(pinOutputType << pinNumber)) | (pinOutputType<< pinNumber);
	/*Pin pull up pull down mode*/
	pGPIOHandle->pGPIOx->PUPDR = (pGPIOHandle->pGPIOx->OSPEEDR & ~(pinPullUpPullDownMode << pinNumber*2)) | (pinPullUpPullDownMode<< pinNumber*2);



}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t pinState)
{

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t portState)
{

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{

}

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, GPIO_IRQControlState cotrolState)
{

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t pinNumber)
{

}





