/*
 * usart_send_data_it_mode.c
 *
 *  Created on: Oct 23, 2024
 *      Author: karol
 */

#include <stdint.h>
#include <string.h>

//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

#include "stm32f407xx.h"

char message[] = "Hello World\n";
uint32_t isButtonPushed = 0;

int main(void)
{
	// PB6 - SCL
	// PB7 - SDA

	// baudrate 115200, 1 stop bit, 8 bit, no parity


	GPIO_Handle_t pinUSART_Tx={0};
	pinUSART_Tx.pGPIOx=GPIOA;
	pinUSART_Tx.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_2;
	pinUSART_Tx.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT_FN;
	pinUSART_Tx.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	pinUSART_Tx.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OUTPUT_TYPE_PP;
	pinUSART_Tx.GPIO_PinConfig.GPIO_PinPullUpPullDownMode = GPIO_PULL_UP;
	pinUSART_Tx.GPIO_PinConfig.GPIO_PinOutputSpeed = GPIO_OUTPUT_SPEED_HIGH;



	GPIO_Handle_t pinUSART_Rx={0};
	pinUSART_Rx.pGPIOx=GPIOA;
	pinUSART_Rx.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_2;
	pinUSART_Rx.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT_FN;
	pinUSART_Rx.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	pinUSART_Rx.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OUTPUT_TYPE_PP;
	pinUSART_Rx.GPIO_PinConfig.GPIO_PinPullUpPullDownMode = GPIO_PULL_UP;
	pinUSART_Rx.GPIO_PinConfig.GPIO_PinOutputSpeed = GPIO_OUTPUT_SPEED_HIGH;


	GPIO_Init(&pinUSART_Tx);
	GPIO_Init(&pinUSART_Rx);


	USART_Handle_t pUSART1Handle = {0};
	pUSART1Handle.pUSARTx=USART2;
	pUSART1Handle.USART_Config.USART_BaudRate = USART_StandardBaudRate_115200;
	pUSART1Handle.USART_Config.USART_DataSize = USART_DataSize_8Bits;
	pUSART1Handle.USART_Config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	pUSART1Handle.USART_Config.USART_Mode = USART_Mode_Tx;
	pUSART1Handle.USART_Config.USART_OversamplingMode = USART_OversamplingMode_16SampelsPerBit;
	pUSART1Handle.USART_Config.USART_ParityControl = USART_Parity_Disable;
	pUSART1Handle.USART_Config.USART_StopBitSize = USART_StopBitSize_1;

	USART_Init(&pUSART1Handle);

	USART_IRQInterruptConfig(IRQ_NUM_USART2, USART_IRQ_Enable);
	USART_IRQPriorityConfig(IRQ_NUM_USART2, 0);


	GPIO_Handle_t gpioButtonDHandler={0};
	gpioButtonDHandler=(GPIO_Handle_t){
		.pGPIOx=GPIOA,
		.GPIO_PinConfig=(GPIO_PinConfig_t){
		.GPIO_PinNumber=GPIO_PIN_NUM_0,
		.GPIO_PinMode=GPIO_MODE_IT_RT,
		.GPIO_PinPullUpPullDownMode=GPIO_NO_PULL_DONW_AND_UP,
		}
	};

	GPIO_Init(&gpioButtonDHandler);

	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI0, 1);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI0, GPIO_IRQ_ENABLE);



	while(1)
	{
		if(isButtonPushed == 1)
		{
			uint32_t sizeOfMessage = sizeof(message);
			USART_SendDataIT(pUSART1Handle.pUSARTx, (uint8_t*)message, sizeOfMessage);

			isButtonPushed = 0;
		}
	}
}



void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NUM_0);
	isButtonPushed = 1;
}
