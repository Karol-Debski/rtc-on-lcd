/*
 * spi_send_data_blocking_mode.c
 *
 *  Created on: Aug 22, 2024
 *      Author: karol
 */

#include <stdint.h>
#include <string.h>

//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

#include "stm32f407xx.h"




int main(void)
{
	// PB6 - SCL
	// PB7 - SDA

	GPIO_Handle_t pinI2C_SCL={0};
	pinI2C_SCL.pGPIOx=GPIOB;
	pinI2C_SCL.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_6;
	pinI2C_SCL.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT_FN;
	pinI2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	pinI2C_SCL.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OUTPUT_TYPE_OD;
	pinI2C_SCL.GPIO_PinConfig.GPIO_PinOutputSpeed = GPIO_OUTPUT_SPEED_HIGH;



	GPIO_Handle_t pinI2C_SDA={0};
	pinI2C_SDA.pGPIOx=GPIOB;
	pinI2C_SDA.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NUM_7;
	pinI2C_SDA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALT_FN;
	pinI2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	pinI2C_SDA.GPIO_PinConfig.GPIO_PinOutputType = GPIO_OUTPUT_TYPE_OD;
	pinI2C_SDA.GPIO_PinConfig.GPIO_PinOutputSpeed = GPIO_OUTPUT_SPEED_HIGH;


	GPIO_Init(&pinI2C_SCL);
	GPIO_Init(&pinI2C_SDA);


	I2C_Handle_t pI2C1Handle = {0};
	pI2C1Handle.pI2Cx=I2C1;
	pI2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	pI2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCLSpeed_StandardMode;

	I2C_Init(&pI2C1Handle);



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



	while(1);
}



void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NUM_0);

	char data[]="Hello World\n";

	uint8_t size = strlen(data);

	I2C_MasterSendData(I2C1, (uint8_t*)data, size, 0x68, ENABLE);

}
