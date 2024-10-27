/*
 * i2c_receive_data_slave.c
 *
 *  Created on: Oct 14, 2024
 *      Author: karol
 */

#include <stdint.h>
#include <string.h>

//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

#include "stm32f407xx.h"


uint8_t command = 0;
char dataBuffer[]="Hello world\n";
uint32_t sizeOfData=12;
uint8_t isButtonPushed = 0;

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
	pI2C1Handle.I2C_Config.I2C_DeviceAddress = 0x68;
	pI2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCLSpeed_StandardMode;


	I2C_Init(&pI2C1Handle);

	I2C_IRQInterruptConfig(IRQ_NUM_I2C1_EVENT, I2C_IRQ_Enable);
	I2C_IRQPriorityConfig(IRQ_NUM_I2C1_EVENT, 0);


	while(1)
	{
		I2C_SlaveReceiveData(I2C1, &command, 1, ENABLE);

		if(command == 0x51)
		{
			I2C_SlaveSendData(I2C1, (uint8_t*)(&sizeOfData), 4);
		}

		I2C_SlaveReceiveData(I2C1, &command, 1, DISABLE);

		if(command == 0x52)
		{
			I2C_SlaveSendData(I2C1, (uint8_t*)dataBuffer, sizeOfData);
		}
	}
}

