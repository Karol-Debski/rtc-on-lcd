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



typedef enum
{
	ARDUINO_COMMAND_LED_CTRL = 0x50,
	ARDUINO_COMMAND_SENSOR_READ = 0x51,
	ARDUINO_COMMAND_LED_RED = 0x52,
	ARDUINO_COMMAND_PRINT = 0x53,
	ARDUINO_COMMAND_ID_READ = 0x54,
}EArduinoSpi_Commands;

typedef enum
{
	ARDUINO_LED_OFF = 0,
	ARDUINO_LED_ON = 1,
}EArduino_LedState;

typedef enum
{
	ARDUINO_ANALOG_PIN_0 = 0,
	ARDUINO_ANALOG_PIN_1 = 1,
	ARDUINO_ANALOG_PIN_2 = 2,
	ARDUINO_ANALOG_PIN_3 = 3,
	ARDUINO_ANALOG_PIN_4 = 4,
}EArduino_AnalogPins;

typedef enum
{
	ARDUINO_LED_PIN = 9,
}EArduino_LedPin;

int main(void)
{
	// PA5 - SPI1_SCK
	// PA7 - SPI1_MOSI
	// PA6 - SPI1_MISO
	// PA4 - SPI1_NSS
	// AF5

	GPIO_Handle_t pinSPI_SCK={0};
	pinSPI_SCK=(GPIO_Handle_t){
			.pGPIOx=GPIOB,
			.GPIO_PinConfig=(GPIO_PinConfig_t){
				.GPIO_PinNumber=GPIO_PIN_NUM_13,
				.GPIO_PinMode=GPIO_MODE_ALT_FN,
				.GPIO_PinAltFunMode=5,
				.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP,
		}
	};

	GPIO_Handle_t pinSPI_MOSI={0};
	pinSPI_MOSI=(GPIO_Handle_t){
			.pGPIOx=GPIOB,
			.GPIO_PinConfig=(GPIO_PinConfig_t){
				.GPIO_PinNumber=GPIO_PIN_NUM_15,
				.GPIO_PinMode=GPIO_MODE_ALT_FN,
				.GPIO_PinAltFunMode=5,
				.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP,
		}
	};

	GPIO_Handle_t pinSPI_NSS={0};
	pinSPI_NSS=(GPIO_Handle_t){
		.pGPIOx=GPIOB,
		.GPIO_PinConfig=(GPIO_PinConfig_t){
			.GPIO_PinNumber=GPIO_PIN_NUM_12,
			.GPIO_PinMode=GPIO_MODE_ALT_FN,
			.GPIO_PinAltFunMode=5,
			.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP,
		}
	};

	GPIO_Handle_t pinSPI_MISO={0};
	pinSPI_MISO=(GPIO_Handle_t){
		.pGPIOx=GPIOB,
		.GPIO_PinConfig=(GPIO_PinConfig_t){
			.GPIO_PinNumber=GPIO_PIN_NUM_14,
			.GPIO_PinMode=GPIO_MODE_ALT_FN,
			.GPIO_PinAltFunMode=5,
			.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP,
		}
	};


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

	GPIO_Init(&pinSPI_SCK);
	GPIO_Init(&pinSPI_MOSI);
	GPIO_Init(&pinSPI_NSS);
	GPIO_Init(&pinSPI_MISO);


	SPI_Handle_t SPI2Handler = {0};
	SPI2Handler=(SPI_Handle_t){
		.pSPIx = SPI2,
		.SPIConfig = (SPI_Config_t){
			.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER,
			.SPI_SclkSpeed = SPI_CLOCK_SPEED_EQUAL_F_PCLK_DIV_8,
			.SPI_DataFrameFormat = SPI_8_BIT,
			.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX,
			.SPI_ClockPolarity = SPI_IDLE_STATE_LOW,
			.SPI_ClockPhase = SPI_SAMPLING_ON_THE_FIRST_EDGE,
			.SPI_SoftwareSlaveManagment = SPI_SOFTWARE_SLAVE_MANAGMENT_DISABLED,
		}
	};


	SPI_Init(&SPI2Handler);

	//SPI_SSIConfig(SPI1Handler.pSPIx,SPI_NSS_HIGH);

	SPI_SSOEConfig(SPI2Handler.pSPIx, SPI_SLAVE_SELECT_CONFIG_SINGLE_MASTER);



	while(1);
}

void turnOnArduinoLed()
{
	uint8_t dummyTxMessage = 0xFF;
	uint8_t dummyRxMessage = 0x0;

	uint8_t message = ARDUINO_COMMAND_LED_CTRL;

	SPI_SendData(SPI2, &message, 1);
	SPI_ReceiveData(SPI2, &dummyRxMessage, 1);



	SPI_SendData(SPI2, &dummyTxMessage, 1);
	uint8_t ackByte=0;
	SPI_ReceiveData(SPI2, &ackByte, 1);



	if(ackByte == 0xF5)
	{
		uint8_t args[2];
		args[0]=ARDUINO_LED_PIN;
		args[1]=ARDUINO_LED_ON;

		SPI_SendData(SPI2, args, 2);
	}
}

void sensorReadArduino()
{
	uint8_t dummyTxMessage = 0xFF;
	uint8_t dummyRxMessage = 0x0;

	uint8_t message = ARDUINO_COMMAND_SENSOR_READ;

	SPI_SendData(SPI2, &message, 1);
	SPI_ReceiveData(SPI2, &dummyRxMessage, 1);



	SPI_SendData(SPI2, &dummyTxMessage, 1);
	uint8_t ackByte=0;
	SPI_ReceiveData(SPI2, &ackByte, 1);



	if(ackByte == 0xF5)
	{
		uint8_t arg=ARDUINO_ANALOG_PIN_0;
		uint8_t readFromSensor=0;

		SPI_SendData(SPI2, &arg, 1);
		SPI_ReceiveData(SPI2, &dummyRxMessage, 1);

		for(int32_t i=0;i<50000;i++);

		SPI_SendData(SPI2, &dummyTxMessage, 1);
		SPI_ReceiveData(SPI2, &readFromSensor, 1);
	}
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NUM_0);
	SPI_PeripheralControl(SPI2, SPI_ENABLE);

	//turnOnArduinoLed();
	sensorReadArduino();

	SPI_PeripheralControl(SPI2, SPI_DISABLE);
}
