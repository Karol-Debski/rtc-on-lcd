/*
 * lcd.c
 *
 *  Created on: Oct 27, 2024
 *      Author: karol
 */
#include <string.h>
#include "stm32f407xx.h"
#include "lcd.h"



void uDelay(uint32_t microseconds)
{
	for(uint32_t i = 0; i < (microseconds*DELAY_COUNT_1US); i++);
}

void mDelay(uint32_t miliseconds)
{
	for(uint32_t i = 0; i < (miliseconds*DELAY_COUNT_1MS); i++);
}


static void LCD_write4Bits(uint8_t dataInHex)
{
	// backlight
	dataInHex |= BT_BIT_SET_I2C;

	dataInHex |= E_BIT_SET_I2C;
	I2C_MasterSendData(I2C1, &dataInHex, 1, LCD_ADDRESS, DISABLE);

	uDelay(10);

	dataInHex &= ~E_BIT_SET_I2C;
	I2C_MasterSendData(I2C1, &dataInHex, 1, LCD_ADDRESS, ENABLE);

	uDelay(10);
}


static uint8_t LCD_FunctionSetIn4BitMode(uint8_t dataLength, uint8_t displayLines, uint8_t characterFont)
{
	uint8_t dataInHexFirst4Bits = D5_BIT_SET_I2C;
	uint8_t dataInHexSecond4Bits = 0;


	if(dataLength == 8 )
	{
		dataInHexSecond4Bits |= D4_BIT_SET_I2C;
	}
	else if(dataLength != 4)
	{
		return 1;
	}

	if(displayLines == 2)
	{
		dataInHexSecond4Bits |= D3_BIT_SET_I2C;
	}
	else if(displayLines != 1)
	{
		return 1;
	}

	if(characterFont == LCD_5x10Dots)
	{
		dataInHexSecond4Bits |= D2_BIT_SET_I2C;
	}
	else if(characterFont != LCD_5x8Dots)
	{
		return 1;
	}


	LCD_write4Bits(dataInHexFirst4Bits);
	LCD_write4Bits(dataInHexSecond4Bits);

	uDelay(37);

	return 0;
}

uint8_t LCD_DisplayOnOffControlIn4BitMode(uint8_t displayControl, uint8_t cursorControl, uint8_t blinkingCursorControl)
{
	uint8_t dataInHexFirst4Bits = 0;
	uint8_t dataInHexSecond4Bits = D3_BIT_SET_I2C;


	if(displayControl == ENABLE)
	{
		dataInHexSecond4Bits |= D2_BIT_SET_I2C;
	}
	else if(displayControl != DISABLE)
	{
		return 1;
	}

	if(cursorControl == ENABLE)
	{
		dataInHexSecond4Bits |= D1_BIT_SET_I2C;
	}
	else if(cursorControl != DISABLE)
	{
		return 1;
	}

	if(blinkingCursorControl == ENABLE)
	{
		dataInHexSecond4Bits |= D0_BIT_SET_I2C;
	}
	else if(blinkingCursorControl != DISABLE)
	{
		return 1;
	}


	LCD_write4Bits(dataInHexFirst4Bits);
	LCD_write4Bits(dataInHexSecond4Bits);

	uDelay(37);

	return 0;
}

void LCD_ClearDisplayIn4BitMode()
{
	uint8_t dataInHexFirst4Bits = 0;
	uint8_t dataInHexSecond4Bits = D0_BIT_SET_I2C;


	LCD_write4Bits(dataInHexFirst4Bits);
	LCD_write4Bits(dataInHexSecond4Bits);

	uDelay(37);
}

void LCD_ReturnHomeIn4BitMode()
{
	uint8_t dataInHexFirst4Bits = 0;
	uint8_t dataInHexSecond4Bits = D1_BIT_SET_I2C;


	LCD_write4Bits(dataInHexFirst4Bits);
	LCD_write4Bits(dataInHexSecond4Bits);

	mDelay(2);
}

static uint8_t LCD_EntryModeSetIn4BitMode(uint8_t incrementDecrementControl, uint8_t shiftDisplayControl)
{
	uint8_t dataInHexFirst4Bits = 0;
	uint8_t dataInHexSecond4Bits = D2_BIT_SET_I2C;


	if(incrementDecrementControl == LCD_InrementAddressControl)
	{
		dataInHexSecond4Bits |= D1_BIT_SET_I2C;
	}
	else if(incrementDecrementControl != LCD_DecrementAddressControl)
	{
		return 1;
	}

	if(shiftDisplayControl == ENABLE)
	{
		dataInHexSecond4Bits |= D0_BIT_SET_I2C;
	}
	else if(shiftDisplayControl != DISABLE)
	{
		return 1;
	}


	LCD_write4Bits(dataInHexFirst4Bits);
	LCD_write4Bits(dataInHexSecond4Bits);

	uDelay(37);

	return 0;
}

static void LCD_SetDDRAMAddressIn4BitMode(uint8_t address)
{
	uint8_t dataInHexFirst4Bits = D7_BIT_SET_I2C;
	uint8_t dataInHexSecond4Bits = 0;

	dataInHexFirst4Bits |= (address >> 4) << 4;
	dataInHexSecond4Bits |= (address & 0xF) << 4;

	LCD_write4Bits(dataInHexFirst4Bits);
	LCD_write4Bits(dataInHexSecond4Bits);

	uDelay(37);
}

void LCD_DisplaySingleCharIn4BitMode(char* byte)
{
	uint8_t dataInHexFirst4Bits = RS_BIT_SET_I2C;
	uint8_t dataInHexSecond4Bits = RS_BIT_SET_I2C;

	dataInHexFirst4Bits |= (*byte >> 4) << 4;
	dataInHexSecond4Bits |= (*byte & 0xF) << 4;

	LCD_write4Bits(dataInHexFirst4Bits);
	LCD_write4Bits(dataInHexSecond4Bits);
}

void LCD_SetCoursorIn4BitMode(uint8_t address)
{
	LCD_SetDDRAMAddressIn4BitMode(address);
}

void LCD_DisplayStringIn4BitMode(char* string, uint32_t stringSize)
{
	for(uint32_t i = 0; i < stringSize; i++)
	{
		if(i == NUMBER_OF_CHARACTERS_IN_LINE)
		{
			LCD_SetDDRAMAddressIn4BitMode(SECOND_LINE_ADDRESS);
		}

		LCD_DisplaySingleCharIn4BitMode(&string[i]);
	}
}

void LCD_InitIn4BitMode()
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
	pI2C1Handle.I2C_Config.I2C_SCLSpeed = LCD_I2C_SPEED;

	I2C_Init(&pI2C1Handle);

	mDelay(15);

	uint8_t data = D4_BIT_SET_I2C | D5_BIT_SET_I2C;
	LCD_write4Bits(data);

	mDelay(5);

	LCD_write4Bits(data);

	uDelay(100);

	LCD_write4Bits(data);

	uDelay(37);

	data = D5_BIT_SET_I2C;
	LCD_write4Bits(data);

	uDelay(37);


	LCD_FunctionSetIn4BitMode(4, 2, LCD_5x8Dots);

	LCD_DisplayOnOffControlIn4BitMode(DISABLE, DISABLE, DISABLE);

	LCD_ClearDisplayIn4BitMode();

	LCD_EntryModeSetIn4BitMode(LCD_InrementAddressControl, DISABLE);

}













