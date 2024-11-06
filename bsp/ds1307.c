/*
 * ds1307.c
 *
 *  Created on: Oct 27, 2024
 *      Author: karol
 */

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"
#include "ds1307.h"

char* daysOfWeek[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

char* getNameOfDay(uint8_t numberOfDay)
{
	return daysOfWeek[numberOfDay-1];
}

static uint8_t ds1307ReadRegister(uint8_t addressOfRegister)
{
	uint8_t pData = 0;
	I2C_MasterSendData(I2C1, &addressOfRegister, 1, DS1307_ADDRESS, DISABLE);
	I2C_MasterReceiveData(I2C1, &pData, 1, DS1307_ADDRESS, ENABLE);

	return pData;
}

static void ds1307WriteRegister(uint8_t addressOfRegister, uint8_t value)
{
	uint8_t data[2];
	data[0] = addressOfRegister;
	data[1] = value;
	I2C_MasterSendData(I2C1, data, 2, DS1307_ADDRESS, ENABLE);
}


uint8_t ds1307Init()
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
	pI2C1Handle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_Init(&pI2C1Handle);

	// release clock
	uint8_t regVal = 1 << 7;
	ds1307WriteRegister(DS1307_Address_Seconds, regVal);

	// read clock state
	regVal = ds1307ReadRegister(DS1307_Address_Seconds);

	uint8_t isClockEnabled = regVal >> 7;

	return isClockEnabled;
}

static uint8_t convertDecimalToBinaryCodedDecimal(uint8_t decimalValue)
{
	uint32_t binaryCodedDecimalValue = 0;

	uint32_t unitPart = decimalValue%10;
	uint32_t decimalPart = decimalValue/10;

	binaryCodedDecimalValue = (decimalPart << 4) | unitPart;

	return binaryCodedDecimalValue;
}

static uint8_t convertBinaryCodedDecimalToDecimal(uint8_t binaryCodedDecimalValue)
{
	uint8_t decimalValue = 0;

	uint8_t decimalPart = ( binaryCodedDecimalValue & (0xF << 4) ) >> 4;
	uint8_t unitPart = binaryCodedDecimalValue & (0xF);

	decimalValue = decimalPart*10 + unitPart;

	return decimalValue;
}

void ds1307SetCurrentTime(RTC_time_t* pRtcTime)
{
	uint8_t secondsBCD = convertDecimalToBinaryCodedDecimal(pRtcTime->seconds);

	secondsBCD = secondsBCD & (0x7F);

	ds1307WriteRegister(DS1307_Address_Seconds, secondsBCD);


	uint8_t minutesBCD = convertDecimalToBinaryCodedDecimal(pRtcTime->minutes);

	minutesBCD = minutesBCD & (0x7F);

	ds1307WriteRegister(DS1307_Address_Minutes, minutesBCD);



	if(pRtcTime->timeFormat == DS1307_TimeFormat_24)
	{
		uint8_t hoursBCD = convertDecimalToBinaryCodedDecimal(pRtcTime->hours);
		hoursBCD = hoursBCD & (0x3F);

		ds1307WriteRegister(DS1307_Address_Hours, hoursBCD);
	}
	else if((pRtcTime->timeFormat == DS1307_TimeFormat_12_AM) || (pRtcTime->timeFormat == DS1307_TimeFormat_12_PM))
	{
		uint8_t hoursBCD = convertDecimalToBinaryCodedDecimal(pRtcTime->hours);
		hoursBCD = hoursBCD & (0x1F);

		//enable 12h time format
		hoursBCD = hoursBCD | (1U << 6);

		if(pRtcTime->timeFormat == DS1307_TimeFormat_12_PM)
		{
			hoursBCD = hoursBCD | (1U << 5);
		}

		ds1307WriteRegister(DS1307_Address_Hours, hoursBCD);
	}

}

void ds1307GetCurrentTime(RTC_time_t* pRtcTime)
{
	uint8_t secondsBCD = ds1307ReadRegister(DS1307_Address_Seconds);

	secondsBCD = secondsBCD & (0x7F);

	pRtcTime->seconds = convertBinaryCodedDecimalToDecimal(secondsBCD);


	uint8_t minutesBCD = ds1307ReadRegister(DS1307_Address_Minutes);

	minutesBCD = minutesBCD & (0x7F);

	pRtcTime->minutes = convertBinaryCodedDecimalToDecimal(minutesBCD);


	uint8_t hoursBCD = ds1307ReadRegister(DS1307_Address_Hours);

	uint8_t hourMode = (hoursBCD >> 6) & 1U;
	uint8_t AM_PM_mode = (hoursBCD >> 5) & 1U;

	if(hourMode == 0)
	{
		pRtcTime->timeFormat = DS1307_TimeFormat_24;
		hoursBCD = hoursBCD & (0x3F);
		pRtcTime->hours = convertBinaryCodedDecimalToDecimal(hoursBCD);
	}
	else if(hourMode == 1)
	{
		if(AM_PM_mode == 1)
		{
			pRtcTime->timeFormat = DS1307_TimeFormat_12_PM;
		}
		else if(AM_PM_mode == 0)
		{
			pRtcTime->timeFormat = DS1307_TimeFormat_12_AM;
		}
		hoursBCD = hoursBCD & (0x1F);
		pRtcTime->hours = convertBinaryCodedDecimalToDecimal(hoursBCD);
	}

}

void ds1307SetCurrentDate(RTC_date_t* pRtcDate)
{
	uint8_t dayOfWeekBCD = convertDecimalToBinaryCodedDecimal(pRtcDate->dayOfWeek);

	dayOfWeekBCD = dayOfWeekBCD & (0x7);

	ds1307WriteRegister(DS1307_Address_Day, dayOfWeekBCD);


	uint8_t dateBCD = convertDecimalToBinaryCodedDecimal(pRtcDate->date);

	dateBCD = dateBCD & (0x3F);

	ds1307WriteRegister(DS1307_Address_Date, dateBCD);


	uint8_t monthBCD = convertDecimalToBinaryCodedDecimal(pRtcDate->month);

	monthBCD = monthBCD & (0x1F);

	ds1307WriteRegister(DS1307_Address_Month, monthBCD);


	uint8_t yearBCD = convertDecimalToBinaryCodedDecimal(pRtcDate->year);

	ds1307WriteRegister(DS1307_Address_Year, yearBCD);
}

void ds1307GetCurrentDate(RTC_date_t* pRtcDate)
{
	uint8_t dayOfWeekBCD = ds1307ReadRegister(DS1307_Address_Day);

	dayOfWeekBCD = dayOfWeekBCD & (0x7);

	pRtcDate->dayOfWeek = convertBinaryCodedDecimalToDecimal(dayOfWeekBCD);


	uint8_t dateBCD = ds1307ReadRegister(DS1307_Address_Date);

	dateBCD = dateBCD & (0x3F);

	pRtcDate->date = convertBinaryCodedDecimalToDecimal(dateBCD);


	uint8_t monthBCD = ds1307ReadRegister(DS1307_Address_Month);

	monthBCD = monthBCD & (0x1F);

	pRtcDate->month = convertBinaryCodedDecimalToDecimal(monthBCD);


	uint8_t yearBCD = ds1307ReadRegister(DS1307_Address_Year);

	pRtcDate->year = convertBinaryCodedDecimalToDecimal(yearBCD);
}
