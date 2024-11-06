/*
 * main.c
 *
 *  Created on: Oct 27, 2024
 *      Author: karol
 */


#include <stdint.h>
#include <string.h>
#include <stdio.h>

//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

#include "stm32f407xx.h"
#include "ds1307.h"
#include "lcd.h"




int main(void)
{
	LCD_InitIn4BitMode();
	LCD_DisplayOnOffControlIn4BitMode(ENABLE, DISABLE, DISABLE);

	ds1307Init();

	RTC_time_t rtcTime = {0};
	rtcTime.seconds = 30;
	rtcTime.minutes = 3;
	rtcTime.hours = 21;
	rtcTime.timeFormat = DS1307_TimeFormat_24;

	RTC_date_t rtcDate = {0};
	rtcDate.year = 24;
	rtcDate.month = 11;
	rtcDate.dayOfWeek = 3;
	rtcDate.date = 6;

	ds1307SetCurrentDate(&rtcDate);
	ds1307SetCurrentTime(&rtcTime);




	while(1)
	{
		LCD_ClearDisplayIn4BitMode();
		ds1307GetCurrentTime(&rtcTime);
		ds1307GetCurrentDate(&rtcDate);


		char dateBuffer[16] = {0};
		uint8_t numOfDateCharactersWritten = sprintf(dateBuffer, "%d.%d.%d,%s", rtcDate.date, rtcDate.month, rtcDate.year, getNameOfDay(rtcDate.date));

		LCD_DisplayStringIn4BitMode(dateBuffer, numOfDateCharactersWritten);

		LCD_SetCoursorIn4BitMode(SECOND_LINE_ADDRESS);

		char timeBuffer[16] = {0};
		uint8_t numOfTimeCharactersWritten = sprintf(timeBuffer, "%02dh%02dm%02ds", rtcTime.hours, rtcTime.minutes, rtcTime.seconds);

		LCD_DisplayStringIn4BitMode(timeBuffer, numOfTimeCharactersWritten);

		mDelay(1000);
	}
}
