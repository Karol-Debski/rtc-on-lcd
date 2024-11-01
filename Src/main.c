/*
 * main.c
 *
 *  Created on: Oct 27, 2024
 *      Author: karol
 */


#include <stdint.h>
#include <string.h>

//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

#include "stm32f407xx.h"
#include "ds1307.h"


int main(void)
{
	uint8_t val = ds1307Init();
	RTC_time_t rtcTime1 = {0};
	rtcTime1.seconds = 1;
	rtcTime1.minutes = 1;
	rtcTime1.hours = 1;
	rtcTime1.timeFormat = DS1307_TimeFormat_24;


	ds1307SetCurrentTime(&rtcTime1);

	RTC_time_t rtcTime2 = {0};

	for(int i = 1; i < 10000000; i++);


	ds1307GetCurrentTime(&rtcTime2);
	while(1);
}

