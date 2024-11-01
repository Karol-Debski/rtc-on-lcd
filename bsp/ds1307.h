/*
 * ds1307.h
 *
 *  Created on: Oct 27, 2024
 *      Author: karol
 */

#ifndef DS1307_H_
#define DS1307_H_


#define DS1307_ADDRESS 		0x68U
#define DS1307_I2C_SPEED	100000U

typedef struct
{
	uint8_t		seconds;
	uint8_t		minutes;
	uint8_t		hours;
	uint8_t		timeFormat;
}RTC_time_t;

typedef struct
{
	uint8_t		dayOfWeek;
	uint8_t		date;
	uint8_t		month;
	uint8_t		year;
}RTC_date_t;

typedef enum EDS1307_TimeFormat
{
	DS1307_TimeFormat_24	= 0U,
	DS1307_TimeFormat_12_AM	= 1U,
	DS1307_TimeFormat_12_PM	= 2U,
}EDS1307_TimeFormat;

typedef enum EDS1307_RTCAndRamAddress
{
	DS1307_Address_Seconds	= 0x0U,
	DS1307_Address_Minutes	= 0x1U,
	DS1307_Address_Hours	= 0x2U,
	DS1307_Address_Day		= 0x3U,
	DS1307_Address_Date		= 0x4U,
	DS1307_Address_Month	= 0x5U,
	DS1307_Address_Year		= 0x6U,
	DS1307_Address_Control	= 0x7U,
	DS1307_Address_RAMStart	= 0x8U,
	DS1307_Address_RAMEnd	= 0x3FU,
}EDS1307_RTCAndRamAddress;




uint8_t ds1307Init();

void ds1307SetCurrentTime(RTC_time_t* pRtcTime);
void ds1307GetCurrentTime(RTC_time_t* pRtcTime);

void ds1307SetCurrentDate(RTC_date_t* pRtcDate);
void ds1307GetCurrentDate(RTC_date_t* pRtcDate);

#endif /* DS1307_H_ */
