/*
 * lcd.h
 *
 *  Created on: Oct 27, 2024
 *      Author: karol
 */

#ifndef LCD_H_
#define LCD_H_

#define LCD_ADDRESS 		0x27U
#define LCD_I2C_SPEED		100000U

#define NUMBER_OF_CHARACTERS_IN_LINE 16U

#define SECOND_LINE_ADDRESS	0x40U

#define RS_BIT_SET_I2C 		(uint8_t)(1U << 0)
#define RW_BIT_SET_I2C		(uint8_t)(1U << 1)
#define E_BIT_SET_I2C 		(uint8_t)(1U << 2)
#define BT_BIT_SET_I2C		(uint8_t)(1U << 3)
#define D4_BIT_SET_I2C 		(uint8_t)(1U << 4)
#define D5_BIT_SET_I2C 		(uint8_t)(1U << 5)
#define D6_BIT_SET_I2C 		(uint8_t)(1U << 6)
#define D7_BIT_SET_I2C 		(uint8_t)(1U << 7)
#define D0_BIT_SET_I2C		D4_BIT_SET_I2C
#define D1_BIT_SET_I2C		D5_BIT_SET_I2C
#define D2_BIT_SET_I2C		D6_BIT_SET_I2C
#define D3_BIT_SET_I2C		D7_BIT_SET_I2C

	/* P0 - RS - 0 - instruction, 1 - data register
	 * P1 - RW 0 - write, 1 - read
	 * P2 - E - start data write
	 * P3 - backlight
	 * P4 - D4
	 * P5 - D5
	 * P6 - D6
	 * P7 - D7 - busy flag
	 * */

#define DELAY_COUNT_1US			2U
#define DELAY_COUNT_1MS 		1250U


typedef enum ELCD_CharacterFont
{
	LCD_5x8Dots = 	0U,
	LCD_5x10Dots = 	1U,
}ELCD_CharacterFont;

typedef enum ELCD_DirectionAddressControl
{
	LCD_InrementAddressControl = 	0U,
	LCD_DecrementAddressControl = 	1U,
}ELCD_DirectionAddressControl;



void LCD_SetCoursorIn4BitMode(uint8_t address);

uint8_t LCD_DisplayOnOffControlIn4BitMode(uint8_t displayControl, uint8_t cursorControl, uint8_t blinkingCursorControl);

void LCD_ClearDisplayIn4BitMode();

void LCD_ReturnHomeIn4BitMode();

void LCD_DisplaySingleCharIn4BitMode(char* byte);

void LCD_DisplayStringIn4BitMode(char* string, uint32_t stringSize);

void LCD_InitIn4BitMode();


void uDelay(uint32_t microseconds);
void mDelay(uint32_t miliseconds);

#endif /* LCD_H_ */
