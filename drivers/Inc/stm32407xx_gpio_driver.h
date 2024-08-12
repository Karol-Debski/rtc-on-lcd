/*
 * stm32407xx_gpio_driver.h
 *
 *  Created on: Oct 29, 2023
 *      Author: karol
 */

#ifndef INC_STM32407XX_GPIO_DRIVER_H_
#define INC_STM32407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;					// EGPIO_PinNumber
	uint8_t GPIO_PinMode;					// EGPIO_Mode
	uint8_t GPIO_PinOutputSpeed;			// EGPIO_OutputSpeed
	uint8_t GPIO_PinPullUpPullDownMode;		// EGPIO_PullUpPullDownMode
	uint8_t GPIO_PinOutputType;				// EGPIO_OutputType
	uint8_t GPIO_PinAltFunMode;				//
} GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t* pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

typedef enum EGPIO_ClockControlState
{
	GPIO_CLOCK_DISABLE = 0,
	GPIO_CLOCK_ENABLE = 1,
} EGPIO_ClockControlState;

typedef enum EGPIO_IRQControlState
{
	GPIO_IRQ_DISABLE = 0,
	GPIO_IRQ_ENABLE = 1,
} GPIO_IRQControlState;


typedef enum EGPIO_PinState
{
	GPIO_LOW = 0,
	GPIO_HIGH = 1,
} EGPIO_PinState;

typedef enum EGPIO_Mode
{
	GPIO_MODE_INPUT = 0x0,
	GPIO_MODE_OUTPUT = 0x1,
	GPIO_MODE_ALT_FN = 0x2,
	GPIO_MODE_ANALOG = 0x3,
	GPIO_MODE_IT_FT = 4,
	GPIO_MODE_IT_RT = 5,
	GPIO_MODE_IT_RFT = 6,
} EGPIO_Mode;

typedef enum EGPIO_OutputType
{
	GPIO_OUTPUT_TYPE_PP = 0x0,
	GPIO_OUTPUT_TYPE_OP = 0x1,
} EGPIO_OutputType;

typedef enum EGPIO_OutputSpeed
{
	GPIO_OUTPUT_SPEED_LOW = 0x0,
	GPIO_OUTPUT_SPEED_MEDIUM = 0x1,
	GPIO_OUTPUT_SPEED_HIGH = 0x2,
	GPIO_OUTPUT_SPEED_VERY_HIGH = 0x3,
} EGPIO_OutputSpeed;

typedef enum EGPIO_PullUpPullDownMode
{
	GPIO_NO_PULL_DONW_AND_UP = 0,
	GPIO_PULL_DONW = 1,
	GPIO_PULL_UP = 2,
} EGPIO_PullUpPullDownMode;

typedef enum EGPIO_PinNumber
{
	GPIO_PIN_NUM_0 = 0,
	GPIO_PIN_NUM_1 = 1,
	GPIO_PIN_NUM_2 = 2,
    GPIO_PIN_NUM_3 = 3,
    GPIO_PIN_NUM_4 = 4,
    GPIO_PIN_NUM_5 = 5,
    GPIO_PIN_NUM_6 = 6,
    GPIO_PIN_NUM_7 = 7,
    GPIO_PIN_NUM_8 = 8,
    GPIO_PIN_NUM_9 = 9,
    GPIO_PIN_NUM_10 = 10,
    GPIO_PIN_NUM_11 = 11,
    GPIO_PIN_NUM_12 = 12,
    GPIO_PIN_NUM_13 = 13,
    GPIO_PIN_NUM_14 = 14,
    GPIO_PIN_NUM_15 = 15,
} EGPIO_PinNumber;

/*
 * Gpio clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, EGPIO_ClockControlState clockControlState);

/*
 * Gpio init/de-init
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * Gpio read from input pin/port
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);

/*
 * Gpio write to output pin/port
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t pinState);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t portState);

/*
 * Gpio toggle pin output
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);

/*
 * Gpio IRQ config
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, GPIO_IRQControlState cotrolState);

/*
 * Gpio IRQ callback
 */
void GPIO_IRQHandling(uint8_t pinNumber);









#endif /* INC_STM32407XX_GPIO_DRIVER_H_ */
