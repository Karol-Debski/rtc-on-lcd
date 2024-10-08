/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Sep 29, 2024
 *      Author: karol
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
}I2C_Handle_t;

typedef enum EI2C_SCLSpeed
{
	I2C_SCLSpeed_StandardMode = 100000,
	I2C_SCLSpeed_FastMode2K =	200000,
	I2C_SCLSpeed_FastMode4K = 	400000,
}EI2C_SCLSpeed;

typedef enum EI2C_FMDuty
{
	I2C_FMDuty_2 = 0,
	I2C_FMDuty_16_9 = 1,
}EI2C_FMDuty;

/******************************** I2c Register Bit fields START ********************************/

typedef enum EI2C_CR1_BitFieldsPositions
{
	I2C_CR1_PE = 0,
	I2C_CR1_SMBUS = 1,
	I2C_CR1_SMBTYPE = 3,
	I2C_CR1_ENARP = 4,
	I2C_CR1_ENPEC = 5,
	I2C_CR1_ENGC = 6,
	I2C_CR1_NOSTRETCH = 7,
	I2C_CR1_START = 8,
	I2C_CR1_STOP = 9,
	I2C_CR1_ACK = 10,
	I2C_CR1_POS = 11,
	I2C_CR1_PEC = 12,
	I2C_CR1_ALERT = 13,
	I2C_CR1_SWRST = 15,
}EI2C_CR1_BitFieldsPositions;

typedef enum EI2C_SR1_BitFieldsPositions
{
	I2C_SR1_SB = 0,
	I2C_SR1_ADDR = 1,
	I2C_SR1_BTF = 2,
	I2C_SR1_ADD10 = 3,
	I2C_SR1_STOPF = 4,
	I2C_SR1_RxNE = 6,
	I2C_SR1_TxE = 7,
	I2C_SR1_BERR = 8,
	I2C_SR1_ARLO = 9,
	I2C_SR1_AF = 10,
	I2C_SR1_OVR = 11,
	I2C_SR1_PECERR = 12,
	I2C_SR1_TIMEOUT = 14,
	I2C_SR1_SMBALERT = 15,
}EI2C_SR1_BitFieldsPositions;

/******************************** I2C Register Bit fields END ********************************/

typedef enum EI2C_ClockPeriControlState
{
	I2C_ClockPeriDisable=0,
	I2C_ClockPeriEnable=1,
}EI2C_ClockPeriControlState;

typedef enum EI2C_PeriControlState
{
	I2C_Disable=0,
	I2C_Enable=1,
}EI2C_PeriControlState;

typedef enum EI2C_IRQControlState
{
	I2C_IRQ_Disable=0,
	I2C_IRQ_Enable=1,
}EI2C_IRQControlState;

typedef enum EI2C_OperationType
{
	I2C_Write=0,
	I2C_Read=1,
}EI2C_OperationType;

/*
 * Clock control
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, EI2C_ClockPeriControlState clockControlState);


/*
 * Init and deinit I2C
 */
void I2C_Init(I2C_Handle_t* pI2CHandle);

void I2C_DeInit(I2C_RegDef_t* pI2Cx);

void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, EI2C_PeriControlState controlState);



/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_RegDef_t* pI2Cx, uint8_t* pTxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop);

void I2C_MasterReceiveData(I2C_RegDef_t* pI2Cx, uint8_t* pRxBuffer, uint8_t size, uint8_t slaveAddr, uint8_t endWithStop);




/*
 * IRQ config and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, EI2C_IRQControlState state);

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);



#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
