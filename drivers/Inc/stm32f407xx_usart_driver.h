/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Oct 16, 2024
 *      Author: karol
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t 	USART_Mode;
	uint8_t		USART_OversamplingMode;
	uint32_t 	USART_BaudRate;
	uint8_t 	USART_StopBitSize;
	uint8_t 	USART_DataSize;
	uint8_t 	USART_ParityControl;
	uint8_t 	USART_HardwareFlowControl;
}USART_Config_t;


typedef struct
{
	USART_RegDef_t*		pUSARTx;
	USART_Config_t		USART_Config;
	uint8_t*			pTxBuffer;
	uint8_t*			pRxBuffer;
	uint32_t			TxSize;
	uint32_t			RxSize;
	uint8_t				TxState;
	uint8_t				RxState;
}USART_Handle_t;


typedef enum EUSART_TxState
{
	USART_TxReady 		= 0U,
	USART_BusyInTx 		= 1U,
}EUSART_TxState;

typedef enum EUSART_RxState
{
	USART_RxReady 		= 0U,
	USART_BusyInRx 		= 1U,
}EUSART_RxState;

typedef enum EUSART_StandardBaudRate
{
	USART_StandardBaudRate_1200 	= 1200U,
	USART_StandardBaudRate_2400 	= 2400U,
	USART_StandardBaudRate_9600 	= 9600U,
	USART_StandardBaudRate_19200 	= 19200U,
	USART_StandardBaudRate_38400 	= 38400U,
	USART_StandardBaudRate_57600 	= 57600U,
	USART_StandardBaudRate_115200 	= 115200U,
	USART_StandardBaudRate_230400 	= 230400U,
	USART_StandardBaudRate_460800 	= 460800U,
	USART_StandardBaudRate_921600 	= 921600U,
	USART_StandardBaudRate_2000000 	= 2000000U,
	USART_StandardBaudRate_3000000 	= 3000000U,
}EUSART_StandardBaudRate;

typedef enum EUSART_HardwareFlowControl
{
	USART_HardwareFlowControl_None 		= 0U,
	USART_HardwareFlowControl_CTS 		= 1U,
	USART_HardwareFlowControl_RTS 		= 2U,
	USART_HardwareFlowControl_CTS_RTS 	= 3U,
}EUSART_HardwareFlowControl;

typedef enum EUSART_StopBitSize
{
	USART_StopBitSize_1 	= 0U,
	USART_StopBitSize_0_5 	= 1U,
	USART_StopBitSize_2 	= 2U,
	USART_StopBitSize_1_5	= 3U,
}EUSART_StopBitSize;

typedef enum EUSART_OversamplingMode
{
	USART_OversamplingMode_16SampelsPerBit = 0U,
	USART_OversamplingMode_8SampelsPerBit = 1U,
}EUSART_OversamplingMode;

typedef enum EUSART_DataSize
{
	USART_DataSize_8Bits = 0U,
	USART_DataSize_9Bits = 1U,
}EUSART_DataSize;

typedef enum EUSART_Parity
{
	USART_Parity_Disable 	= 0U,
	USART_Parity_Odd 		= 1U,
	USART_Parity_Even 		= 2U,
}EUSART_Parity;

typedef enum EUSART_IRQControlState
{
	USART_IRQ_Disable	=0U,
	USART_IRQ_Enable	=1U,
}EUSART_IRQControlState;

typedef enum EUSART_PeriControlState
{
	USART_Disable	= 0U,
	USART_Enable	= 1U,
}EUSART_PeriControlState;

typedef enum EUSART_ClockPeriControlState
{
	USART_ClockPeriDisable	= 0U,
	USART_ClockPeriEnable	= 1U,
}EUSART_ClockPeriControlState;

typedef enum EUSART_Mode
{
	USART_Mode_Tx	= 0U,
	USART_Mode_Rx	= 0U,
	USART_Mode_TxRx	= 0U,
}EUSART_Mode;

typedef enum EUSART_TransmitterControl
{
	USART_TransmitterDisable	= 0U,
	USART_TransmitterEnable		= 1U,
}EUSART_TransmitterControl;

typedef enum EUSART_ReceiverControl
{
	USART_ReceiverDisable	= 0U,
	USART_ReceiverEnable	= 1U,
}EUSART_ReceiverControl;

/******************************** USART Register Bit fields START ********************************/

typedef enum EUSART_SR_BitFieldsPositions
{
	USART_SR_PE 	= 0U,
	USART_SR_FE 	= 1U,
	USART_SR_NF 	= 2U,
	USART_SR_ORE 	= 3U,
	USART_SR_IDLE 	= 4U,
	USART_SR_RXNE 	= 5U,
	USART_SR_TC 	= 6U,
	USART_SR_TXE 	= 7U,
	USART_SR_LBD 	= 8U,
	USART_SR_CTS 	= 9U,
}EUSART_SR_BitFieldsPositions;

typedef enum EUSART_CR1_BitFieldsPositions
{
	USART_CR1_SBK 		= 0U,
	USART_CR1_RWU 		= 1U,
	USART_CR1_RE 		= 2U,
	USART_CR1_TE 		= 3U,
	USART_CR1_IDLEIE 	= 4U,
	USART_CR1_RXNEIE 	= 5U,
	USART_CR1_TCIE 		= 6U,
	USART_CR1_TXEIE 	= 7U,
	USART_CR1_PEIE 		= 8U,
	USART_CR1_PS 		= 9U,
	USART_CR1_PCE 		= 10U,
	USART_CR1_WAKE 		= 11U,
	USART_CR1_M 		= 12U,
	USART_CR1_UE 		= 13U,
	USART_CR1_OVER8 	= 15U,
}EUSART_CR1_BitFieldsPositions;

typedef enum EUSART_CR2_BitFieldsPositions
{
	USART_CR2_ADD 		= 0U,
	USART_CR2_LBDL 		= 5U,
	USART_CR2_LBDIE 	= 6U,
	USART_CR2_LBCL 		= 8U,
	USART_CR2_CPHA 		= 9U,
	USART_CR2_CPOL 		= 10U,
	USART_CR2_CLKEN 	= 11U,
	USART_CR2_STOP 		= 12U,
	USART_CR2_LINEN 	= 14U,
}EUSART_CR2_BitFieldsPositions;

typedef enum EUSART_CR3_BitFieldsPositions
{
	USART_CR3_EIE 		= 0U,
	USART_CR3_IREN 		= 1U,
	USART_CR3_IRLP 		= 2U,
	USART_CR3_HDSEL 	= 3U,
	USART_CR3_NACK 		= 4U,
	USART_CR3_SCEN 		= 5U,
	USART_CR3_DMAR 		= 6U,
	USART_CR3_DMAT 		= 7U,
	USART_CR3_RTSE 		= 8U,
	USART_CR3_CTSE 		= 9U,
	USART_CR3_CTSIE 	= 10U,
	USART_CR3_ONEBIT 	= 11U,
}EUSART_CR3_BitFieldsPositions;

/******************************** USART Register Bit fields END ********************************/

typedef enum EUSART_IT_Flag
{
	/* EVENT */
	USART_IT_EVENT_Transmit_Data_Register_Empty = 0U,
	USART_IT_EVENT_CTS_Flag,
	USART_IT_EVENT_Transmission_Complete,
	USART_IT_EVENT_Received_Data_Ready_To_Be_Read,
	USART_IT_EVENT_Overrun_Error_Detected,
	USART_IT_EVENT_Idle_Line_Detected,
	USART_IT_EVENT_Parity_Error,
	USART_IT_EVENT_LIN_Break_Flag,
	USART_IT_EVENT_Noise_Flag_Overrun_Error_And_Framing_Error,

	USART_IT_Unknown,
}EUSART_IT_Flag;

/*
 * Clock control USARTx
 */
void USART_PeripheralClockControl(USART_RegDef_t* pUSARTx, EUSART_ClockPeriControlState clockControlState);


/*
 * Init and deinit USARTx
 */
uint8_t USART_Init(USART_Handle_t* pUSARTHandle);

void USART_DeInit(USART_RegDef_t* pUSARTx);

void USART_PeripheralControl(USART_RegDef_t* pUSARTx, EUSART_PeriControlState controlState);


/*
 * Data send and receive blocking
 */
uint8_t USART_SendData(USART_RegDef_t* pUSARTx, uint8_t* pTxBuffer, uint8_t size);
uint8_t USART_ReceiveData(USART_RegDef_t* pUSARTx, uint8_t* pRxBuffer, uint8_t size);


/*
 * Data send and receive interrupt
 */
uint8_t USART_SendDataIT(USART_RegDef_t* pUSARTx, uint8_t* pTxBuffer, uint8_t size);
uint8_t USART_ReceiveDataIT(USART_RegDef_t* pUSARTx, uint8_t* pRxBuffer, uint8_t size);


/*
 * IRQ config and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, EUSART_IRQControlState state);

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
