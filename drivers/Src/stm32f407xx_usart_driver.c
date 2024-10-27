/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Oct 16, 2024
 *      Author: karol
 */

#include "stm32f407xx_usart_driver.h"

static USART_Handle_t USART1_Handle 	= {.pUSARTx = USART1};
static USART_Handle_t USART2_Handle 	= {.pUSARTx = USART2};
static USART_Handle_t USART3_Handle 	= {.pUSARTx = USART3};
static USART_Handle_t UART4_Handle 		= {.pUSARTx = UART4};
static USART_Handle_t UART5_Handle 		= {.pUSARTx = UART5};
static USART_Handle_t USART6_Handle 	= {.pUSARTx = USART6};

static USART_Handle_t* getUSART_Handle(USART_RegDef_t* pUSARTx)
{
	if(pUSARTx == USART1)
	{
		return &USART1_Handle;
	}
	else if (pUSARTx == USART2)
	{
		return &USART2_Handle;
	}
	else if (pUSARTx == USART3)
	{
		return &USART3_Handle;
	}
	else if (pUSARTx == UART4)
	{
		return &UART4_Handle;
	}
	else if (pUSARTx == UART5)
	{
		return &UART5_Handle;
	}
	else if (pUSARTx == USART6)
	{
		return &USART6_Handle;
	}
	else
	{
		return NULL;
	}
}

void USART_DeInit(USART_RegDef_t* pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REGISTERS_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REGISTERS_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REGISTERS_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REGISTERS_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REGISTERS_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REGISTERS_RESET();
	}
}

uint32_t USART_GetPclkValueHz(USART_RegDef_t* pUSARTx)
{
	if( (pUSARTx == USART1) || (pUSARTx == USART6) )
	{
		return RCC_GetPCLK2Value();
	}
	else if( (pUSARTx == USART2) || (pUSARTx == USART3) || (pUSARTx == UART4) || (pUSARTx == UART5) )
	{
		return RCC_GetPCLK1Value();
	}
	else
	{
		return 0;
	}
}

static void USART_SetBaudRate(USART_Handle_t* pUSARTHandle)
{
	uint32_t usartPclkHzValue = USART_GetPclkValueHz(pUSARTHandle->pUSARTx);

	uint32_t desiredBaudRate = pUSARTHandle->USART_Config.USART_BaudRate;

	uint32_t oversamplingMode = pUSARTHandle->USART_Config.USART_OversamplingMode;

	uint32_t USART_DIV_Times100 = 100U * usartPclkHzValue/(8U*(2U-oversamplingMode)*desiredBaudRate);

	uint32_t DIV_Mantissa = USART_DIV_Times100/100U;

	uint8_t DIV_Fraction=0;

	if(oversamplingMode == USART_OversamplingMode_16SampelsPerBit)
	{
		DIV_Fraction = ( (USART_DIV_Times100-DIV_Mantissa)*16U+50U )/100U;
		if(DIV_Fraction == 16U)
		{
			DIV_Mantissa= DIV_Mantissa + 1;
			MULTI_BIT_SET_VAL(pUSARTHandle->pUSARTx->BRR,DIV_Mantissa,4,12);
			MULTI_BIT_CLEAR(pUSARTHandle->pUSARTx->BRR,0,4);
		}
		else
		{
			MULTI_BIT_SET_VAL(pUSARTHandle->pUSARTx->BRR,DIV_Mantissa,4,12);
			MULTI_BIT_SET_VAL(pUSARTHandle->pUSARTx->BRR,DIV_Fraction,0,4);
		}
	}
	else if(oversamplingMode == USART_OversamplingMode_8SampelsPerBit)
	{
		DIV_Fraction = ( (USART_DIV_Times100-DIV_Mantissa)*8U+50U )/100U;
		if(DIV_Fraction == 8U)
		{
			DIV_Mantissa= DIV_Mantissa + 1;
			MULTI_BIT_SET_VAL(pUSARTHandle->pUSARTx->BRR,DIV_Mantissa,4,12);
			MULTI_BIT_CLEAR(pUSARTHandle->pUSARTx->BRR,0,4);
		}
		else
		{
			MULTI_BIT_SET_VAL(pUSARTHandle->pUSARTx->BRR,DIV_Mantissa,4,12);
			MULTI_BIT_SET_VAL(pUSARTHandle->pUSARTx->BRR,DIV_Fraction,0,3);
			BIT_CLEAR(pUSARTHandle->pUSARTx->BRR, 3);
		}
	}
}

uint8_t USART_Init(USART_Handle_t* pUSARTHandle)
{
	USART_PeripheralClockControl(pUSARTHandle->pUSARTx, USART_ClockPeriEnable);

	// Mode
	if(pUSARTHandle->USART_Config.USART_Mode == USART_Mode_Tx)
	{
		BIT_SET(pUSARTHandle->pUSARTx->CR1, USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_Mode_Tx)
	{
		BIT_SET(pUSARTHandle->pUSARTx->CR1, USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_Mode_TxRx)
	{
		BIT_SET(pUSARTHandle->pUSARTx->CR1, USART_CR1_TE);
		BIT_SET(pUSARTHandle->pUSARTx->CR1, USART_CR1_RE);
	}

	// Stop bit size
	if( ((pUSARTHandle->pUSARTx == UART4) || (pUSARTHandle->pUSARTx == UART5))
		&&
		((pUSARTHandle->USART_Config.USART_StopBitSize == USART_StopBitSize_0_5)
		|| (pUSARTHandle->USART_Config.USART_StopBitSize == USART_StopBitSize_1_5)) )
	{
		USART_DeInit(pUSARTHandle->pUSARTx);
		USART_PeripheralClockControl(pUSARTHandle->pUSARTx, USART_ClockPeriDisable);
		return 1;
	}


	MULTI_BIT_SET_VAL(pUSARTHandle->pUSARTx->CR2, pUSARTHandle->USART_Config.USART_StopBitSize, USART_CR2_STOP, 2);


	// Parity
	if(pUSARTHandle->USART_Config.USART_ParityControl != USART_Parity_Disable)
	{
		BIT_SET(pUSARTHandle->pUSARTx->CR1, USART_CR1_PCE);

		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_Parity_Odd)
		{
			BIT_SET(pUSARTHandle->pUSARTx->CR1, USART_CR1_PS);
		}
		else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_Parity_Even)
		{
			BIT_CLEAR(pUSARTHandle->pUSARTx->CR1, USART_CR1_PS);
		}

	}

	// Data size
	BIT_SET_VAL(pUSARTHandle->pUSARTx->CR1, pUSARTHandle->USART_Config.USART_DataSize, USART_CR1_M);


	// Hardware flow control
	if( ((pUSARTHandle->pUSARTx == UART4) || (pUSARTHandle->pUSARTx == UART5))
		&&
		(pUSARTHandle->USART_Config.USART_HardwareFlowControl != USART_HardwareFlowControl_None) )
	{
		USART_DeInit(pUSARTHandle->pUSARTx);
		USART_PeripheralClockControl(pUSARTHandle->pUSARTx, USART_ClockPeriDisable);
		return 1;
	}

	if(pUSARTHandle->USART_Config.USART_HardwareFlowControl == USART_HardwareFlowControl_CTS)
	{
		BIT_SET(pUSARTHandle->pUSARTx->CR3, USART_CR3_CTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HardwareFlowControl == USART_HardwareFlowControl_RTS)
	{
		BIT_SET(pUSARTHandle->pUSARTx->CR3, USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config.USART_HardwareFlowControl == USART_HardwareFlowControl_CTS_RTS)
	{
		BIT_SET(pUSARTHandle->pUSARTx->CR3, USART_CR3_CTSE);
		BIT_SET(pUSARTHandle->pUSARTx->CR3, USART_CR3_RTSE);
	}

	// Oversampling metod
	BIT_SET_VAL(pUSARTHandle->pUSARTx->CR1,pUSARTHandle->USART_Config.USART_OversamplingMode,USART_CR1_OVER8);


	USART_SetBaudRate(pUSARTHandle);


	USART_Handle_t* pUSARTx_HandleInternal = getUSART_Handle(pUSARTHandle->pUSARTx);

	pUSARTx_HandleInternal->USART_Config = pUSARTHandle->USART_Config;

	return 0;
}

void USART_WaitUntilTransmitterDataRegisterIsEmpty(USART_RegDef_t* pUSARTx)
{
	while( 0 == BIT_READ(pUSARTx->SR, USART_SR_TXE));
}

void USART_WaitUntilLastByteIsSent(USART_RegDef_t* pUSARTx)
{
	while( 0 == BIT_READ(pUSARTx->SR, USART_SR_TC));
}

void USART_WaitUntilReceiverDataRegisterIsNotEmpty(USART_RegDef_t* pUSARTx)
{
	while( 0 == BIT_READ(pUSARTx->SR, USART_SR_RXNE));
}



void USART_ReceiverControl(USART_RegDef_t* pUSARTx, EUSART_ReceiverControl controlState)
{
	BIT_SET_VAL(pUSARTx->CR1, controlState, USART_CR1_RE);
}

void USART_TransmitterControl(USART_RegDef_t* pUSARTx, EUSART_TransmitterControl controlState)
{
	BIT_SET_VAL(pUSARTx->CR1, controlState, USART_CR1_TE);
}

uint8_t USART_SendData(USART_RegDef_t* pUSARTx, uint8_t* pTxBuffer, uint8_t size)
{
	USART_Handle_t* pUSARTx_Handle = getUSART_Handle(pUSARTx);

	if(pUSARTx_Handle->TxState != USART_TxReady)
	{
		return 1;
	}

	pUSARTx_Handle->TxState = USART_BusyInTx;


	USART_PeripheralControl(pUSARTx, USART_Enable);
	USART_TransmitterControl(pUSARTx, USART_TransmitterEnable);


	while(size > 0)
	{
		USART_WaitUntilTransmitterDataRegisterIsEmpty(pUSARTx);
		// write to Transmit data register
		if((pUSARTx_Handle->USART_Config.USART_DataSize == USART_DataSize_9Bits)
			&& (pUSARTx_Handle->USART_Config.USART_ParityControl == USART_Parity_Disable))
		{
			pUSARTx->DR = *((uint16_t*)pTxBuffer) & (0x1FFU);
			pTxBuffer = pTxBuffer + 2;
			size = size - 2;
		}
		else
		{
			pUSARTx->DR = *pTxBuffer;
			pTxBuffer++;
			size--;
		}
	}

	USART_WaitUntilLastByteIsSent(pUSARTx);


	USART_TransmitterControl(pUSARTx, USART_TransmitterDisable);
	USART_PeripheralControl(pUSARTx, USART_Disable);

	pUSARTx_Handle->TxState = USART_TxReady;

	return 0;
}

uint8_t USART_ReceiveData(USART_RegDef_t* pUSARTx, uint8_t* pRxBuffer, uint8_t size)
{
	USART_Handle_t* pUSARTx_Handle = getUSART_Handle(pUSARTx);

	if(pUSARTx_Handle->RxState != USART_RxReady)
	{
		return 1;
	}

	pUSARTx_Handle->RxState = USART_BusyInRx;


	USART_PeripheralControl(pUSARTx, USART_Enable);
	USART_ReceiverControl(pUSARTx, USART_ReceiverEnable);


	while(size > 0)
	{
		USART_WaitUntilReceiverDataRegisterIsNotEmpty(pUSARTx);
		// read from Receive data register
		if((pUSARTx_Handle->USART_Config.USART_DataSize == USART_DataSize_9Bits)
			&& (pUSARTx_Handle->USART_Config.USART_ParityControl == USART_Parity_Disable))
		{
			*pRxBuffer = (uint16_t)((pUSARTx->DR) & (0x1FFU));
			pRxBuffer = pRxBuffer + 2;
			size = size - 2;
		}
		else
		{
			*pRxBuffer = (uint8_t)(pUSARTx->DR);
			pRxBuffer++;
			size--;
		}
	}


	USART_ReceiverControl(pUSARTx, USART_ReceiverDisable);
	USART_PeripheralControl(pUSARTx, USART_Disable);

	pUSARTx_Handle->RxState = USART_RxReady;

	return 0;
}

static void USART_EnableEventsIT(USART_RegDef_t* pUSARTx)
{
	BIT_SET(pUSARTx->CR1, USART_CR1_TXEIE);
	BIT_SET(pUSARTx->CR3, USART_CR3_CTSIE);
	BIT_SET(pUSARTx->CR1, USART_CR1_TCIE);
	BIT_SET(pUSARTx->CR1, USART_CR1_RXNEIE);
	BIT_SET(pUSARTx->CR1, USART_CR1_IDLEIE);
	BIT_SET(pUSARTx->CR1, USART_CR1_PEIE);
	BIT_SET(pUSARTx->CR2, USART_CR2_LBDIE);
	BIT_SET(pUSARTx->CR3, USART_CR3_EIE);
}

static void USART_DisableEventsIT(USART_RegDef_t* pUSARTx)
{
	BIT_CLEAR(pUSARTx->CR1, USART_CR1_TXEIE);
	BIT_CLEAR(pUSARTx->CR3, USART_CR3_CTSIE);
	BIT_CLEAR(pUSARTx->CR1, USART_CR1_TCIE);
	BIT_CLEAR(pUSARTx->CR1, USART_CR1_RXNEIE);
	BIT_CLEAR(pUSARTx->CR1, USART_CR1_IDLEIE);
	BIT_CLEAR(pUSARTx->CR1, USART_CR1_PEIE);
	BIT_CLEAR(pUSARTx->CR2, USART_CR2_LBDIE);
	BIT_CLEAR(pUSARTx->CR3, USART_CR3_EIE);
}

uint8_t USART_SendDataIT(USART_RegDef_t* pUSARTx, uint8_t* pTxBuffer, uint8_t size)
{
	USART_Handle_t* pUSARTx_Handle = getUSART_Handle(pUSARTx);

	if(pUSARTx_Handle->TxState != USART_TxReady)
	{
		return 1;
	}


	pUSARTx_Handle->TxSize = size;
	pUSARTx_Handle->pTxBuffer = pTxBuffer;

	pUSARTx_Handle->TxState = USART_BusyInTx;


	USART_PeripheralControl(pUSARTx, USART_Enable);
	USART_EnableEventsIT(pUSARTx);
	USART_TransmitterControl(pUSARTx, USART_TransmitterEnable);

	return 0;
}

uint8_t USART_ReceiveDataIT(USART_RegDef_t* pUSARTx, uint8_t* pRxBuffer, uint8_t size)
{
	USART_Handle_t* pUSARTx_Handle = getUSART_Handle(pUSARTx);

	if(pUSARTx_Handle->RxState != USART_RxReady)
	{
		return 1;
	}


	pUSARTx_Handle->RxSize = size;
	pUSARTx_Handle->pRxBuffer = pRxBuffer;

	pUSARTx_Handle->RxState = USART_BusyInRx;


	USART_PeripheralControl(pUSARTx, USART_Enable);
	USART_EnableEventsIT(pUSARTx);
	USART_ReceiverControl(pUSARTx, USART_ReceiverEnable);

	return 0;
}

EUSART_IT_Flag USART_GetActiveFlag(USART_RegDef_t* pUSARTx)
{
	if( (BIT_READ(pUSARTx->SR, USART_SR_TXE) == 1) && (BIT_READ(pUSARTx->CR1, USART_CR1_TXEIE)) )
	{
		return USART_IT_EVENT_Transmit_Data_Register_Empty;
	}
	else if( (BIT_READ(pUSARTx->SR, USART_SR_CTS) == 1) && (BIT_READ(pUSARTx->CR3, USART_CR3_CTSIE)) )
	{
		return USART_IT_EVENT_CTS_Flag;
	}
	else if( (BIT_READ(pUSARTx->SR, USART_SR_TC) == 1) && (BIT_READ(pUSARTx->CR1, USART_CR1_TCIE)) )
	{
		return USART_IT_EVENT_Transmission_Complete;
	}
	else if( (BIT_READ(pUSARTx->SR, USART_SR_RXNE) == 1) && (BIT_READ(pUSARTx->CR1, USART_CR1_RXNEIE)) )
	{
		return USART_IT_EVENT_Received_Data_Ready_To_Be_Read;
	}
	else if( (BIT_READ(pUSARTx->SR, USART_SR_ORE) == 1) && (BIT_READ(pUSARTx->CR1, USART_CR1_RXNEIE)) )
	{
		return USART_IT_EVENT_Overrun_Error_Detected;
	}
	else if( (BIT_READ(pUSARTx->SR, USART_SR_IDLE) == 1) && (BIT_READ(pUSARTx->CR1, USART_CR1_IDLEIE)) )
	{
		return USART_IT_EVENT_Idle_Line_Detected;
	}
	else if( (BIT_READ(pUSARTx->SR, USART_SR_PE) == 1) && (BIT_READ(pUSARTx->CR1, USART_CR1_PEIE)) )
	{
		return USART_IT_EVENT_Parity_Error;
	}
	else if( (BIT_READ(pUSARTx->SR, USART_SR_LBD) == 1) && (BIT_READ(pUSARTx->CR2, USART_CR2_LBDIE)) )
	{
		return USART_IT_EVENT_LIN_Break_Flag;
	}
	else if( ((BIT_READ(pUSARTx->SR, USART_SR_NF) == 1)
			|| (BIT_READ(pUSARTx->SR, USART_SR_ORE) == 1)
			|| (BIT_READ(pUSARTx->SR, USART_SR_FE) == 1)) && (BIT_READ(pUSARTx->CR3, USART_CR3_EIE)) )
	{
		return USART_IT_EVENT_Noise_Flag_Overrun_Error_And_Framing_Error;
	}


	return USART_IT_Unknown;
}

void USART_ProcessFlag(USART_Handle_t* pUSARTHandle, EUSART_IT_Flag activeITFlag)
{
	switch(activeITFlag)
	{
		case USART_IT_EVENT_Transmit_Data_Register_Empty:
		{
			if(pUSARTHandle->TxSize > 0)
			{
				if((pUSARTHandle->USART_Config.USART_DataSize == USART_DataSize_9Bits)
					&& (pUSARTHandle->USART_Config.USART_ParityControl == USART_Parity_Disable))
				{
					pUSARTHandle->pUSARTx->DR = *((uint16_t*)pUSARTHandle->pTxBuffer) & (0x1FFU);
					pUSARTHandle->pTxBuffer = pUSARTHandle->pTxBuffer + 2;
					pUSARTHandle->TxSize = pUSARTHandle->TxSize - 2;
				}
				else
				{
					pUSARTHandle->pUSARTx->DR = *(pUSARTHandle->pTxBuffer);
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxSize--;
				}
			}
			else
			{
				BIT_CLEAR(pUSARTHandle->pUSARTx->CR1, USART_CR1_TXEIE);
			}
			break;
		}
		case USART_IT_EVENT_CTS_Flag:
		{

			break;
		}
		case USART_IT_EVENT_Transmission_Complete:
		{
			if(pUSARTHandle->TxSize == 0)
			{
				USART_TransmitterControl(pUSARTHandle->pUSARTx, USART_TransmitterDisable);
				USART_DisableEventsIT(pUSARTHandle->pUSARTx);
				USART_PeripheralControl(pUSARTHandle->pUSARTx, USART_Disable);
				pUSARTHandle->TxState = USART_TxReady;
			}
			break;
		}
		case USART_IT_EVENT_Received_Data_Ready_To_Be_Read:
		{
			if(pUSARTHandle->RxSize > 0)
			{
				if((pUSARTHandle->USART_Config.USART_DataSize == USART_DataSize_9Bits)
					&& (pUSARTHandle->USART_Config.USART_ParityControl == USART_Parity_Disable))
				{
					*pUSARTHandle->pRxBuffer = (uint16_t)((pUSARTHandle->pUSARTx->DR) & (0x1FFU));
					pUSARTHandle->pRxBuffer = pUSARTHandle->pRxBuffer + 2;
					pUSARTHandle->RxSize = pUSARTHandle->RxSize - 2;
				}
				else
				{
					*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR);
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxSize--;
				}
			}
			else if(pUSARTHandle->RxSize == 0)
			{
				USART_ReceiverControl(pUSARTHandle->pUSARTx, USART_ReceiverDisable);
				USART_DisableEventsIT(pUSARTHandle->pUSARTx);
				USART_PeripheralControl(pUSARTHandle->pUSARTx, USART_Disable);

				pUSARTHandle->RxState = USART_RxReady;
			}
			break;
		}
		case USART_IT_EVENT_Overrun_Error_Detected:
		{

			break;
		}
		case USART_IT_EVENT_Idle_Line_Detected:
		{

			break;
		}
		case USART_IT_EVENT_Parity_Error:
		{

			break;
		}
		case USART_IT_EVENT_LIN_Break_Flag:
		{

			break;
		}
		case USART_IT_EVENT_Noise_Flag_Overrun_Error_And_Framing_Error:
		{

			break;
		}
        default:
        {
            break;
        }
	}
}

void USART_IRQHandling(USART_Handle_t* pUSARTHandle)
{
	EUSART_IT_Flag activeITFlag = USART_GetActiveFlag(pUSARTHandle->pUSARTx);

	USART_ProcessFlag(pUSARTHandle, activeITFlag);
}

void USART1_IRQHandler()
{
	USART_IRQHandling(&USART1_Handle);
}

void USART2_IRQHandler()
{
	USART_IRQHandling(&USART2_Handle);
}

void USART3_IRQHandler()
{
	USART_IRQHandling(&USART3_Handle);
}

void UART4_IRQHandler()
{
	USART_IRQHandling(&UART4_Handle);
}

void UART5_IRQHandler()
{
	USART_IRQHandling(&UART5_Handle);
}

void USART6_IRQHandler()
{
	USART_IRQHandling(&USART6_Handle);
}

void USART_PeripheralClockControl(USART_RegDef_t* pUSARTx, EUSART_ClockPeriControlState clockControlState)
{
	if(clockControlState == USART_ClockPeriEnable)
	{
		if(pUSARTx == USART1)
		{
			USART1_PERI_CLOCK_EN();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PERI_CLOCK_EN();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PERI_CLOCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PERI_CLOCK_EN();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PERI_CLOCK_EN();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PERI_CLOCK_EN();
		}
	}
	else if(clockControlState == USART_ClockPeriDisable)
	{
		if(pUSARTx == USART1)
		{
			USART1_PERI_CLOCK_DIS();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PERI_CLOCK_DIS();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PERI_CLOCK_DIS();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PERI_CLOCK_DIS();
		}
		else if (pUSARTx == UART5)
		{
			UART5_PERI_CLOCK_DIS();
		}
		else if (pUSARTx == USART6)
		{
			USART6_PERI_CLOCK_DIS();
		}
	}
}

void USART_PeripheralControl(USART_RegDef_t* pUSARTx, EUSART_PeriControlState controlState)
{
	BIT_SET_VAL(pUSARTx->CR1, controlState, USART_CR1_UE);
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, EUSART_IRQControlState state)
{
	if(state == USART_IRQ_Enable)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else if(state == USART_IRQ_Disable)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= 1 << IRQNumber;
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= 1 << IRQNumber % 32;
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= 1 << IRQNumber % 64;
		}
	}
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t nvicIprRegNum = IRQNumber / 4;
	uint8_t nvicIprRegByteNum = IRQNumber % 4;

	uint8_t shiftValue = nvicIprRegByteNum * 8 +( 8 - NO_PR_BITS_IMPLEMENTED);
	// clear previous priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] &= ~(0xF << shiftValue);
	// set new priority
	NVIC_PR_BASE_ADDR[nvicIprRegNum] |= (IRQPriority << shiftValue);
}
