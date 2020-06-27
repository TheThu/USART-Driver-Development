/*
 * usart.h
 *
 *  Created on: May 15, 2020
 *      Author: T2
 */

#ifndef USART_H_
#define USART_H_

#include <stdint.h>
#include <stm32f303xx.h>
#include <stdbool.h>
#include <stdio.h>





// USART_Mode

#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX 2


// possible options for USART_Baud

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200			    19200
#define USART_STD_BAUD_38400			    38400
#define USART_STD_BAUD_57600			    57600
#define USART_STD_BAUD_115200			    115200
#define USART_STD_BAUD_230400			    230400
#define USART_STD_BAUD_460800			    460800
#define USART_STD_BAUD_921600			    921600
#define USART_STD_BAUD_2M			    	2000000
#define USART_STD_BAUD_3M			    	3000000



// possible options for USART_NoOfStopBits
#define USART_STOPBITS_1					0
#define USART_STOPBITS_0_5					1
#define USART_STOPBITS_2					2
#define USART_STOPBITS_1_5					3


// possible options for USART_Wordlength
#define USART_WORDLEN_8BITS					0
#define USART_WORDLEN_9BITS					1

// possible options for USART_ParityControl
#define USART_PARITY_EN_ODD 				2
#define USART_PARITY_EN_EVEN 				1
#define USART_PARITY_EN_DISABLE 			0


// possible options for USART_HWFlowControl
#define USART_HW_FLOW_CTRL_NONE				0
#define USART_HW_FLOW_CTRL_CTS				1
#define USART_HW_FLOW_CTRL_RTS				2
#define USART_HW_FLOW_CTRL_CTS_RTS		    3


// Macros for Register Bit-fields for CR1-Register
#define USART_CR1_RE						2 	// Read Enable Bit-field
#define USART_CR1_TE						3 	// Transmit Enable Bit-field
#define USART_CR1_M0						12  // Wordlength Bit-field, M0= 0 1 Startbits 8 data bits , 1 Startbits 9 data bits...
#define USART_CR1_PS						9   // Parity Selection Bit-field
#define USART_CR1_PCE						10
#define USART_CR1_PS						9   // Parity Selection Bit-field
#define USART_CR1_OVER8 					15  // Oversampling Bit field

// Macros for Register Bit-fields for CR2-Register
#define USART_CR2_STOP						12 	// Stop bits

// Macros for Register Bit-fields for CR3-Register
#define USART_CR3_CTSE						8 	// CTS mode enabled, data is only transmitted when the CTS input is asserted (tied to 0). Ifthe CTS input is de-asserted while data is being transmitted, then the transmission iscompleted before stopping. If data is written into the data
#define USART_CR3_RTSE						9 	// RTS Enable RTS output enabled, data is only requested when there is space in the receive buffer. The transmission of data is expected to cease after the current character has been transmitted.The RTS output is asserted (pulled to 0) when data can be received.

// Flags for Transmission and Reception
#define USART_FLAG_TC						6   // USART Transmission complete flag
#define USART_FLAG_TXE						7	// USART Transmit data register empty flag
#define USART_FLAG_RXE					    5   // USART Read data register not empty






// Configuration for USARTx peripheral
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_Wordlength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*  Handle structure for USARTx peripheral
 */

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t	USART_Config;
}USART_Handle_t;

// API supported by this driver



// Peripheral Clock setup
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

// Calculate Baudrate

void USART_SetBaudRate(USART_RegDef_t *pUSARTx,uint32_t BaudRate);


// UART2 GPIO Init
void USART2_GPIOInit(void);

// Initilize USART2 specific parameters
void USART2_Init();


// Init and DeInit

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


// Data, Send and Receive
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

// IRq Configuration and ISR Handling
void USART_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// Other Peripheral Control APIs
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

//  Application Callback
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);


#endif /* USART_H_ */
