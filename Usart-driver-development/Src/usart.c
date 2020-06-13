/*
 * usart.c
 *
 *  Created on: May 16, 2020
 *      Author: T2
 */
#include <usart.h>
#include <stm32f303xx.h>

void USART_Init(USART_Handle_t *pUSARTHandle){

/*
 Configuration of CR1-Register
 */

	// temporary variable
	uint32_t tempreg = 0;
	// Implement the code to enable the clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and Rx engines according to USART Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		// Mask out Read Enable (RE) Bit-field
		tempreg |= (1 << USART_CR1_RE);
		// Set Read Enable Bit (RE)
		pUSARTHandle->pUSARTx->CR1 = tempreg;
	}

	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		// Mask out Transmit Enable (TE) Bit-field
		tempreg |= (1 << USART_CR1_TE);
		// Set Transmit Enable (TE) Bit-field
		pUSARTHandle->pUSARTx->CR1 = tempreg;
	}

	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		// Not available for stm32f303
	}

		//	Implement the code to enable the Word length configuration item
		// Mask out word length (M0) Bit-field
		tempreg|= pUSARTHandle->USART_Config.USART_Wordlength << USART_CR1_M0;

		// Set World length M0 Bit-field
		pUSARTHandle->pUSARTx->CR1 = tempreg;

	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		// Mask out Parity Control Enable (PCE)Bit-field
		tempreg |= (1 << USART_CR1_PCE);
		// Mask out Parity Selection (PS) Bit-field

		// Set EVEN-Parity
		tempreg |= (1 << USART_CR1_PS);
		pUSARTHandle->pUSARTx->CR1 = tempreg;

	}


/*
 Configuration of Control Register 2 (CR2)
 */

	tempreg = 0;
	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
	pUSARTHandle->pUSARTx->CR3 = tempreg;




/*
 Configuration of Control Register 3 (CR3)
 */

	//Configuration of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// Mask out CTSE Bit-field
		tempreg |= pUSARTHandle->USART_Config.USART_HWFlowControl << USART_CR3_CTSE;

		// Set CTSE-Bitfield
		pUSARTHandle->pUSARTx->CR3 = tempreg;
	}
	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// Mask out RTSE Bit-field
		tempreg |= pUSARTHandle->USART_Config.USART_HWFlowControl << USART_CR3_RTSE;

		// Set RTSE-Bitfield
		pUSARTHandle->pUSARTx->CR3 = tempreg;
	}
}


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCCK_EN();
		}
		else
		{
         //ToDO disable clocks
		}

	}
	else
	{
		// ToDO Disable Clocks
	}

}


// Data, Send and Receive
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pData;

	for(uint32_t i = 0;i < Len; i++)
	{
		// implement the code to wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		if(pUSARTHandle->USART_Config.USART_Wordlength == USART_WORDLEN_9BITS)
		{
			// if 9BIT, load the DR with 2bytes masking the bits other  that first 9 bits
			pData = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = *pData & 0x01FF;

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
			{
				// No Parity is used in this transfer, so 9Bits of data will be sent
				pTxBuffer++;
				pTxBuffer++;

			}
			else
			{
				/* Parity bit is used in this transfer, so 8 Bits of user data will be sent
                   The 9th Bit will be replaced by parity bit by the hardware
				 */
				pTxBuffer++;
			}
		}

		else
		{
			// Masking out 8 Bits
			pUSARTHandle->pUSARTx->TDR = *pTxBuffer & 0x00FF;
			// increment the buffer adress
			pTxBuffer++;
		}

//Implement the code to wait till TC flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
	}

}



void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	// Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++)
	{
	// Implement the code to wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

	// Check the USART Wordlength to decide whether we are going to receive 9bit of data in a frame or 8 Bit
		if(pUSARTHandle->USART_Config.USART_Wordlength == USART_WORDLEN_9BITS)
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
			{
				// No Parity is used, all 9 bits will be masked
				*(uint16_t*)pRxBuffer =  pUSARTHandle->pUSARTx->RDR & (uint16_t)0x1FF;
				pRxBuffer++;
				pRxBuffer++;

				// Sollte nicht 8 bit pointer sein und deshalb zwei incrementieren?
			}

		}

		else
		{
			*pRxBuffer = pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF;
			pRxBuffer++;

		}

	};

}





uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
			if(FlagName == USART_FLAG_TXE)
			{
				// Set Transmission register empty flag
				pUSARTx->ISR |= (1<<USART_FLAG_TXE);
				return 1;
			}

			else if(FlagName == USART_FLAG_TC)
			{
				// Set Transmission complete flag
				pUSARTx->ISR |= (1<<USART_FLAG_TC);
				return 1;
			}

			else if(FlagName == USART_FLAG_RXE)
			{
				// Set Receive data register empty flag
				pUSARTx->ISR |= (1<<USART_FLAG_RXE);
			}

			else
			{
				return 0;
			}
}


void USART_SetBaudRate(USART_RegDef_t *pUSARTx,uint32_t BaudRate)
{
	// Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	// Variables to hold Manitssa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempred = 0;

	if(pUSARTx == USART1)
	{
		// Usart1 hanging on PCLK1
		PCLKx = RCC_GetPCLK1Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK2Value();
	}

	// Check for OVER8 = 1
	if(pUSARTx->CR1 & (1 << TODO))
	{
		//OVER8 = 1, over sampling by 8, dealing with integer
		usartdiv = 100 * PCLKx/(8 * Baudrate);
	}
	else
	{
		// OVER8 = 0, over sampling by 16
		usartdiv = 100 * PCLKx/(16 * Baudrate);

	}

	// Calculate the Mantissa part
	M_part = usartdiv/100;

	// Place the Mantissa part in appropriate bit position, refer USART_BRR [15:4] Bit-fields
	tempreg |= M_part << 4;

	// Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	// calculate the final fractional
	if(pUSARTx->CR1 & (1<< USART_CR1_OVER8))
	{
		//Over8 = 1, over sampling by 8
		F_part = ( ( (F_part * TODO) + 50 ) / 100 ) & ((uint8_t)0x07);
	}
	else
	{
		//over sampling by 16
		F_part = ( ( (F_part * TODO) + 50 ) / 100 ) & ((uint8_t)0x07);
	}

	// place the fractional part in appropriate bit position, refer USART_BRR
	tempreg |= F_part;
	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;


}








