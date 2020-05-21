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
		// Mask out word length (M1) Bit-field
		tempreg|= pUSARTHandle->USART_Config.USART_Wordlength << USART_CR1_M1;

		// Set World length M1 Bit-field
		pUSARTHandle->pUSARTx->CR1 = tempreg;

	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN);
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

//ToDo Enable Baudrate


}


USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			//USART1_PCCK_EN();
		}
		else if(pUSARTx == USART2)
		{
			//USART2_PCCK_EN();
		}
		else if(pUSARTx == USART3)
		{
			//USART3_PCCK_EN();
		}
		else if(pUSARTx == UART4)
		{
			//UART4_PCCK_EN();
		}
		else if(pUSARTx == UART5)
		{
			//UART5_PCCK_EN();
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
