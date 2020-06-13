/*
 * stm32f303xx_rcc_driver.c
 *
 *  Created on: 7 Jun 2020
 *      Author: T2
 */


#include <stm32f303xx_rcc_driver.h>

uint16_t AHB_Prescaler[8] = {1, 4 ,8 , 16, 64, 128, 256, 512};
uint16_t AHB1_Prescaler[4] = {1, 4 , 8 , 16};
uint16_t AHB2_Prescaler[4] = {1, 4 , 8 , 16};


uint32_t RCC_GetPCLK1Value(void)
{

	uint32_t pclk1, SystemClk, temp;
	uint32_t ahb_prescaler;
	uint32_t apb1_prescaler;
	uint8_t clksrc;

	// masking Bit-field SWS: System clock switch status
	clksrc = ( RCC->CFGR >> 2 )  &  0x3;



	if(clksrc == 0)
	{

		// HSI is activated
		SystemClk  = 8000000;
	}

	else if(clksrc == 1)
	{
		// No HSE
	}
	else if(clksrc == 2)
	{
		// PLL mode enabled
		// SystemClk = RCC_GetPPLLOutputClock();
	}
	// mask Bit-field HPRE: HLCK prescaler
    temp = (RCC->CFGR >> 4)& 0xF;

    if(temp < 8)
    {
    	ahb_prescaler = 1;
    }
    else
    {
    	ahb_prescaler = AHB_Prescaler[temp-8];

    }

    // mask Bit-field PPRE1: APB Low-speed prescaler
    temp = (RCC->CFGR >>8) & 0x7;

    if(temp < 4)
    {
    	apb1_prescaler = 1;
    }

    else
    {
        apb1_prescaler = AHB1_Prescaler[temp-4];
    }

    // Calculate Peripheral Clock 1
    pclk1 = (SystemClk/ahb_prescaler) / (apb1_prescaler);

    return pclk1;


}




uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk, temp;
	uint32_t ahb_prescaler;
	uint32_t ahb2_prescaler;
	uint8_t clksrc;

	// masking Bit-field SWS: System clock switch status
	clksrc = ( RCC->CFGR >> 2 )  &  0x3;

	if(clksrc == 0)
		{

			// HSI is activated
			SystemClk  = 8000000;
		}

		else if(clksrc == 1)
		{
			// No HSE
		}
		else if(clksrc == 2)
		{
			// PLL mode enabled
			// SystemClk = RCC_GetPPLLOutputClock();
		}
	// mask Bit-field HPRE: HLCK prescaler
	temp = (RCC->CFGR >> 4)& 0xF;
	if(temp < 8)
	{
		ahb_prescaler = 1;
	}
	else
	{
		ahb_prescaler = AHB_Prescaler[temp - 8];
	}
	temp = (RCC->CFGR >> 11 ) & 0x3;

	if(temp < 4)
	{
		ahb2_prescaler = 1;
	}

	else
	{
		ahb2_prescaler = AHB2_Prescaler[temp - 4];

	}

	pclk2 = (SystemClk/ahb_prescaler)/ ahb2_prescaler;

	return pclk2;

}
