/*
 * stm32f303xxgpio_driver.c
 *
 *  Created on: 13 Jun 2020
 *      Author: T2
 */


#include <stm32f303xx_gpio_driver.h>




// Application Programming Interface (API)


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

// Peripheral Clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx== GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else
		{

			// ToDO disable Clk
		}

	}

}



/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes GPIO Peripheral.
 *
 * @param[in]         - base address of Handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

// Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temp register
	// Configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{


		// The two because of 2 Bit-field is used to configure the Pin Mode MODER[1:0] for gpio pin 0 left shiftet gpio pin 1 and so on...
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// Clear Bit-fields for gpios before setting Bit-fields
		//pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// Set Bit-fields

		pGPIOHandle->pGPIOx->MODER |= temp;



	}

	else
	{
		// TODO Interrupt Mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the FTSR
			EXTI->FTSR1 |= 1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the RTSR
			EXTI->RTSR1 |= 1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure both FTSR and RTSR
			EXTI->FTSR1 |= 1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= 1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// 2. Configure the GPIO port selection SYSCFG_EXTICR

		// Calculate which EXTICR Register (1...4) needs to be set
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		// Calculate which EXTICR Register Position needs to be set
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		// Enable SYSCFG Clk
		SYSCFG_PCLK_EN();
		// Set external interupt control register for specific Port A....H and GPIO Pinnumber
		SYSCFG->EXTICR[temp1] |= portcode << 4 * temp2;

		// 3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR1 |= 1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

//
//	// Configure the speed


	 temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// Clear Bit-fields for gpios before setting Bit-fields
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// Set Bitfields

	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// Configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// Configure the optype
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFCT)
	{
		// Configure alternate function register
		uint32_t temp1, temp2;

		// calculate whether to use AFR[0] or AFR[1]
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/ 8;
		// calculate with Pin
		temp2 =  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		// times 4 because 4 Bit-field for alternate function mode
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));


	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initializes GPIO Peripheral.
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

// Can be take care by the RCC AHB1 peripheral reset register for instance GPIOA RST, must not be set individually
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if(pGPIOx== GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else
			{
				// ToDO disable Clk
			}

}




/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads data from input.
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - PinNumber
 *
 * @return            -  boolean (uint8_t), zero or one
 *
 * @Note              -  none
 */

// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t )((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;

}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads Input Port.
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         -
 *
 * @return            -  uint16_t
 *
 * @Note              -  none
 */

// How long is a Port? 16 Pins
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}



/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes value to Pin.
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - PinNumber
 * @param[in]         - Value
 *
 * @return            -
 *
 * @Note              -  none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// Write 1 to the output register at the bit filed corresponding to the pin Number
		pGPIOx->ODR |= (1 << PinNumber);

	}
	else
	{
		// Write 0 to the output register at the bit filed corresponding to the pin Number
		pGPIOx->ODR &= (1 << PinNumber);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes value to Port.
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - Value
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	// Put in whole in the entire Port register
	pGPIOx->ODR = Value;

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles gpio pin.
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - pin number
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  none
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	// Toggle pin at the corresponding Pin number
	pGPIOx->ODR ^= (1 << PinNumber);

}

// IRQ Configuration and ISR handling

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - This function configures interupt.
 *
 * @param[in]         - IRQnumber
 * @param[in]         - IRQPriority
 * @param[in]         - EnorDi
 *
 * @return            -
 *
 * @Note              -  none
 */


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{

		/*
		Each register is 32 Bit wide differentiate which ISER Register should written
		*/

		if(IRQNumber <=31)
		{


			// program  ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// program  ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if((IRQNumber >= 64) && (IRQNumber < 96) )
		{
			// program  ISER1 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
			// program  ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// program  ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// program  ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}

}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function handles IRQ.
 *
 * @param[in]         - Pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Handling IRQ
		// Clear the exti pr register corresponding to the pin number

		if(EXTI->PR1 & (1 << PinNumber))
		{
			// Clear
			EXTI->PR1 |= (1 << PinNumber);
		}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	/*
	 32 Register is divided by 4 sections
	 */

	// Find out IPR Register
	uint8_t iPRx_offset = IRQNumber/4;

	// Find section
	uint8_t iPRx_section = IRQNumber % 4;

	/* times 4 registers are 32 Bits next adress would be + 4 Bytes, times 8 section are 8 Bits wide

	*/

// Some uC only the upper four bits are used in the 8 Bitfield, the shiftamount is MCU specific
	uint8_t shiftamount = 8 * iPRx_section + (8 - NO_PR_BITS_IMPLEMENTED);

	// NVIC base adress is a pointer of 32 bit that why the for needs
	*(NVIC_IPR_BASE_ADDR + iPRx_offset ) |= (IRQPriority << shiftamount);
}

u_int8_t GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *pGPIOx)
{
	uint8_t temp;
	if(pGPIOx == GPIOA)
	{
		temp = 0;
		return temp;
	}
	else if(pGPIOx == GPIOB)
	{
		temp = 1;
		return temp;
	}
	else if(pGPIOx == GPIOC)
	{
		temp = 2;
		return temp;
	}
	else if(pGPIOx == GPIOD)
	{
		temp = 3;
		return temp;
	}
	else if(pGPIOx == GPIOE)
	{
		temp = 4;
		return temp;
	}
	else if(pGPIOx == GPIOF)
	{
		temp = 5;
		return temp;
	}
	else if(pGPIOx== GPIOG)
	{
		temp = 6;
		return temp;
	}
	else if(pGPIOx == GPIOH)
	{
		temp = 7;
		return temp;
	}
	else
	{

		// No such Port
		return -1;
	}

}
