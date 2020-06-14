/*
 * stm32f303xxgpio_driver.c
 *
 *  Created on: 13 Jun 2020
 *      Author: T2
 */


#include <stm32f303xx_gpio_driver.h>
#include <stm32f303xx_rcc_driver.h>
#include <stm32f303xx.h>



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
	uint32_t temp; // temp register
	// Configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{


		// The two because of 2 Bit-field is used to configure the Pin Mode MODER[1:0] for gpio pin 0 left shiftet gpio pin 1 and so on...
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		// Clear Bit-fields for gpios before setting Bit-fields
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// Set Bit-fields
		pGPIOHandle->pGPIOx->MODER = temp;

	}
	else
	{
		// TODO Interupt Mode
	}


	// Configure the speed
	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <= (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// Clear Bit-fields for gpios before setting Bit-fields
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// Set Bitfields
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// Configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <= (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// Configure the optype
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
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
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
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
 * @return            -  boolean (uint8_t)
 *
 * @Note              -  none
 */

// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


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
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);



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

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);


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

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

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

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

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


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);


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
void GPIO_IRQHandling(uint8_t PinNumber);
