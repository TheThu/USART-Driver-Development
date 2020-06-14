/*
 * stm32f303xx_gpio_driver.h
 *
 *  Created on: 13 Jun 2020
 *      Author: T2
 */

#ifndef INC_STM32F303XX_GPIO_DRIVER_H_
#define INC_STM32F303XX_GPIO_DRIVER_H_

#include <stm32f303xx.h>

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PonOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig;


typedef struct
{
	// pointer to hold the baseadrres of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; // holds the base adress of the GPIO port to which the pin belongs
    GPIO_PinConfig GPIO_PinConfig;
}GPIO_Handle_t;

#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */




// Application Programming Interface (API)

// Peripheral Clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
// RCC AHB1 peripheral reset register for instance GPIOA RST, must not be set individually
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);




// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
// How long is a Port? 16 Pins
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ Configuration and ISR handling

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);







