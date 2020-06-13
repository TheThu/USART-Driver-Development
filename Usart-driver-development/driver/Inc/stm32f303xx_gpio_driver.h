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
void GPIO_Init(void);
void GPIO_DeInit(void);
void GPIO_PeriClockControl(void);
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

// IRQ Configuration and ISR handling

void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);







