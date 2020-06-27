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

	uint8_t GPIO_PinNumber;				/* Possible values from @GPIO_PIN_NUMBER */
	uint8_t GPIO_PinMode;				/* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;				/* Possible values from @GPIO_PIN_SPEEDS */
	uint8_t GPIO_PinPuPdControl;		/* Possible values from @GPIO_PIN_PuPd */
	uint8_t GPIO_PinOPType;				/* Possible values from @GPIO_PIN_OPType */
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT	 	1
#define GPIO_MODE_ALTFCT	2
#define GPIO_MODE_ANALOG	3

// Interrupt falling edge and risiing edge and both
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OPType
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output types
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PuPd
 * GPIO pin possible pull up and pull down configuration Macros
 */

#define GPIO_NO_PUPD		0
#define GPIO_PU			    1
#define GPIO_PD			    2

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin number configuration Macros
 */


#define GPIO_PIN0			0
#define GPIO_PIN1			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10			10
#define GPIO_PIN11			11
#define GPIO_PIN12			12
#define GPIO_PIN13			13
#define GPIO_PIN14			14
#define GPIO_PIN15			15

// Stm32303RE specific
#define NO_PR_BITS_IMPLEMENTED	4





typedef struct
{
	// pointer to hold the baseadrres of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; // holds the base adress of the GPIO port to which the pin belongs
    GPIO_PinConfig GPIO_PinConfig;

}GPIO_Handle_t;






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

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

// Convert GPIO_BASEADDR to Code number in order to write into EXTI Register
u_int8_t GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *pGPIOx);

#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */




