/*
 * stm32f303xx_rcc_driver.h
 *
 *  Created on: 7 Jun 2020
 *      Author: T2
 */

#ifndef INC_STM32F303XX_RCC_DRIVER_H_
#define INC_STM32F303XX_RCC_DRIVER_H_

#include <stm32f303xx.h>



uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F303XX_RCC_DRIVER_H_ */
