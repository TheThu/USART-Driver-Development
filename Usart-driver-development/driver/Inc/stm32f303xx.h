/*
 * stm32f303xx.h
 *
 *  Created on: May 15, 2020
 *      Author: T2
 */

#ifndef STM32F303XX_H_
#define STM32F303XX_H_
/*
 * stm32f303xx.h
 *
 *  Created on: May 15, 2020
 *      Author: T2
 */


#include <stdint.h>
#include <stdio.h>












/*

 base adresses of Flash and SRAM memories according reference manual

 */



#define FLASH_BASEADDR 						0x08000000U
#define SRAM								0x20000000U
#define ROM 								0x1FFFD800U


/*
  AHBx and ApBx Bus Peripheral base adresses
 */


#define PERIPH_BASE							0x40000000U  // U stands for unsigned
#define APB1PERIPH_BASE						PERIPH_BASE
#define APB2PERIPH_BASE						0x40010000U
#define AHB1PERIPH_BASE					    0x40020000U
#define AHB2PERIPH_BASE						0x48000000U

#define AHB3PERIPH_BASE						0x50000000U
#define AHB4PERIPH_BASE						0x60000000U



/*
 Base adresses of peripherals which are hanging on AHB2
 */

#define GPIOA_BASEADDR					    AHB2PERIPH_BASE + 0X0000
#define GPIOB_BASEADDR					    AHB2PERIPH_BASE + 0X0400
#define GPIOC_BASEADDR					    AHB2PERIPH_BASE + 0X0800
#define GPIOD_BASEADDR					    AHB2PERIPH_BASE + 0XC000
#define GPIOE_BASEADDR					    AHB2PERIPH_BASE + 0X1000
#define GPIOF_BASEADDR					    AHB2PERIPH_BASE + 0X1400
#define GPIOG_BASEADDR					    AHB2PERIPH_BASE + 0X1800
#define GPIOH_BASEADDR					    AHB2PERIPH_BASE + 0X1C00

/*
 Base adresses of peripherals which are hanging on APB1
 */

#define TIM2_BASEADDR					    APB1PERIPH_BASE + 0X0000
#define TIM3_BASEADDR					    APB1PERIPH_BASE + 0X0400
#define TIM6_BASEADDR					    APB1PERIPH_BASE + 0X1000
#define TIM7_BASEADDR					    APB1PERIPH_BASE + 0X1400

#define RTC_BASEADDR					    APB1PERIPH_BASE + 0X2800
#define WWDG_BASEADDR						APB1PERIPH_BASE + 0X2C00
#define IWDG_BASEADDR						APB1PERIPH_BASE + 0X3000
#define I2S2ext_BASEADDR					APB1PERIPH_BASE + 0X3400

#define SPI2_BASEADDR						APB1PERIPH_BASE + 0X3800  // SPI2/I2S2
#define SPI3_BASEADDR						APB1PERIPH_BASE + 0X3C00  // SPI3/I2S3

#define I2S3ext_BASEADDR					APB1PERIPH_BASE + 0X4000


#define USART2_BASEADDR					    APB1PERIPH_BASE + 0X4400
#define USART3_BASEADDR					    APB1PERIPH_BASE + 0X4800
#define UART4_BASEADDR					    APB1PERIPH_BASE + 0X4C00
#define UART5_BASEADDR					    APB1PERIPH_BASE + 0X5000

#define I2C1_BASEADDR					    APB1PERIPH_BASE + 0X5400
#define I2C2_BASEADDR					    APB1PERIPH_BASE + 0X5800

#define USBFS_BASEADDR					    APB1PERIPH_BASE + 0X5C00 // USB/CAN SRAM
#define USB_BASEADDR					    APB1PERIPH_BASE + 0X6000
#define bxCAN_BASEADDR					    APB1PERIPH_BASE + 0X6400

#define PWR_BASEADDR					    APB1PERIPH_BASE + 0X7000
#define DAC1_BASEADDR					    APB1PERIPH_BASE + 0X7400
#define I2C3_BASEADDR					    APB1PERIPH_BASE + 0X7800


/*
 Base adresses of peripherals which are hanging on APB2
 */

#define SYSCFG_BASEADDR					    APB2PERIPH_BASE			 // SYSCFG + COMP + OPAMP
#define EXTI_BASEADDR					    APB2PERIPH_BASE + 0X0400
#define TIM1_BASEADDR					    APB2PERIPH_BASE + 0X2C00
#define SPI1_BASEADDR					    APB2PERIPH_BASE + 0X3000
#define TIM8_BASEADDR					    APB2PERIPH_BASE + 0X3400
#define USART1_BASEADDR					    APB2PERIPH_BASE + 0X3800
#define SPI4_BASEADDR					    APB2PERIPH_BASE + 0X3C00
#define TIM15_BASEADDR					    APB2PERIPH_BASE + 0X4000
#define TIM16_BASEADDR					    APB2PERIPH_BASE + 0X4400
#define TIM17_BASEADDR					    APB2PERIPH_BASE + 0X4800
#define TIM20_BASEADDR					    APB2PERIPH_BASE + 0X5000

/*
 Base adresses of peripherals which are hanging on AHB1
 */



#define DMA1_BASEADDR					    AHB1PERIPH_BASE
#define DMA2_BASEADDR					    AHB1PERIPH_BASE + 0X0400

#define RCC_BASEADDR					    AHB1PERIPH_BASE + 0X1000
#define FLASH_INTERFACE_BASEADDR			AHB1PERIPH_BASE + 0X2000
#define CRC_BASEADDR					    AHB1PERIPH_BASE + 0X3000
#define TSC_BASEADDR			            AHB1PERIPH_BASE + 0X4000

/*
 Base adresses of peripherals which are hanging on AHB3
 */
#define ADC1_ADC2_BASEADDR					AHB3PERIPH_BASE
#define ADC3_ADC4_BASEADDR					AHB3PERIPH_BASE + 0X0400


/*
 Generic Macros
 */


#define ENABLE 								1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE;


/*
 Macros Baseadresses of GPIO typecasted
 */

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOA_BASEADDR)




typedef struct
{
	volatile uint32_t MODER;                    // GPIO port mode register, Offset: 0x00
	volatile uint32_t OTYPER;                   // GPIO port output type register, Offset: 0x04
	volatile uint32_t OSPEEDR;                  // GPIO port output speed register, Offset: 0x08
	volatile uint32_t PUPDR;                    // GPIO port pull-up/pull-down register, Offset: 0x0C
	volatile uint32_t IDR;						// GPIO port input data register, Offset: 0x10
	volatile uint32_t ODR;                      // GPIO port output data register, Offset: 0x14
	volatile uint32_t BSRR;                     // GPIO port bit set/reset register, Offset: 0x18
	volatile uint32_t LCKR;                     // GPIO port configuration lock register, Offset: 0x1C
	volatile uint32_t AFR[2];                   // GPIO alternate function high register AFR[0], GPIO alternate function low register AFR[1]
	volatile uint32_t BRR;                      // GPIO port bit reset register, Offset: 0x28
}GPIO_RegDef_t;




typedef struct
{
	volatile uint32_t CR;                       // Clock control register, Offset: 0x00
	volatile uint32_t CFGR;                     // Clock configuration register, Offset: 0x04
	volatile uint32_t CIR;                      // Clock interrupt register, Offset: 0x08
	volatile uint32_t APB2RSTR;                 // APB2 peripheral reset register, Offset: 0x0C
	volatile uint32_t APB1RSTR;				    // APB1 peripheral reset register, Offset: 0x10
	volatile uint32_t AHBENR;                   // AHB peripheral clock enable register, Offset: 0x14
	volatile uint32_t APB2ENR;                  // APB2 peripheral clock enable register, Offset: 0x18
	volatile uint32_t APB1ENR;                  // APB1 peripheral clock enable register, Offset: 0x1C
	volatile uint32_t BDCR;                     // RTC domain control register, Offset: 0x20
	volatile uint32_t CSR;                      // Control/status register, Offset: 0x24
	volatile uint32_t AHBRSTR;                  // AHB peripheral reset register, Offset: 0x28
	volatile uint32_t CFGR2;					// Clock configuration register 2, Offset: 0x2C
	volatile uint32_t CFGR3;					// Clock configuration register 3, Offset: 0x30
}RCC_RegDef_t;


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 17))
#define GPIOB_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 18))
#define GPIOC_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 19))
#define GPIOD_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 20))
#define GPIOE_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 21))
#define GPIOF_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 22))
#define GPIOG_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 23))
#define GPIOH_PERI_CLOCK_ENABLE() (RCC->AHBENR |= (1 << 16))



// Typecasted to typedef struct RCC_RegDef_t*
#define USART1 							    (USART_RegDef_t*)USART1_BASEADDR
#define USART2 							    (USART_RegDef_t*)USART2_BASEADDR
#define USART3 							    (USART_RegDef_t*)USART3_BASEADDR
#define UART4 							    (USART_RegDef_t*)UART4_BASEADDR
#define UART5 							    (USART_RegDef_t*)UART5_BASEADDR


typedef struct
{
	volatile uint32_t CR1;                      // Control register1, Offset: 0x00
	volatile uint32_t CR2;                      // Control register2, Offset: 0x04
	volatile uint32_t CR3;                      // Control register3, Offset: 0x08
	volatile uint32_t BRR;                      // Baud rate register, Offset: 0x0C
	volatile uint32_t GTPR;						// Guard time and prescaler register, Offset: 0x10
	volatile uint32_t RTOR;                     // Receiver timeout register, Offset: 0x14
	volatile uint32_t RQR;                      // Request register, Offset: 0x18
	volatile uint32_t ISR;                      // Interrupt and status register, Offset: 0x1C
	volatile uint32_t RDR;                      // Receive data register, Offset: 0x20
	volatile uint32_t TDR;                      // Transmit data register, Offset: 0x24
}USART_RegDef_t;






/*
 * Clock Enable Macros for USARTx and UARTx peripherals
 */
// Typecasted to typedef struc RCC_RegDef_t*

#define RCC 				(RCC_RegDef_t*)RCC_BASEADDR
#define USART1_PCCK_EN() 	(RCC->APB2ENR) |= (1 << 14))
#define USART2_PCCK_EN() 	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN() 	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN() 	(RCC->APB1ENR |= (1 << 20))















#endif /* STM32F303XX_H_ */
