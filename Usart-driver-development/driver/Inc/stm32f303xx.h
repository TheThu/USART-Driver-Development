/*
 * stm32f303xx.h
 *
 *  Created on: May 15, 2020
 *      Author: T2
 */

#ifndef STM32F303xx_H_
#define STM32F303xx_H_
/*
 * stm32f303xx.h
 *
 *  Created on: May 15, 2020
 *      Author: T2
 */


#include <stdint.h>
#include <stdio.h>


/*
 * Processor specific adresses
 */

#define NVIC_ISER0							((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1							((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2							((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3							((volatile uint32_t*)0xE000E10C)


#define NVIC_ICER0							((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1							((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2							((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3							((volatile uint32_t*)0xE000E18C)
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

#define GPIOA_BASEADDR					    (AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR					    (AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR					    (AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR					    (AHB2PERIPH_BASE + 0xC000)
#define GPIOE_BASEADDR					    (AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR					    (AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR					    (AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR					    (AHB2PERIPH_BASE + 0x1C00)

/*
 Base adresses of peripherals which are hanging on APB1
 */

#define TIM2_BASEADDR					    (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR					    (APB1PERIPH_BASE + 0x0400)
#define TIM6_BASEADDR					    (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR					    (APB1PERIPH_BASE + 0x1400)

#define RTC_BASEADDR					    (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR						(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR						(APB1PERIPH_BASE + 0x3000)
#define I2S2ext_BASEADDR					(APB1PERIPH_BASE + 0x3400)

#define SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800)  // SPI2/I2S2
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00)  // SPI3/I2S3

#define I2S3ext_BASEADDR					(APB1PERIPH_BASE + 0x4000)


#define USART2_BASEADDR					    (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR					    (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR					    (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR					    (APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR					    (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR					    (APB1PERIPH_BASE + 0x5800)

#define USBFS_BASEADDR					    (APB1PERIPH_BASE + 0x5C00) // USB/CAN SRAM
#define USB_BASEADDR					    (APB1PERIPH_BASE + 0x6000)
#define bxCAN_BASEADDR					    (APB1PERIPH_BASE + 0x6400)

#define PWR_BASEADDR					    (APB1PERIPH_BASE + 0x7000)
#define DAC1_BASEADDR					    (APB1PERIPH_BASE + 0x7400)
#define I2C3_BASEADDR					    (APB1PERIPH_BASE + 0x7800)


/*
 Base adresses of peripherals which are hanging on APB2
 */

#define SYSCFG_BASEADDR					    APB2PERIPH_BASE			 // SYSCFG + COMP + OPAMP
#define EXTI_BASEADDR					    (APB2PERIPH_BASE + 0x0400)
#define TIM1_BASEADDR					    (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR					    (APB2PERIPH_BASE + 0x3000)
#define TIM8_BASEADDR					    (APB2PERIPH_BASE + 0x3400)
#define USART1_BASEADDR					    (APB2PERIPH_BASE + 0x3800)
#define SPI4_BASEADDR					    (APB2PERIPH_BASE + 0x3C00)
#define TIM15_BASEADDR					    (APB2PERIPH_BASE + 0x4000)
#define TIM16_BASEADDR					    (APB2PERIPH_BASE + 0x4400)
#define TIM17_BASEADDR					    (APB2PERIPH_BASE + 0x4800)
#define TIM20_BASEADDR					    (APB2PERIPH_BASE + 0x5000)

/*
 Base adresses of peripherals which are hanging on AHB1
 */
//0x4002 1000


#define DMA1_BASEADDR					    AHB1PERIPH_BASE
#define DMA2_BASEADDR					    (AHB1PERIPH_BASE + 0x0400)

//#define RCC_BASEADDR					    0x40021000U AHB1PERIPH_BASE
#define RCC_BASEADDR					    (AHB1PERIPH_BASE + 0x1000)
#define FLASH_INTERFACE_BASEADDR			(AHB1PERIPH_BASE + 0x2000)
#define CRC_BASEADDR					    (AHB1PERIPH_BASE + 0x3000)
#define TSC_BASEADDR			            (AHB1PERIPH_BASE + 0x4000)

/*
 Base adresses of peripherals which are hanging on AHB3
 */
#define ADC1_ADC2_BASEADDR					AHB3PERIPH_BASE
#define ADC3_ADC4_BASEADDR					(AHB3PERIPH_BASE + 0x0400)



/*
 IRQ (Interrupt Request) Number of STM32F303REx MCU
 */

#define IRQ_NO_EXTI0							6
#define IRQ_NO_EXTI1							7
#define IRQ_NO_EXTI2							8
#define IRQ_NO_EXTI3							9
#define IRQ_NO_EXTI4							10
#define IRQ_NO_EXTI9_5							23
#define IRQ_NO_EXTI15_10						40



/*
 Generic Macros
 */


#define ENABLE 								1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET

// Verify in vector file in datasheet

#define IRQ_NO_EXTI



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

/*
 Macros Baseadresses of GPIO typecasted
 */

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)



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


typedef struct
{
	volatile uint32_t CFGR1;                    // SYSCFG configuration register 1, Offset: 0x00
	volatile uint32_t RCR;                     	// SYSCFG CCM SRAM protection register, Offset: 0x04
	volatile uint32_t EXTICR[4];                // SYSCFG external interrupt configuration register 1,2,3,4, Offset: 0x08
	volatile uint32_t CFGR2;                  	// SYSCFG configuration register 2, Offset: 0x18
		     uint32_t reserved[8];              // Reserced 0x1C - 4C
	volatile uint32_t CFGR3;                    // SYSCFG configuration register 2, Offset: 0x50
}SYSCFG_RegDef_t;





// Typecasted to typedef struct RCC_RegDef_t*
#define USART1 							    (USART_RegDef_t*)USART1_BASEADDR
#define USART2 							    (USART_RegDef_t*)USART2_BASEADDR
#define USART3 							    (USART_RegDef_t*)USART3_BASEADDR
#define USART4 							    (USART_RegDef_t*)UART4_BASEADDR
#define USART5 							    (USART_RegDef_t*)UART5_BASEADDR


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


typedef struct
{
	volatile uint32_t IMR1;                     // Interrupt mask register1, Offset: 0x00
	volatile uint32_t EMR1;                     // Event mask register1, Offset: 0x04
	volatile uint32_t RTSR1;                    // Rising trigger selection register1, Offset: 0x08
	volatile uint32_t FTSR1;                    // Falling trigger selection register1, Offset: 0x0C
	volatile uint32_t SWIER1;				    // Software interrupt event register1, Offset: 0x10
	volatile uint32_t PR1;                      // Pending register1, Offset: 0x14
			 uint32_t Reserved;					// Reserved, Offset: 0x18
	volatile uint32_t IMR2;						// Event mask register2, Offset: 0x20
	volatile uint32_t EMR2;						// Event mask register2, Offset: 0x24
	volatile uint32_t RTSR2;					// Rising trigger selection register2, Offset 0x28
	volatile uint32_t FTSR2;					// Falling trigger selection register2, Offset 0x2C
}EXTI_RegDef_t;






/*
 * Clock Enable Macros for USARTx and UARTx peripherals
 */
// Typecasted to typedef struc RCC_RegDef_t*

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define USART1_PCCK_EN() 	((RCC->APB2ENR) |= (1 << 14))
#define USART2_PCCK_EN() 	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN() 	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN() 	(RCC->APB1ENR |= (1 << 20))




/*
 * Clock Enable Macros for GPIO Peripherals peripherals
 */


#define GPIOA_PCLK_EN() 	((RCC->AHBENR) |= (1 << 17))
#define GPIOB_PCLK_EN() 	((RCC->AHBENR) |= (1 << 18))
#define GPIOC_PCLK_EN() 	((RCC->AHBENR) |= (1 << 19))
#define GPIOD_PCLK_EN() 	((RCC->AHBENR) |= (1 << 20))
#define GPIOE_PCLK_EN() 	((RCC->AHBENR) |= (1 << 21))
#define GPIOF_PCLK_EN() 	((RCC->AHBENR) |= (1 << 22))
#define GPIOG_PCLK_EN() 	((RCC->AHBENR) |= (1 << 23))
#define	GPIOH_PCLK_EN()		((RCC->AHBENR) |= (1 << 16))


/*
 * Clock Enable Macros SYSCFG
 */


#define SYSCFG_PCLK_EN()	((RCC->APB2ENR) |= (1 << 0))






/*
 * Clock Reset Macros for GPIOx peripherals
 */

#define GPIOA_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOE_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); }while(0)
#define GPIOF_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); }while(0)
#define GPIOG_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 23)); (RCC->AHBRSTR &= ~(1 << 23)); }while(0)
#define GPIOH_REG_RESET() do{ (RCC->AHBRSTR |= (1 << 16)); (RCC->AHBRSTR &= ~(1 << 16)); }while(0)





#include <stm32f303xx_gpio_driver.h>
#include <stm32f303xx_rcc_driver.h>


#endif /* STM32F303xx_H_ */
