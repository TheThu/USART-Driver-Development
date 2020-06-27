/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************

*/
#include <main.h>
#include <stm32f303xx.h>

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


void GPIOLEDsetup()
{
	   GPIO_Handle_t GPIO_LED;

	   memset(&GPIO_LED,0, sizeof(GPIO_LED));
	   GPIO_LED.pGPIOx = GPIOA;
	   GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN5;
	   GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	   GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	   GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	   GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	   GPIO_PeriClockControl(GPIOA, ENABLE);
	   GPIO_Init(&GPIO_LED);
}


void GPIOButtonsetup()
{
	GPIO_Handle_t GPIO_USER_BUTTON;

	// local will contain garbage value when Output type is not assigned
	memset(&GPIO_USER_BUTTON,0, sizeof(GPIO_USER_BUTTON));

	GPIO_USER_BUTTON.pGPIOx = GPIOC;
	GPIO_USER_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN13;
	GPIO_USER_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_USER_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_USER_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIOC_PCLK_EN();
	GPIO_Init(&GPIO_USER_BUTTON);
}






void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
}

int main(void)
{

	GPIOButtonsetup();
	GPIOLEDsetup();

	GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);

	  while(1)
	   {



	   }



	  return 0;






};

