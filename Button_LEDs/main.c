#include "stm32f4xx.h"                  // Device header

//LEDS ON STM32F4-DISCOVERY
#define   LED_RED      GPIO_ODR_OD14_Pos
#define   LED_GREEN    GPIO_ODR_OD12_Pos
#define   LED_BLUE     GPIO_ODR_OD15_Pos
#define   LED_ORANGE   GPIO_ODR_OD13_Pos

//DEFINE USER BUTTON
#define   BUTTON_1     GPIO_IDR_ID0_Pos

void delaySystick_ms(uint32_t delay);

int main()
{
	//ENABLE IO PORTD AND PORTA CLOCK IN RCC REGISTER
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIODEN_Pos)| 
									(1 << RCC_AHB1ENR_GPIOAEN_Pos);
	
	//CONFIGURING BUTTON PINS AS INPUT
	GPIOA->MODER &= ~(1 << GPIO_MODER_MODER0_Pos);	
	
	//CONFIGURING LED PINS AS OUTPUT
	GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 |
									GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	
	//CONFIGURING BUTTON GPIO SPEED AS MEDIUM SPEED
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_0;
	
	//CONFIGURING LED GPIO SPEED AS HIGH SPEED
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1 | GPIO_OSPEEDR_OSPEED13_1 |
										GPIO_OSPEEDR_OSPEED14_1 | GPIO_OSPEEDR_OSPEED15_1;
	
  //CONFIGURING BUTTON PORT WITHOUT PULL-UP/DOWN
	GPIOA->PUPDR &= ~(1 << GPIO_PUPDR_PUPD0_Pos);
	
	//CONFIGURING LED PORT WITH PULL-UP
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD12_0 | GPIO_PUPDR_PUPD13_0 |
									GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;

  int pin = 12;
		
	while(1)
	{	
		if((GPIOA->IDR) & (1 << BUTTON_1))
		{	
			//TOGGLE LEDS
			//GPIOD->ODR ^= (1 << LED_RED) ^ (1 << LED_BLUE) ^ (1 << LED_GREEN) ^ (1 << LED_ORANGE);
			GPIOD->ODR ^= (1 << pin);
			delaySystick_ms(100);
			
			if(pin <= 15)	pin++;
			else pin =12;
		}	
		
		else
		{
			//GPIOD->ODR = 0;
			GPIOD->ODR ^= (1 << pin);
			delaySystick_ms(100);
			
			if(pin >= 12)	pin--;
			else pin = 15;
		}			
	}
}	

//FUNCTION O CONFIGURE SYSTICK
void delaySystick_ms(uint32_t delay)
{
	SysTick->LOAD = (SystemCoreClock/1000) - 1; //TAKES THE DEFAULT CLOCK AND DIVIDE TO GET THE NUMBER OF CYCLES PER MILISECOND
	SysTick->VAL = 0; //MAKING SYSTICK INITIAL VALUE AS 0
	SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos)|(1 << SysTick_CTRL_CLKSOURCE_Pos); //ENABLING SYSTEM CLOCK, CLKSOURCE AND SYSTICK ENABLE
	for(int i = 0; i < delay; i++)
	{
		while((SysTick->CTRL & (1 << SysTick_CTRL_COUNTFLAG_Pos)) == 0);
	}	
	SysTick->CTRL = 0; //DISABE SYSTICK
}	
