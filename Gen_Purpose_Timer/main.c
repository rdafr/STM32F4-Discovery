#include <stdint.h>
#include "stm32f4xx.h"

void initTimer4(void);

int main()
{
	__disable_irq();
	
  //ENABLE IO PORTD CLOCK IN RCC REGISTER
	RCC->AHB1ENR |= (1 << 3);
	
	//CONFIGURING GPIO 
	GPIOD->MODER = 0;
	GPIOD->MODER |= GPIO_MODER_MODER14_0 | //LED PIN GPIO
									GPIO_MODER_MODER15_0 ; //LED PIN GPIO
	GPIOD->MODER |= (1 << 25); //ALTERNATE FUNCTION FOR OUTPUT COMPARE
	GPIOD->MODER |= (1 << 27); //ALTERNATE FUNCTION FOR PWM
	
	GPIOD->AFR[1] = 0;
	GPIOD->AFR[1] |= (1 << 17);
	GPIOD->AFR[1] |= (1 << 21);
	
	initTimer4();
	
	__NVIC_EnableIRQ(TIM4_IRQn);
	__enable_irq();
	
	while(1)
	{	
		while(!(TIM4->SR & (1 << TIM_SR_UIF_Pos)));
		//TOGGLE LED BLUE
		GPIOD->ODR ^= (1 << GPIO_ODR_OD15_Pos);
		TIM4->SR &= ~(1 << TIM_SR_UIF_Pos);
	}
}	

void initTimer4(void)
{
	//ENABLE TIMER3 CLOCK
	RCC->APB1ENR |= (1 << 2);
		
	//CONFIGURING PRESCALE AS 1600
	TIM4->PSC = 1600 - 1; //16MHz/160 = 10 kHz
	
	//CONFIGURING AUTO-RELOAD VALUE AS 10000 
	TIM4->ARR = 10000 - 1;
	
	//CONFIGURING TO TOGGLE ON OUTPUT COMPARE
	TIM4->CCMR1 = 0;
	TIM4->CCMR1 |= (3 << TIM_CCMR1_OC1M_Pos);
	
  //CONFIGURING PWM ON GPIOD 13 
	TIM4->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);
	
	//OUTPUT COMPARE 1 VALUE
	TIM4->CCR1 = 5000;
	
	//LOW OUTPUT COMPARE 2 VALUE
	TIM4->CCR2 = 9000;	
	
	//ENABLE OUTPUT COMPARE 1
	TIM4->CCER |= TIM_CCER_CC1E_Msk;
	
	//ENABLE OUTPUT COMPARE 2
	TIM4->CCER |= TIM_CCER_CC2E_Msk;	
	
	//CLEAR CNT REGISTER
	TIM4->CNT = 0;
	
	//CONFIGURING TIMER
	TIM4->CR1 |= 1; //COUNTER ENABLE	
	
	//UPDATE INTERRUPT ENABLE
	TIM4->DIER |= 1;
}

void TIM4_IRQHandler()
{
	TIM4->SR &= ~1;
	GPIOD->ODR ^= (1 << GPIO_ODR_OD14_Pos);
}	
