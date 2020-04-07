#include "stm32f4xx.h"                  // Device header

#define BAUD 9600

void delaySystick_ms(uint32_t delay);
void usart3_Init(void);
void usart3_Write (uint8_t word);

uint8_t test1 = 'H';
uint8_t test2 = 'i';
uint8_t test3 = '!';

volatile uint8_t test4;

int main()
{
	//ENABLE IO PORTD CLOCK IN RCC REGISTER
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIODEN_Pos);
	
	//CONFIGURING LED PINS AS OUTPUT
	GPIOD->MODER |= GPIO_MODER_MODER12_0;
	
	//CONFIGURING GPIO SPEED AS HIGH SPEED
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1;
	
	//CONFIGURING PORT PULL-UP
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD12_0;
	
	usart3_Init();			
	
	while(1)
	{	
		/*usart3_Write (test1);
		usart3_Write (test2);
		usart3_Write (test3);
		delaySystick_ms(1000);
		GPIOD->ODR ^= (1 << GPIO_ODR_OD12_Pos);*/
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

//INITIALIZE USART3
void usart3_Init(void)
{
	__disable_irq();
	
	//ENABLE USART3 CLOCK AND IO PORTB CLOCK FOR UART IO
	RCC->APB1ENR |= (1 <<RCC_APB1ENR_USART3EN_Pos);
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos);
	
	//DEFINE AFR REGEISTER WITH USART3
	GPIOB->AFR[1] = (7 << GPIO_AFRH_AFSEL10_Pos) | 
									(7 << GPIO_AFRH_AFSEL11_Pos);
	
	//DEFINE UART PINS AS ALTERNATE FUNCTION PINS
	GPIOB->MODER |= GPIO_MODER_MODER10_1 |
									GPIO_MODER_MODER11_1;
	
	//DEFINE BAUD RATE
	USART3->BRR = SystemCoreClock / BAUD; // 16MHZ/9600 bps = 0x683
	
	//ENABLE USART PERIPHERAL, TRANSMITTER AND RECIEVER
	USART3->CR1 |= (1 << USART_CR1_UE_Pos) |
								 (1 << USART_CR1_TE_Pos) |
								 (1 << USART_CR1_RE_Pos) |
								 (1 << USART_CR1_RXNEIE_Pos);
									
	
	__NVIC_EnableIRQ(USART3_IRQn);
	__enable_irq();
}

//WRITE CHARACTER BY USART3
void usart3_Write (uint8_t word)
{
	while(!(USART3->SR & 0x80));
	USART3->DR = word;
}

//IRQ USART3
void USART3_IRQHandler()
{
  if(USART3->SR & USART_SR_RXNE)
  {
    GPIOD->ODR ^= (1 << GPIO_ODR_OD12_Pos);
		test4 = USART3->DR;
		usart3_Write(test4);		
		USART3->SR = ~(1 << USART_SR_RXNE_Pos);
  }
}
