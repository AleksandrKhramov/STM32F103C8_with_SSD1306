#include "main.h"

bool Mode_Rectangle = false;
bool LED_En = false;
uint32_t TriggeredNumber = 0;

int main()
{
	RCC_Init();
	GPIO_Init();
	SPI1_Init();	
	SSD1306_Init();	
	USART3_Init();
	TIM1_Init();
	//TIM2_Init();
	//TIM3_Init();
	//TIM4_Init();
	

	//ResetState();
	
	while(1)
	{
		//GPIOC->BSRR |= GPIO_BSRR_BS13;
		delay_ms(200);
		disp1color_FillScreenbuff(0);
		
		disp1color_printf(0, 1, FONTID_6X8M,  "             Готов  к  работе\n\r");
		disp1color_printf(118, 1, FONTID_6X8M, "%c", 0x81);

		//disp1color_printf(0, 1, FONTID_6X8M,  "             Выход  на  режим\n\r");
		//disp1color_printf(118, 1, FONTID_6X8M, "x");

		disp1color_DrawLine(0, 10, 127, 10);
		disp1color_printf(28, 23, FONTID_10X16F, "32.537 %cC", 0x80);

		//disp1color_printf(0, 46, FONTID_6X8M, "Сработало %d раз", TriggeredNumber); 

		disp1color_DrawLine(0, 54, 127, 54);

		disp1color_printf(43, 57, FONTID_6X8M, "R1: 0.001044 Ом");  

		disp1color_DrawLine(34, 59, 36, 57);
		disp1color_DrawLine(36, 57, 38, 59);
		disp1color_DrawLine(37, 59, 35, 59);

		disp1color_DrawLine(34, 61, 36, 63);
		disp1color_DrawLine(36, 63, 38, 61);
		disp1color_DrawLine(37, 61, 35, 61);

		if(Mode_Rectangle)
			disp1color_DrawRectangle(MODE_RECT_L - 1, MODE_RECT_T - 1, MODE_RECT_L + 3, MODE_RECT_T + 3);	
		else
			disp1color_DrawRectangle(MODE_RECT_L, MODE_RECT_T, MODE_RECT_L + 2, MODE_RECT_T + 2);

		disp1color_UpdateFromBuff();

		Mode_Rectangle = !Mode_Rectangle;

		//GPIOC->BSRR |= GPIO_BSRR_BR13;
		delay_ms(200);
		//GPIOC->BSRR |= GPIO_BSRR_BR13;
	}
}
//**************************  Initialization functions  ***********************************
void RCC_Init(void)
{
 //Description on 13:30 of third lesson	
	
 //Clock control register settings
	
	RCC->CR |= ((uint32_t)RCC_CR_HSEON); 												// Enable HSE   							/*!< External High Speed clock enable */
	//Set and cleared by software.
	//Cleared by hardware to stop the HSE oscillator when entering in Stop or Standby mode. This
	//bit cannot be reset if the HSE oscillator is used directly or indirectly as the system clock.
	
	while (!(RCC->CR & RCC_CR_HSERDY));													// Ready start HSE						/*!< External High Speed clock ready flag */	
	//Set by hardware to indicate that the HSE oscillator is stable. This bit needs 6 cycles of the
	//HSE oscillator clock to go to zero after HSEON is reset.
	
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;					// Cloclk Flash memory

 //Clock configuration register
	
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;														// AHB = SYSCLK/1
	//AHB prescaler
  //Set and cleared by software to control the division factor of the AHB clock.
	
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;														// APB1 = HCLK/1	
	//APB low-speed prescaler (APB1)
	//Set and cleared by software to control the division factor of the APB low-speed clock (PCLK1).
	//Warning: the software has to set correctly these bits to not exceed 36 MHz on this domain.
	
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;														// APB2 = HCLK/1
	//APB high-speed prescaler (APB2)
	//Set and cleared by software to control the division factor of the APB high-speed clock (PCLK2).
	
	//Description of next three registers are below themselves 
	RCC->CFGR &= ~RCC_CFGR_PLLMULL;               							// clear PLLMULL bits
	RCC->CFGR &= ~RCC_CFGR_PLLSRC;															// clearn PLLSRC bits
	RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;														// clearn PLLXTPRE bits
	
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE; 											// source HSE									/*!< PREDIV1 clock selected as PLL entry clock source */
	//PLL entry clock source
	//Set and cleared by software to select PLL clock source. This bit can be written only when
	//PLL is disabled.
	
	RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE_Div2; 								// source HSE/2 = 4 MHz				/*!< PREDIV1 clock divided by 2 for PLL entry */
	//LSB of division factor PREDIV1
	//Set and cleared by software to select the least significant bit of the PREDIV1 division factor. 
	
	RCC->CFGR |= RCC_CFGR_PLLMULL6; 														// PLL x6: clock = 4 MHz * 6 = 24 MHz		/*!< PLL input clock*6 */
	//PLL multiplication factor
	//These bits are written by software to define the PLL multiplication factor. These bits can be
	//written only when PLL is disabled.

 //Clock configuration register	
	RCC->CR |= RCC_CR_PLLON;                      							// enable PLL
	//PLL enable
	//Set and cleared by software to enable PLL.
	//Cleared by hardware when entering Stop or Standby mode. This bit can not be reset if the
	//PLL clock is used as system clock or is selected to become the system clock.
	
	while((RCC->CR & RCC_CR_PLLRDY) == 0);      								// wait till PLL is ready
	//PLL clock ready flag
	//Set by hardware to indicate that the PLL is locked.

 //Clock configuration register		
	RCC->CFGR &= ~RCC_CFGR_SW;                   							 	// clear SW bits
  RCC->CFGR |= RCC_CFGR_SW_PLL;                 							// select source SYSCLK = PLL
	//System clock switch
	//Set and cleared by software to select SYSCLK source.
	//Set by hardware to force HSI selection when leaving Stop and Standby mode or in case of
	//failure of the HSE oscillator used directly or indirectly as system clock (if the Clock Security
	//System is enabled).

	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1);  			// wait till PLL is used
	//System clock switch status
	//Set and cleared by hardware to indicate which clock source is used as system clock.
	
}
//-----------------------------------------------------------------------------------------
void GPIO_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                       	//enable clock for port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                       	//enable clock for port C
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                       	//enable clock for port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 						//enable clock for AFIO
	
	//Green LED init
	GPIOC->CRH &= ~GPIO_CRH_CNF13;				
	GPIOC->CRH |= GPIO_CRH_MODE13_0;			

	//******************************
	//** Setting GPIO for buttons **
	//******************************

	//Left button
	GPIOB->CRH &= ~GPIO_CRH_CNF14;					//Reset CNF register				
	GPIOB->CRH &= ~GPIO_CRH_MODE14;					//Input mode
	GPIOB->CRH |= GPIO_CRH_CNF14_1;					//Input with pull up/pull down
	GPIOB->ODR &= ~GPIO_ODR_ODR14;					//Pull down	

	AFIO->EXTICR[3] &= ~AFIO_EXTICR4_EXTI14;		//Channel EXTI Reset
	AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PB;		//Channel EXTI connected to PB 

	EXTI->RTSR &= ~EXTI_RTSR_TR14;					//Rising trigger enabled for fourth channel
	EXTI->FTSR |= EXTI_FTSR_TR14;					//Falling trigger disabled for fourth channel

	EXTI->PR |= EXTI_PR_PR14;						//Clear interrupt flag
	EXTI->IMR |= EXTI_IMR_MR14;						//Enable interrupt
	
	//Right button
	GPIOB->CRH &= ~GPIO_CRH_CNF15;					//Reset CNF register				
	GPIOB->CRH &= ~GPIO_CRH_MODE15;					//Input mode
	GPIOB->CRH |= GPIO_CRH_CNF15_1;					//Input with pull up/pull down
	GPIOB->ODR &= ~GPIO_ODR_ODR15;					//Pull down	

	AFIO->EXTICR[3] &= ~AFIO_EXTICR4_EXTI15;		//Channel EXTI Reset
	AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PB;		//Channel EXTI connected to PA 

	EXTI->RTSR &= ~EXTI_RTSR_TR15;					//Rising trigger enabled for fourth channel
	EXTI->FTSR |= EXTI_FTSR_TR15;					//Falling trigger disabled for fourth channel

	EXTI->PR |= EXTI_PR_PR15;						//Clear interrupt flag
	EXTI->IMR |= EXTI_IMR_MR15;						//Enable interrupt

	//Up button
	GPIOA->CRH &= ~GPIO_CRH_CNF8;					//Reset CNF register		
	GPIOA->CRH &= ~GPIO_CRH_MODE8;					//Input mode
	GPIOA->CRH |= GPIO_CRH_CNF8_1;					//Input with pull up/pull down
	GPIOA->ODR &= ~GPIO_ODR_ODR8;					//Pull down						

	AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI8;			//Channel EXTI connected to PA 

	EXTI->RTSR &= ~EXTI_RTSR_TR8;					//Rising trigger enabled for third channel
	EXTI->FTSR |= EXTI_FTSR_TR8;					//Falling trigger disabled for third channel

	EXTI->PR |= EXTI_PR_PR8;						//Clear interrupt flag
	EXTI->IMR |= EXTI_IMR_MR8;						//Enable interrupt

	//Down button
	GPIOA->CRH &= ~GPIO_CRH_CNF9;					//Reset CNF register				
	GPIOA->CRH &= ~GPIO_CRH_MODE9;					//Input mode
	GPIOA->CRH |= GPIO_CRH_CNF9_1;					//Input with pull up/pull down
	GPIOA->ODR &= ~GPIO_ODR_ODR9;					//Pull down	

	AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI9;			//Channel EXTI connected to PA 

	EXTI->RTSR &= ~EXTI_RTSR_TR9;					//Rising trigger enabled for third channel
	EXTI->FTSR |= EXTI_FTSR_TR9;					//Falling trigger disabled for third channel

	EXTI->PR |= EXTI_PR_PR9;						//Clear interrupt flag
	EXTI->IMR |= EXTI_IMR_MR9;						//Enable interrupt

	//Enter button
	GPIOA->CRH &= ~GPIO_CRH_CNF10;					//Reset CNF register				
	GPIOA->CRH &= ~GPIO_CRH_MODE10;					//Input mode
	GPIOA->CRH |= GPIO_CRH_CNF10_1;					//Input with pull up/pull down
	GPIOA->ODR &= ~GPIO_ODR_ODR10;					//Pull down	

	AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI10;		//Channel EXTI connected to PA 

	EXTI->RTSR &= ~EXTI_RTSR_TR10;					//Rising trigger enabled for third channel
	EXTI->FTSR |= EXTI_FTSR_TR10;					//Falling trigger disabled for third channel

	EXTI->PR |= EXTI_PR_PR10;						//Clear interrupt flag
	EXTI->IMR |= EXTI_IMR_MR10;						//Enable interrupt

	//Clear button
	GPIOA->CRH &= ~GPIO_CRH_CNF11;					//Reset CNF register				
	GPIOA->CRH &= ~GPIO_CRH_MODE11;					//Input mode
	GPIOA->CRH |= GPIO_CRH_CNF11_1;					//Input with pull up/pull down
	GPIOA->ODR &= ~GPIO_ODR_ODR11;					//Pull down	

	AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI11;		//Channel EXTI connected to PA 

	EXTI->RTSR &= ~EXTI_RTSR_TR11;					//Rising trigger enabled for third channel
	EXTI->FTSR |= EXTI_FTSR_TR11;					//Falling trigger disabled for third channel

	EXTI->PR |= EXTI_PR_PR11;						//Clear interrupt flag
	EXTI->IMR |= EXTI_IMR_MR11;						//Enable interrupt

	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
//------------------------------------------------------------------------------------------
void SPI1_Init(void)
{
	//Enable clock for SPI1 Рё GPIOA
  	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	/*********************************************/
	/*** Setting pins GPIOA for work with SPI1 ***/
	/*********************************************/
	//PA7 - MOSI
	//PA6 - MISO
	//PA5 - SCK
	
	//First, clear all comfiguration bits
  	GPIOA->CRL &= ~GPIO_CRL_CNF5;
	GPIOA->CRL &= ~GPIO_CRL_CNF6;
	GPIOA->CRL &= ~GPIO_CRL_CNF7;
	
	GPIOA->CRL &= ~GPIO_CRL_MODE5;
	GPIOA->CRL &= ~GPIO_CRL_MODE5;
	GPIOA->CRL &= ~GPIO_CRL_CNF5;
	
	
	//Setting
	//SCK
	GPIOA->CRL |= GPIO_CRL_CNF5_1; 
	GPIOA->CRL |= GPIO_CRL_MODE5;
  
	//MISO
	GPIOA->CRL |= GPIO_CRL_CNF6_0; 
	GPIOA->CRL &= ~GPIO_CRL_MODE6;
  
	//MOSI
	GPIOA->CRL |= GPIO_CRL_CNF7_1; 
	GPIOA->CRL |= GPIO_CRL_MODE7;
	
  
	/**********************/
	/***  Setting SPI1  ***/
	/**********************/
  
	SPI1->CR1 |= SPI_CR1_BR_2;        			//Baud rate: F_PCLK/32
	SPI1->CR1 &= ~SPI_CR1_CPOL;					//Clock polarity SPI: 0
	SPI1->CR1 &= ~SPI_CR1_CPHA;					//Clock phase SPI: 0
	SPI1->CR1 &= ~SPI_CR1_DFF;  				//Data frame format is 8 bit 
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;    			//MSB first
	SPI1->CR1 |= SPI_CR1_SSM;          			//Software NSS management SS
	SPI1->CR1 |= SPI_CR1_SSI;          			//SS in high level
  
	SPI1->CR1 |= SPI_CR1_MSTR;         			//Master mode
	
	SPI1->CR1 |= SPI_CR1_SPE; 					//Enable SPI
}
//-----------------------------------------------------------------------------------------
void USART3_Init(void)
{	
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 
	
	//Tx
	GPIOB->CRH |= GPIO_CRH_CNF10_1;
	GPIOB->CRH |= GPIO_CRH_MODE10;
	
	//Rx
	GPIOB->CRH |= GPIO_CRH_CNF11_0;
	GPIOB->CRH &= ~GPIO_CRH_MODE11;
	
	USART3->BRR = 0x9C4; 
	
	USART3->CR1 |= USART_CR1_TE; 
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_UE;
	
	USART3->CR1 |= USART_CR1_RXNEIE; 

	NVIC_EnableIRQ(USART3_IRQn);	
}
//-----------------------------------------------------------------------------------------
void TIM1_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  					// Enable TIM2 Periph clock

	TIM1->PSC = 7199; 										// 10000 tick/sec 	
	TIM1->ARR = 10000;  									// 
	TIM1->DIER |= TIM_DIER_UIE; 							// Enable tim2 interrupt
	TIM1->CR1 |= TIM_CR1_CEN;  			 					// Start count

  	NVIC_EnableIRQ(TIM1_UP_IRQn); 		 						// Enable IRQ
}
//-----------------------------------------------------------------------------------------
void TIM2_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  					// Enable TIM2 Periph clock

	TIM2->PSC = 7199; 										// 10000 tick/sec 	
	TIM2->ARR = 10000;  									// 
	TIM2->DIER |= TIM_DIER_UIE; 							// Enable tim2 interrupt
	TIM2->CR1 |= TIM_CR1_CEN;  			 					// Start count

  	NVIC_EnableIRQ(TIM2_IRQn); 		 						// Enable IRQ
}
//-----------------------------------------------------------------------------------------
void TIM3_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  					// Enable TIM2 Periph clock

	TIM3->PSC = 7199; 										// 10000 tick/sec 	
	TIM3->ARR = 10000;  									// 
	TIM3->DIER |= TIM_DIER_UIE; 							// Enable tim2 interrupt
	TIM3->CR1 |= TIM_CR1_CEN;  			 					// Start count

  	NVIC_EnableIRQ(TIM3_IRQn); 		 						// Enable IRQ
}
//-----------------------------------------------------------------------------------------
void TIM4_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  					// Enable TIM2 Periph clock

	TIM4->PSC = 7199; 										// 10000 tick/sec 	
	TIM4->ARR = 10000;  									// 
	TIM4->DIER |= TIM_DIER_UIE; 							// Enable tim2 interrupt
	TIM4->CR1 |= TIM_CR1_CEN;  			 					// Start count

  	NVIC_EnableIRQ(TIM4_IRQn); 		 						// Enable IRQ
}
//-----------------------------------------------------------------------------------------
void SSD1306_GPIO_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	//----------------------------
	//SSD1306_RESET
	GPIOA->CRL &= ~GPIO_CRL_CNF3;
	GPIOA->CRL &= ~GPIO_CRL_MODE3;

	GPIOA->CRL &= ~GPIO_CRL_CNF3; 
	GPIOA->CRL |= GPIO_CRL_MODE3_0;

	//SSD1306_DC
	GPIOA->CRL &= ~GPIO_CRL_CNF1;
	GPIOA->CRL &= ~GPIO_CRL_MODE1;

	GPIOA->CRL &= ~GPIO_CRL_CNF1; 
	GPIOA->CRL |= GPIO_CRL_MODE1_0;

	//SSD1306_CS
	GPIOA->CRL &= ~GPIO_CRL_CNF2;
	GPIOA->CRL &= ~GPIO_CRL_MODE2;
	
	GPIOA->CRL &= ~GPIO_CRL_CNF2; 
	GPIOA->CRL |= GPIO_CRL_MODE2_0;
	//----------------------------
}
//*****************************************************************************************
//-----------------------------------------------------------------------------------------
void SPI1_Write(uint8_t *pBuff, uint16_t BuffLen)
{
	for(uint16_t i = 0; i < BuffLen; ++i)
	{
		//Expect while buffer ready
		while(!(SPI1->SR & SPI_SR_TXE)) ;	
		//Send data
		SPI1->DR = pBuff[i];
		//Delay caused display MCU handing ability
		DelayMicro(7);
	}	
}
//------------------------------------------------------------------------------------------
//********************************** Interrupts handlers ***********************************
//------------------------------------------------------------------------------------------
void EXTI9_5_IRQHandler(void)
{
	GPIOC->BSRR |= GPIO_BSRR_BS13;
	EXTI->PR |= EXTI_PR_PR5; 
	EXTI->PR |= EXTI_PR_PR6;
	EXTI->PR |= EXTI_PR_PR8; 							//Reset interrupt
	EXTI->PR |= EXTI_PR_PR9; 							//Reset interrupt

	++TriggeredNumber;

  	GPIOC->BSRR |= GPIO_BSRR_BR13;
}
//------------------------------------------------------------------------------------------
void EXTI15_10_IRQHandler(void)
{
	GPIOC->BSRR |= GPIO_BSRR_BS13;
	EXTI->PR |= EXTI_PR_PR10; 							//Reset interrupt
	EXTI->PR |= EXTI_PR_PR11; 							//Reset interrupt
	
	++TriggeredNumber;
	
  	GPIOC->BSRR |= GPIO_BSRR_BR13;
}
//------------------------------------------------------------------------------------------
void USART3_IRQHandler(void)
{
	if(USART1->SR & USART_CR1_RXNEIE)
	{
		USART1->SR &= ~USART_CR1_RXNEIE;
		
		/*if(USART1->DR != 0)
		{
			IsRecCon = true;
			CanHanding = false;
			RecUARTBuf[RecUARTNum++] = USART1->DR;
		}*/
	}
}
//------------------------------------------------------------------------------------------
void TIM1_UP_IRQHandler(void)
{
	TIM1->SR &= ~TIM_SR_UIF; 

	if (LED_En) 
	{
		GPIOC->BSRR |= GPIO_BSRR_BS13;	
	} 
	else 
	{
		GPIOC->BSRR |= GPIO_BSRR_BR13;
	}

	LED_En = !LED_En;
}
//------------------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~TIM_SR_UIF; 

	if (LED_En) 
	{
		GPIOC->BSRR |= GPIO_BSRR_BS13;	
	} 
	else 
	{
		GPIOC->BSRR |= GPIO_BSRR_BR13;
	}

	LED_En = !LED_En;
}
//------------------------------------------------------------------------------------------
void TIM3_IRQHandler(void)
{
	TIM3->SR &= ~TIM_SR_UIF; 

	if (LED_En) 
	{
		GPIOC->BSRR |= GPIO_BSRR_BS13;	
	} 
	else 
	{
		GPIOC->BSRR |= GPIO_BSRR_BR13;
	}

	LED_En = !LED_En;
}
//------------------------------------------------------------------------------------------
void TIM4_IRQHandler(void)
{
	TIM4->SR &= ~TIM_SR_UIF; 

	if (LED_En) 
	{
		GPIOC->BSRR |= GPIO_BSRR_BS13;	
	} 
	else 
	{
		GPIOC->BSRR |= GPIO_BSRR_BR13;
	}

	LED_En = !LED_En;
}
//------------------------------------------------------------------------------------------
//**************************** Working functions *******************************************
//------------------------------------------------------------------------------------------
void DelayMicro(uint32_t time)
{
	for(int i = 0; i < 6; ++i)
		delay(time);
}
//------------------------------------------------------------------------------------------
void delay_ms(uint32_t time)
{
	DelayMicro(1000*time);
}
//------------------------------------------------------------------------------------------
void delay(uint32_t time)
{		
	uint32_t i;
	for(i = 0; i < time; i++){}
}
//------------------------------------------------------------------------------------------

