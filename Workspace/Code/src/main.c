#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdbool.h"

//Initialization functions
void GPIO_Init(void);
void RCC_Init(void);
void MCO_out (void);
void SPI1_Init(void);

//Interconnection functions
void SPI1_Write(uint16_t data);

//User functions
void delay(uint32_t time);	

//FreeRTOS tasks
void vTaskLed(void *argument);


int main()
{
	//RCC_Init();
	GPIO_Init();
	MCO_out();
	SPI1_Init();

	xTaskCreate(vTaskLed, "LED1", 32, NULL, 1, NULL);

	vTaskStartScheduler();
	
	while(1)
	{
		//GPIOA->BSRR |= GPIO_BSRR_BS9;//Set 9-th pin to 1
		//delay(10);						//task sleep
		//GPIOA->BSRR |= GPIO_BSRR_BR9;//Set 9-th pin to 0
		//delay(10);	
	}
}
//*************************************************************************************************
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
void MCO_out (void){
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                       	// enable clock for port A

	GPIOA->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1);				// gpio speed 50 MHz
	
	GPIOA->CRH &= ~GPIO_CRH_CNF8_0;															// setting out alternative push-pull for PA8
	GPIOA->CRH |= GPIO_CRH_CNF8_1;

	
	RCC->CFGR |= RCC_CFGR_MCO_HSE;														// select source clock SYSCLK
	
}
//-----------------------------------------------------------------------------------------
void GPIO_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                       	// enable clock for port A
	
	GPIOA->CRH &= ~GPIO_CRH_CNF9;				//
	GPIOA->CRH |= GPIO_CRH_MODE9_0;			//
}
//------------------------------------------------------------------------------------------
void SPI1_Init(void)
{
	//???????? ???????????? SPI1 ? GPIOA
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  /**********************************************************/
  /*** ????????? ??????? GPIOA ?? ?????? ????????? ? SPI1 ***/
  /**********************************************************/
  //PA7 - MOSI
  //PA6 - MISO
  //PA5 - SCK
	
	//??? ?????? ?????????? ??? ???????????????? ???? ? ????
  GPIOA->CRL &= ~GPIO_CRL_CNF5;
	GPIOA->CRL &= ~GPIO_CRL_CNF6;
	GPIOA->CRL &= ~GPIO_CRL_CNF7;
	
	GPIOA->CRL &= ~GPIO_CRL_MODE5;
	GPIOA->CRL &= ~GPIO_CRL_MODE5;
	GPIOA->CRL &= ~GPIO_CRL_CNF5;
	
	
	//???????????
  //SCK: MODE5 = 0x03 (11b); CNF5 = 0x02 (10b)
  GPIOA->CRL |= GPIO_CRL_CNF5_1; 
	GPIOA->CRL |= GPIO_CRL_MODE5;
  
  //MISO: MODE6 = 0x00 (00b); CNF6 = 0x01 (01b)
  GPIOA->CRL |= GPIO_CRL_CNF6_0; 
	GPIOA->CRL &= ~GPIO_CRL_MODE6;
  
  //MOSI: MODE7 = 0x03 (11b); CNF7 = 0x02 (10b)
  GPIOA->CRL |= GPIO_CRL_CNF7_1; 
	GPIOA->CRL |= GPIO_CRL_MODE7;
	
  
  /**********************/
  /*** ????????? SPI1 ***/
  /**********************/
  
	SPI1->CR1 |= SPI_CR1_BR_2;        		//???????? ????????: F_PCLK/32
	SPI1->CR1 &= ~SPI_CR1_CPOL;						//????? ?????? SPI: 0
	SPI1->CR1 &= ~SPI_CR1_CPHA;
  SPI1->CR1 &= ~SPI_CR1_DFF;  					//?????? ????? 8 ???
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST;    		//MSB first
  SPI1->CR1 |= SPI_CR1_SSM;          		//??????????? ?????????? SS
  SPI1->CR1 |= SPI_CR1_SSI;          		//SS ? ??????? ?????????
  
  SPI1->CR1 |= SPI_CR1_MSTR;         		//????? Master (???????)
   

	NVIC_EnableIRQ(SPI1_IRQn); //????????? ?????????? ?? SPI1
  
  SPI1->CR1 |= SPI_CR1_SPE; //???????? SPI
	
}
//------------------------------------------------------------------------------------------
void SPI1_IRQHandler(void)
{
 /* SPI1->DR = tr_buf[tr_num++]; //?????????? ????? ???????? ? DR
  
  //???? ??? ????????, ?? ????????? ??????????,
  //??? ????? ????????? ???????? ??????
	if(tr_num > 3)
	{
		tr_num = 0;
    SPI1->CR2 &= ~SPI_CR2_TXEIE;
	}
	
	USART1_Send_String("SPI writen");*/
}
//-----------------------------------------------------------------------------------------
void SPI1_Write(uint16_t data)
{
  //????, ???? ?? ??????????? ????? ???????????
  while(!(SPI1->SR & SPI_SR_TXE))
    ;
	
  //????????? ????? ???????????
  SPI1->DR = data;
}
//*************************************************************************************************
void vTaskLed(void *argument)
{
	while(1)
	{
		GPIOA->BSRR |= GPIO_BSRR_BS9;//Set 9-th pin to 1
		SPI1_Write('A');
		SPI1_Write('B');
		SPI1_Write('C');
		SPI1_Write('D');
		vTaskDelay(500);							//task sleep
		GPIOA->BSRR |= GPIO_BSRR_BR9;//Set 9-th pin to 0
		vTaskDelay(500);	
	}
}
//*************************************************************************************************
void delay(uint32_t time){
		
	uint32_t i;
	for(i = 0; i < time; i++){
	}
}
