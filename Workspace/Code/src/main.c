#include "main.h"

/*		 debug 		

	uint32_t bt1 = 0;
	uint32_t bt2 = 0;
	uint32_t bt3 = 0;
	uint32_t bt4 = 0;
	uint32_t bt5 = 0;
	uint32_t bt6 = 0;

*/
extern struct StateType State;

int main()
{
	RCC_Init();
	GPIO_Init();
	SPI1_Init();	
	SPI2_Init();
	USART3_Init();
	//TIM1_Init();`
	TIM2_Init();
	TIM3_Init();
	TIM4_Init();
	IWDG_Init(1200);

	ResetState();
	SSD1306_Init();

	while(1)
	{
		__disable_irq();
		HandButtonsClicks();
		UpdateScreen();	
		__enable_irq();
		delay_ms(100);
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
		GPIOA->CRH &= ~GPIO_CRH_CNF8;					//Reset CNF register		
		GPIOA->CRH &= ~GPIO_CRH_MODE8;					//Input mode
		GPIOA->CRH |= GPIO_CRH_CNF8_1;					//Input with pull up/pull down
		GPIOA->ODR &= ~GPIO_ODR_ODR8;					//Pull down						

		AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI8;			//Channel EXTI connected to PA 

		EXTI->RTSR |= EXTI_RTSR_TR8;					//Rising trigger enabled for third channel
		EXTI->FTSR |= EXTI_FTSR_TR8;					//Falling trigger enabled for third channel

		EXTI->PR |= EXTI_PR_PR8;						//Clear interrupt flag
		EXTI->IMR |= EXTI_IMR_MR8;						//Enable interrupt

	   //Right button
		GPIOA->CRH &= ~GPIO_CRH_CNF9;					//Reset CNF register				
		GPIOA->CRH &= ~GPIO_CRH_MODE9;					//Input mode
		GPIOA->CRH |= GPIO_CRH_CNF9_1;					//Input with pull up/pull down
		GPIOA->ODR &= ~GPIO_ODR_ODR9;					//Pull down	

		AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI9;			//Channel EXTI connected to PA 

		EXTI->RTSR |= EXTI_RTSR_TR9;					//Rising trigger enabled for third channel
		EXTI->FTSR |= EXTI_FTSR_TR9;					//Falling trigger enabled for third channel

		EXTI->PR |= EXTI_PR_PR9;						//Clear interrupt flag
		EXTI->IMR |= EXTI_IMR_MR9;						//Enable interrupt

	   //Up button
		GPIOA->CRH &= ~GPIO_CRH_CNF10;					//Reset CNF register				
		GPIOA->CRH &= ~GPIO_CRH_MODE10;					//Input mode
		GPIOA->CRH |= GPIO_CRH_CNF10_1;					//Input with pull up/pull down
		GPIOA->ODR &= ~GPIO_ODR_ODR10;					//Pull down	

		AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI10;		//Channel EXTI connected to PA 

		EXTI->RTSR |= EXTI_RTSR_TR10;					//Rising trigger enabled for third channel
		EXTI->FTSR |= EXTI_FTSR_TR10;					//Falling trigger enabled for third channel

		EXTI->PR |= EXTI_PR_PR10;						//Clear interrupt flag
		EXTI->IMR |= EXTI_IMR_MR10;						//Enable interrupt

	   //Down button
		GPIOA->CRH &= ~GPIO_CRH_CNF11;					//Reset CNF register				
		GPIOA->CRH &= ~GPIO_CRH_MODE11;					//Input mode
		GPIOA->CRH |= GPIO_CRH_CNF11_1;					//Input with pull up/pull down
		GPIOA->ODR &= ~GPIO_ODR_ODR11;					//Pull down	

		AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI11;		//Channel EXTI connected to PA 

		EXTI->RTSR |= EXTI_RTSR_TR11;					//Rising trigger enabled for third channel
		EXTI->FTSR |= EXTI_FTSR_TR11;					//Falling trigger enabled for third channel

		EXTI->PR |= EXTI_PR_PR11;						//Clear interrupt flag
		EXTI->IMR |= EXTI_IMR_MR11;						//Enable interrupt

	   //Enter button
		GPIOB->CRL &= ~GPIO_CRL_CNF6;					//Reset CNF register				
		GPIOB->CRL &= ~GPIO_CRL_MODE6;					//Input mode
		GPIOB->CRL |= GPIO_CRL_CNF6_1;					//Input with pull up/pull down
		GPIOB->ODR &= ~GPIO_ODR_ODR6;					//Pull down	

		AFIO->EXTICR[1] &= ~AFIO_EXTICR2_EXTI6;			//Channel EXTI connected to PA 
		AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;		//Channel EXTI connected to PB 

		EXTI->RTSR |= EXTI_RTSR_TR6;					//Rising trigger enabled for third channel
		EXTI->FTSR |= EXTI_FTSR_TR6;					//Falling trigger enabled for third channel

		EXTI->PR |= EXTI_PR_PR6;						//Clear interrupt flag
		EXTI->IMR |= EXTI_IMR_MR6;						//Enable interrupt

	   //Clear button
		GPIOB->CRL &= ~GPIO_CRL_CNF7;					//Reset CNF register				
		GPIOB->CRL &= ~GPIO_CRL_MODE7;					//Input mode
		GPIOB->CRL |= GPIO_CRL_CNF7_1;					//Input with pull up/pull down
		GPIOB->ODR &= ~GPIO_ODR_ODR7;					//Pull down	

		AFIO->EXTICR[1] &= ~AFIO_EXTICR2_EXTI7;			//Channel EXTI connected to PA 
		AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PB;		//Channel EXTI connected to PB 

		EXTI->RTSR |= EXTI_RTSR_TR7;					//Rising trigger enabled for third channel
		EXTI->FTSR |= EXTI_FTSR_TR7;					//Falling trigger enabled for third channel

		EXTI->PR |= EXTI_PR_PR7;						//Clear interrupt flag
		EXTI->IMR |= EXTI_IMR_MR7;						//Enable interrupt

		NVIC_EnableIRQ(EXTI9_5_IRQn);
		NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
	//------------------------------------------------------------------------------------------
	void SPI1_Init(void)
	{
		//Enable clock for SPI1 и GPIOA
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
	void SPI2_Init(void)
	{
		//Enable clock for SPI2 and GPIOB
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; 
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

		/*********************************************/
		/*** Setting pins GPIOB for work with SPI2 ***/
		/*********************************************/
		//PA13 - MOSI
		//PA14 - MISO
		//PA15 - SCK
		
		//First, clear all comfiguration bits
		GPIOB->CRH &= ~GPIO_CRH_CNF13;
		GPIOB->CRH &= ~GPIO_CRH_CNF14;
		GPIOB->CRH &= ~GPIO_CRH_CNF15;
		
		GPIOB->CRH &= ~GPIO_CRH_MODE13;
		GPIOB->CRH &= ~GPIO_CRH_MODE14;
		GPIOB->CRH &= ~GPIO_CRH_MODE15;
		
		//Setting
		//SCK
		GPIOB->CRH |= GPIO_CRH_CNF13_1; 
		GPIOB->CRH |= GPIO_CRH_MODE13;
	
		//MISO
		GPIOB->CRH |= GPIO_CRH_CNF14_0; 
		GPIOB->CRH &= ~GPIO_CRH_MODE14;
	
		//MOSI
		GPIOB->CRH |= GPIO_CRH_CNF15_1; 
		GPIOB->CRH |= GPIO_CRH_MODE15;
		
	
		/**********************/
		/***  Setting SPI2  ***/
		/**********************/
	
		SPI2->CR1 |= SPI_CR1_BR_2;        			//Baud rate: F_PCLK/32
		SPI2->CR1 &= ~SPI_CR1_CPOL;					//Clock polarity SPI: 0
		SPI2->CR1 &= ~SPI_CR1_CPHA;					//Clock phase SPI: 0
		SPI2->CR1 &= ~SPI_CR1_DFF;  				//Data frame format is 8 bit 
		SPI2->CR1 &= ~SPI_CR1_LSBFIRST;    			//MSB first
		SPI2->CR1 |= SPI_CR1_SSM;          			//Software NSS management SS
		SPI2->CR1 |= SPI_CR1_SSI;          			//SS in high level
	
		SPI2->CR2 |= SPI_CR2_RXDMAEN;				//Enable DMA request
		SPI2->CR1 |= SPI_CR1_MSTR;         			//Master mode
		SPI2->CR1 |= SPI_CR1_SPE; 					//Enable SPI	
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
		TIM2->ARR = 100;  										// 
		TIM2->DIER |= TIM_DIER_UIE; 							// Enable tim2 interrupt
		Timer2Disable();  			 							// Stop count

		NVIC_EnableIRQ(TIM2_IRQn); 		 						// Enable IRQ
	}
	//-----------------------------------------------------------------------------------------
	void TIM3_Init(void)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  					// Enable TIM2 Periph clock

		TIM3->PSC = 7199; 										// 10000 tick/sec 	
		TIM3->ARR = 10000;  										// 
		TIM3->DIER |= TIM_DIER_UIE; 							// Enable tim3 interrupt
		NVIC_EnableIRQ(TIM3_IRQn); 		 						// Enable IRQ
		Timer3Enable(); 
	}
	//-----------------------------------------------------------------------------------------
	void TIM4_Init(void)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  					// Enable TIM2 Periph clock

		TIM4->PSC = 7199; 										// 10000 tick/sec 	
		TIM4->ARR = 100;  									// 
		TIM4->DIER |= TIM_DIER_UIE; 							// Enable tim4 interrupt
		Timer4Disable();  

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
	//-----------------------------------------------------------------------------------------
	void IWDG_Init(uint16_t tw)
	{
		// ��� IWDG_PR=7 Tmin=6,4�� RLR=T��*40/256
		IWDG->KR=0x5555; // ���� ��� ������� � �������
		IWDG->PR=7; // ���������� IWDG_PR
		IWDG->RLR=tw*40/256; // ��������� ������� ������������
		IWDG->KR=0xAAAA; // ������������
		IWDG->KR=0xCCCC; // ���� �������
	}
//**************************** Interconnection functions **********************************
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
	void SPI2_Write(uint8_t *pBuff, uint16_t BuffLen)
	{
		for(uint16_t i = 0; i < BuffLen; ++i)
		{
			//Expect while buffer ready
			while(!(SPI2->SR & SPI_SR_TXE)) ;	
			//Send data
			SPI2->DR = pBuff[i];
			//Delay caused display MCU handing ability
			DelayMicro(7);
		}	
	}
	//------------------------------------------------------------------------------------------

//******************************* Protocol's functions ************************************	
	uint16_t MakeCRC16(uint8_t* Buffer, uint16_t Count)
	{
		uint16_t crc = 0xFFFF;
		uint8_t t;

		for (uint16_t ByteNumber = 0; ByteNumber < Count; ++ByteNumber)
		{
			t = Buffer[ByteNumber];
			crc ^= t;          // XOR byte into least sig. byte of crc

			for (int i = 0; i < 8; ++i) {    // Loop over each bit
				if ((crc & 0x0001) == 1){
					crc >>= 1;
					crc ^= 0xA001;
				}
				else
					crc >>= 1;
			}
		}
		return crc;
	}
	//------------------------------------------------------------------------------------------
	void SendMODBUSMessage(uint8_t Function, uint8_t FirstRegister, uint8_t SecondRegister, uint8_t DataID, void* Data)
	{
		static uint8_t SendBuffer[100] = {0};
		static uint16_t SendBufferSize;

		SendBuffer[0] = Function;
		SendBuffer[1] = FirstRegister;
		SendBuffer[2] = SecondRegister;
		SendBuffer[3] = Function;
		
		SendBufferSize = 4;

		switch(DataID)
		{
			case FLOAT_ID :
				SendBufferSize += sprintf((char*)(SendBuffer + SendBufferSize), "%f.5", *((float*)Data));
				break;
			case STRING_ID :
				SendBufferSize += sprintf((char*)(SendBuffer + SendBufferSize), "%s", *((const char **)Data));
				break;
			default:
				break;
		}

		//
	}
	//------------------------------------------------------------------------------------------
	uint8_t HandleMODBUSRequest(uint8_t *Buffer, uint8_t Count)
	{
		if(Count < 5)
			return COUNT_LESS_THAN_MINIMUM;

		uint16_t crc = MakeCRC16(Buffer, Count - 2);

		if(((crc >> 8) != Buffer[Count - 2]) || ((crc & 0x00FF) != Buffer[Count - 1]))
			return CRC_ERROR;

		switch(Buffer[0])
		{
			case READ_MODBUS_FUNCTION :
				if(((Buffer[1] >= GENERAL_PURPOSE_BEGIN_REGISTER) && (Buffer[1] <= GENERAL_PURPOSE_END_REGISTER)) || 
				   ((Buffer[1] >= RESISTORS_DESCRIPTION_BEGIN_REGISTER) && (Buffer[1] <= (RESISTORS_DESCRIPTION_BEGIN_REGISTER + State.ResistorsCount - 1))))
				{
					if(Buffer[1] == GENERAL_PURPOSE_BEGIN_REGISTER)
					{
						//+++++++++++++++++ Handle General Purpose Registers ++++++++++++++++++	
					}
					else if((Buffer[1] >= RESISTORS_DESCRIPTION_BEGIN_REGISTER) && (Buffer[1] <= (RESISTORS_DESCRIPTION_BEGIN_REGISTER + State.ResistorsCount - 1)))
					{
						switch(Buffer[2])
						{
							case NOMINAL_VALUE_REGISTER : 
								SendMODBUSMessage(READ_MODBUS_FUNCTION, Buffer[1], Buffer[2], FLOAT_ID, (&State.R1Nom + Buffer[1] - RESISTORS_DESCRIPTION_BEGIN_REGISTER));
								break;
							case REAL_VALUE_REGISTER : 
								SendMODBUSMessage(READ_MODBUS_FUNCTION, Buffer[1], Buffer[2], FLOAT_ID, (&State.R1 + Buffer[1] - RESISTORS_DESCRIPTION_BEGIN_REGISTER));
								break;
							case POWER_REGISTER : 
								SendMODBUSMessage(READ_MODBUS_FUNCTION, Buffer[1], Buffer[2], FLOAT_ID, (&State.P1 + Buffer[1] - RESISTORS_DESCRIPTION_BEGIN_REGISTER));
								break;
							case ACCURACY_CLASS_REGISTER : 
								SendMODBUSMessage(READ_MODBUS_FUNCTION, Buffer[1], Buffer[2], FLOAT_ID, (&State.R1AccuracyClass + Buffer[1] - RESISTORS_DESCRIPTION_BEGIN_REGISTER));
								break;
							case CATEGORY_REGISTER : 
								SendMODBUSMessage(READ_MODBUS_FUNCTION, Buffer[1], Buffer[2], STRING_ID, (&State.R1Cat + Buffer[1] - RESISTORS_DESCRIPTION_BEGIN_REGISTER));
								break;
							default:
								break;
						}
					}
					else
					{
						//+++++++++++++++++ Handle Register Error ++++++++++++++++++
					}
				}
				else
				{
					//+++++++++++++++++ Handle Register Error ++++++++++++++++++
				}
 				break;
			case WRITE_MODBUS_FUNCTION :
				if(((Buffer[1] >= GENERAL_PURPOSE_BEGIN_REGISTER) && (Buffer[1] <= GENERAL_PURPOSE_END_REGISTER)) || 
				   ((Buffer[1] >= RESISTORS_DESCRIPTION_BEGIN_REGISTER) && (Buffer[1] <= (RESISTORS_DESCRIPTION_BEGIN_REGISTER + State.ResistorsCount - 1))))
				{
					if(Buffer[1] == GENERAL_PURPOSE_BEGIN_REGISTER)
					{

					}
					else if((Buffer[1] >= RESISTORS_DESCRIPTION_BEGIN_REGISTER) && (Buffer[1] <= (RESISTORS_DESCRIPTION_BEGIN_REGISTER + State.ResistorsCount - 1)))
					{
						
					}
					else
					{
						//+++++++++++++++++ Handle Register Error ++++++++++++++++++
					}	
				}
				else
				{
					//+++++++++++++++++ Handle Register Error ++++++++++++++++++
				}
				break;
			default:
				//Buffer[0]
				break;
		} 

		return 0;
	}
	//------------------------------------------------------------------------------------------
	
	//------------------------------------------------------------------------------------------
//********************************** Interrupts handlers ***********************************
	//------------------------------------------------------------------------------------------
	void EXTI9_5_IRQHandler(void)
	{	
		if((EXTI->PR & EXTI_PR_PR6) && (GPIOB->IDR & GPIO_IDR_IDR6))
		{
			State.EnterBtnCounter = 0;
			State.EnterBtnFlag = true;
		}

		if((EXTI->PR & EXTI_PR_PR7) && (GPIOB->IDR & GPIO_IDR_IDR7))
		{
			State.ClearBtnCounter = 0;
			State.ClearBtnFlag = true;
		}

		if((EXTI->PR & EXTI_PR_PR8) && (GPIOA->IDR & GPIO_IDR_IDR8))
		{
			State.LeftBtnCounter = 0;
			State.LeftBtnFlag = true;
		}

		if((EXTI->PR & EXTI_PR_PR9) && (GPIOA->IDR & GPIO_IDR_IDR9))
		{
			State.RightBtnCounter = 0;
			State.RightBtnFlag = true;
		}

		//Reset interrupts
		EXTI->PR |= EXTI_PR_PR6;
		EXTI->PR |= EXTI_PR_PR7;
		EXTI->PR |= EXTI_PR_PR8; 							
		EXTI->PR |= EXTI_PR_PR9; 
		
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
	}
	//------------------------------------------------------------------------------------------
	void EXTI15_10_IRQHandler(void)
	{
		if((EXTI->PR & EXTI_PR_PR10) && (GPIOA->IDR & GPIO_IDR_IDR10))
		{
			State.UpBtnCounter = 0;
			State.UpBtnFlag = true;
		}

		if((EXTI->PR & EXTI_PR_PR11) && (GPIOA->IDR & GPIO_IDR_IDR11))
		{
			State.DownBtnCounter = 0;
			State.DownBtnFlag = true;
		}
		
		//Reset interrupts
		EXTI->PR |= EXTI_PR_PR10; 						
		EXTI->PR |= EXTI_PR_PR11; 						
		
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
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
	}
	//------------------------------------------------------------------------------------------
	void TIM2_IRQHandler(void)
	{
		TIM2->SR &= ~TIM_SR_UIF; 
		TIM2->CNT = 100;

		if(State.LeftBtnDownFlag && !(GPIOA->IDR & GPIO_IDR_IDR8) && ((--State.LeftBtnCounter) <= 0))
		{
				State.LeftBtnFlag = false;
				State.LeftBtn = true;
				State.LeftBtnDownFlag = false; 
				State.LeftBtnCounter = 0;
		}
		else if(State.LeftBtnDownFlag && (GPIOA->IDR & GPIO_IDR_IDR8))
		{
			State.LeftBtnCounter = BUTTON_ITERATION_DOWN_COUNT;	
		}

		if(State.RightBtnDownFlag && !(GPIOA->IDR & GPIO_IDR_IDR9) && ((--State.RightBtnCounter) <= 0))
		{
			State.RightBtnFlag = false;
			State.RightBtn = true;
			State.RightBtnDownFlag = false; 
			State.RightBtnCounter = 0;
		}
		else if(State.RightBtnDownFlag && (GPIOA->IDR & GPIO_IDR_IDR9))
		{
			State.RightBtnCounter = BUTTON_ITERATION_DOWN_COUNT;	
		}

		if(State.UpBtnDownFlag && !(GPIOA->IDR & GPIO_IDR_IDR10) && ((--State.UpBtnCounter) <= 0))
		{
			State.UpBtnFlag = false;
			State.UpBtn = true;
			State.UpBtnDownFlag = false; 
			State.UpBtnCounter = 0;
		}
		else if(State.RightBtnDownFlag && (GPIOA->IDR & GPIO_IDR_IDR10))
		{
			State.UpBtnCounter = BUTTON_ITERATION_DOWN_COUNT;		
		}

		if(State.DownBtnDownFlag && !(GPIOA->IDR & GPIO_IDR_IDR11) && ((--State.DownBtnCounter) <= 0))
		{
			State.DownBtnFlag = false;
			State.DownBtn = true;
			State.DownBtnDownFlag = false; 
			State.DownBtnCounter = 0;
		}
		else if(State.DownBtnDownFlag && (GPIOA->IDR & GPIO_IDR_IDR11))
		{
			State.DownBtnCounter = BUTTON_ITERATION_DOWN_COUNT;		
		}

		if(State.EnterBtnDownFlag && !(GPIOB->IDR & GPIO_IDR_IDR6) && ((--State.EnterBtnCounter) <= 0))
		{
			State.EnterBtnFlag = false;
			State.EnterBtn = true;
			State.EnterBtnDownFlag = false; 
			State.EnterBtnCounter = 0;
		}
		else if(State.EnterBtnDownFlag && (GPIOB->IDR & GPIO_IDR_IDR6))
		{
			State.EnterBtnCounter = BUTTON_ITERATION_DOWN_COUNT;		
		}

		if(State.ClearBtnDownFlag && !(GPIOB->IDR & GPIO_IDR_IDR7) && ((--State.ClearBtnCounter) <= 0))
		{
			State.ClearBtnFlag = false;
			State.ClearBtn = true;
			State.ClearBtnDownFlag = false;
			State.ClearBtnCounter = 0;
		}
		else if(State.ClearBtnDownFlag && (GPIOB->IDR & GPIO_IDR_IDR7))
		{
			State.ClearBtnCounter = BUTTON_ITERATION_DOWN_COUNT;		
		}
		
	}
	//------------------------------------------------------------------------------------------
	void TIM3_IRQHandler(void)
	{
		TIM3->SR &= ~TIM_SR_UIF; 

		State.ModeRectangle = !State.ModeRectangle;

		//������������ ����������� ������� IWDG
		IWDG->KR=0xAAAA; // ������������
	}
	//------------------------------------------------------------------------------------------
	void TIM4_IRQHandler(void)
	{
		TIM4->SR &= ~TIM_SR_UIF; 
		TIM4->CNT = 100;

		if(State.LeftBtnFlag && (GPIOA->IDR & GPIO_IDR_IDR8) && ((++State.LeftBtnCounter) > BUTTON_ITERATION_COUNT))
		{
			State.LeftBtnDownFlag = true;
			State.LeftBtnFlag = false;
			State.LeftBtnCounter = BUTTON_ITERATION_DOWN_COUNT;
		}
		else if(State.LeftBtnFlag && !(GPIOA->IDR & GPIO_IDR_IDR8))
		{
			State.LeftBtnFlag = false;	
			State.LeftBtnCounter = 0;
			State.LeftBtnDownFlag = false;
		}

		if(State.RightBtnFlag && (GPIOA->IDR & GPIO_IDR_IDR9) && ((++State.RightBtnCounter) > BUTTON_ITERATION_COUNT))
		{
			State.RightBtnDownFlag = true;
			State.RightBtnFlag = false;
			State.RightBtnCounter = BUTTON_ITERATION_DOWN_COUNT;
		}
		else if(State.RightBtnFlag && !(GPIOA->IDR & GPIO_IDR_IDR9))
		{
			State.RightBtnFlag = false;	
			State.RightBtnCounter = 0;
			State.RightBtnDownFlag = false;
		}

		if(State.UpBtnFlag && (GPIOA->IDR & GPIO_IDR_IDR10) && ((++State.UpBtnCounter) > BUTTON_ITERATION_COUNT))
		{
			State.UpBtnDownFlag = true;
			State.UpBtnFlag = false;
			State.RightBtnCounter = BUTTON_ITERATION_DOWN_COUNT;
		}
		else if(State.UpBtnFlag && !(GPIOA->IDR & GPIO_IDR_IDR10))
		{
			State.UpBtnFlag = false;	
			State.UpBtnCounter = 0;
			State.UpBtnDownFlag = false;
		}

		if(State.DownBtnFlag && (GPIOA->IDR & GPIO_IDR_IDR11) && ((++State.DownBtnCounter) > BUTTON_ITERATION_COUNT))
		{
			State.DownBtnDownFlag = true;
			State.DownBtnFlag = false;
			State.RightBtnCounter = BUTTON_ITERATION_DOWN_COUNT;
		}
		else if(State.DownBtnFlag && !(GPIOA->IDR & GPIO_IDR_IDR11))
		{
			State.DownBtnFlag = false;	
			State.DownBtnCounter = 0;
			State.DownBtnDownFlag = false;
		}

		if(State.EnterBtnFlag && (GPIOB->IDR & GPIO_IDR_IDR6) && ((++State.EnterBtnCounter) > BUTTON_ITERATION_COUNT))
		{
			State.EnterBtnDownFlag = true;
			State.EnterBtnFlag = false;
			State.RightBtnCounter = BUTTON_ITERATION_DOWN_COUNT;
		}
		else if(State.EnterBtnFlag && !(GPIOB->IDR & GPIO_IDR_IDR6))
		{
			State.EnterBtnFlag = false;	
			State.EnterBtnCounter = 0;
			State.EnterBtnDownFlag = false;
		}

		if(State.ClearBtnFlag && (GPIOB->IDR & GPIO_IDR_IDR7) && ((++State.ClearBtnCounter) > BUTTON_ITERATION_COUNT))
		{
			State.ClearBtnDownFlag = true;
			State.ClearBtnFlag = false;
			State.RightBtnCounter = BUTTON_ITERATION_DOWN_COUNT;
		}
		else if(State.ClearBtnFlag && !(GPIOB->IDR & GPIO_IDR_IDR7))
		{
			State.ClearBtnFlag = false;	
			State.ClearBtnCounter = 0;
			State.ClearBtnDownFlag = false;
		}
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
	void UpdateScreen(void)
	{
		disp1color_FillScreenbuff(0);

		switch(State.CurrentPageNumber)
		{
			case FIRST_PAGE :
				DisplayHead(0);

				//Draw temperature
				disp1color_printf(28, 23, FONTID_10X16F, "%.3f %cC", State.Temperature, 0x80);

				//Debug
				//disp1color_printf(0, 46, FONTID_6X8M, " %d|%d|%d|%d|%d|%d  ", bt1, bt2, bt3, bt4, bt5, bt6); 

				//Draw bottom separator
				disp1color_DrawLine(0, 54, 127, 54);

				//Draw bottom writting with triangles
				disp1color_printf(PAGE1_BOTTOM_WRITTING_LEFT, 57, FONTID_6X8M, "%c R%d: %g ��", 0x82, (State.CurrentPage1BottomResistor + 1), *(&State.R1 + State.CurrentPage1BottomResistor));
				break;
			case SECOND_PAGE :
				DisplayHead(1);

				if((State.ResistorsCount <= 4) && (State.CurrentPage2ResistorsSet > 0))
					State.CurrentPage2ResistorsSet = 0;

				disp1color_printf(PAGE2_RESISTORS_LEFT, PAGE2_RESISTORS_TOP, FONTID_6X8M, "R%d: %g ��", (State.CurrentPage2ResistorsSet + 1), *(&State.R1 + State.CurrentPage2ResistorsSet));  
				disp1color_printf(PAGE2_RESISTORS_LEFT, PAGE2_RESISTORS_TOP + 1*PAGE2_RESISTORS_VERTICAL_STEP, FONTID_6X8M, "R%d: %g ��", (State.CurrentPage2ResistorsSet + 2), *(&State.R1 + 1 + State.CurrentPage2ResistorsSet));
				disp1color_printf(PAGE2_RESISTORS_LEFT, PAGE2_RESISTORS_TOP + 2*PAGE2_RESISTORS_VERTICAL_STEP, FONTID_6X8M, "R%d: %g ��", (State.CurrentPage2ResistorsSet + 3), *(&State.R1 + 2 + State.CurrentPage2ResistorsSet));
				disp1color_printf(PAGE2_RESISTORS_LEFT, PAGE2_RESISTORS_TOP + 3*PAGE2_RESISTORS_VERTICAL_STEP, FONTID_6X8M, "R%d: %g ��", (State.CurrentPage2ResistorsSet + 4), *(&State.R1 + 3 + State.CurrentPage2ResistorsSet));
				if(State.ResistorsCount > 4)
					disp1color_printf(PAGE2_RESISTORS_LEFT, PAGE2_RESISTORS_TOP + 4*PAGE2_RESISTORS_VERTICAL_STEP, FONTID_6X8M, "R%d: %g ��", (State.CurrentPage2ResistorsSet + 5) , *(&State.R1 + 4 + State.CurrentPage2ResistorsSet));

				if(State.ChosenPage2Resistor > 4)
					State.ChosenPage2Resistor = 4;
				
				//Draw choose rectangle
				disp1color_DrawRectangle(PAGE2_RESISTORS_LEFT - PAGE2_RESISTORS_LEFT_RECTANGLES_OFFSET, 
										 PAGE2_RESISTORS_TOP + 2 + State.ChosenPage2Resistor*PAGE2_RESISTORS_VERTICAL_STEP, 
										 PAGE2_RESISTORS_LEFT - PAGE2_RESISTORS_LEFT_RECTANGLES_OFFSET + 4, 
										 PAGE2_RESISTORS_TOP + 4 + State.ChosenPage2Resistor*PAGE2_RESISTORS_VERTICAL_STEP);
				
				//Draw top triangle
				if((State.ChosenPage2Resistor > 0) || (State.CurrentPage2ResistorsSet > 0))
					disp1color_printf(PAGE2_RESISTORS_LEFT - PAGE2_RESISTORS_LEFT_TRIANGLES_OFFSET, PAGE2_RESISTORS_TOP, FONTID_6X8M, "%c", 0x83);
				
				//Draw bottom triangle
				if((((State.ChosenPage2Resistor < 4) || (State.CurrentPage2ResistorsSet < 4)) && (State.ResistorsCount == 9)) || ((State.ChosenPage2Resistor < 3) && (State.ResistorsCount == 4)))
					disp1color_printf(PAGE2_RESISTORS_LEFT - PAGE2_RESISTORS_LEFT_TRIANGLES_OFFSET, PAGE2_RESISTORS_TOP + 4*PAGE2_RESISTORS_VERTICAL_STEP, FONTID_6X8M, "%c", 0x84);
				
				break;
			case THIRD_PAGE :
				DisplayHead(1);

				disp1color_printf(3, 15, FONTID_6X8M, "���������:");
				disp1color_printf(10, 26, FONTID_6X8M, "�������:  %d%%", State.Brigtness);
				disp1color_printf(10, 37, FONTID_6X8M, "������:     ********", State.Brigtness);
				break;
			case RESISTORS_INFO_PAGE :
				switch(State.ChosenPage2Resistor + State.CurrentPage2ResistorsSet)
				{
					case 0:
						DisplayResistorInfoPage(State.R1Nom, State.R1, State.P1, State.R1AccuracyClass, State.R1Cat);
						break;
					case 1:
						DisplayResistorInfoPage(State.R2Nom, State.R2, State.P2, State.R2AccuracyClass, State.R2Cat);
						break;
					case 2:
						DisplayResistorInfoPage(State.R3Nom, State.R3, State.P3, State.R3AccuracyClass, State.R3Cat);
						break;
					case 3:
						DisplayResistorInfoPage(State.R4Nom, State.R4, State.P4, State.R4AccuracyClass, State.R4Cat);
						break;
					case 4:
						DisplayResistorInfoPage(State.R5Nom, State.R5, State.P5, State.R5AccuracyClass, State.R5Cat);
						break;
					case 5:
						DisplayResistorInfoPage(State.R6Nom, State.R6, State.P6, State.R6AccuracyClass, State.R6Cat);
						break;
					case 6:
						DisplayResistorInfoPage(State.R7Nom, State.R7, State.P7, State.R7AccuracyClass, State.R7Cat);
						break;
					case 7:
						DisplayResistorInfoPage(State.R8Nom, State.R8, State.P8, State.R8AccuracyClass, State.R8Cat);
						break;
					case 8:
						DisplayResistorInfoPage(State.R9Nom, State.R9, State.P9, State.R9AccuracyClass, State.R9Cat);
						break;
					default:
						break;
				}
				break;
			case 5 :

				break;
			default:
				break;
		}
		disp1color_UpdateFromBuff();
	}
	//------------------------------------------------------------------------------------------
	void DisplayHead(uint8_t HeadType)
	{
		switch(HeadType)
		{
			case HEAD_WORK_MODE :
				if(State.ReadyToWork)
				{
					disp1color_printf(0, 1, FONTID_6X8M,  "             �����  �  ������\n\r");
					disp1color_printf(118, 1, FONTID_6X8M, "%c", 0x81);
				}
				else
				{
					disp1color_printf(0, 1, FONTID_6X8M,  "             �����  ��  �����\n\r");
					disp1color_printf(118, 1, FONTID_6X8M, "x");
				}
				break;
			case HEAD_TEMPERATURE :
				disp1color_printf(0, 1, FONTID_6X8M,  "               T: %.3f %cC\n\r",  State.Temperature, 0x80);
				if(State.ReadyToWork)
					disp1color_printf(118, 1, FONTID_6X8M, "%c", 0x81);
				else
					disp1color_printf(118, 1, FONTID_6X8M, "x");
				break;
		}

		if(State.ModeRectangle)
			disp1color_DrawRectangle(MODE_RECT_L - 1, MODE_RECT_T - 1, MODE_RECT_L + 3, MODE_RECT_T + 3);	
		else
			disp1color_DrawRectangle(MODE_RECT_L, MODE_RECT_T, MODE_RECT_L + 2, MODE_RECT_T + 2);
		disp1color_DrawLine(0, 10, 127, 10);	
	}
	//------------------------------------------------------------------------------------------
	void HandButtonsClicks(void)
	{
		if(State.LeftBtnDownFlag || State.RightBtnDownFlag || State.UpBtnDownFlag || State.DownBtnDownFlag || State.EnterBtnDownFlag || State.ClearBtnDownFlag)
		{
			Timer2Enable();
		}

		if(State.LeftBtnFlag || State.RightBtnFlag || State.UpBtnFlag || State.DownBtnFlag || State.EnterBtnFlag || State.ClearBtnFlag)
		{
			Timer4Enable();
		}

		if(State.LeftBtn)
		{
			//++bt1;
			if((State.CurrentPageNumber > 0) && (State.CurrentPageNumber < 3))
			{
				if(!State.EditingMode)
					--State.CurrentPageNumber;
			}
			else if(State.CurrentPageNumber == 4)
			{
				if(!State.EditingMode)
					State.CurrentPageNumber = 1;
			}
		}
			
		if(State.RightBtn)
		{
			//++bt2;
			if(!State.EditingMode)
				if(State.CurrentPageNumber < 2)
					++State.CurrentPageNumber;

			if(State.CurrentPageNumber == 4)
			{
				if(!State.EditingMode)
					State.CurrentPageNumber = 2;
			}
		}

		if(State.UpBtn)
		{
			//++bt3;
			if(State.CurrentPageNumber == 0)
			{
				if(State.CurrentPage1BottomResistor > 0)
				{
					--State.CurrentPage1BottomResistor;	
				}	
			}
			else if(State.CurrentPageNumber == 1)
			{
				if(State.ChosenPage2Resistor > 0)
					--State.ChosenPage2Resistor;
				else if((State.ResistorsCount == 9) && (State.CurrentPage2ResistorsSet > 0))
				{
					--State.CurrentPage2ResistorsSet;
				}
			}
			else if(State.CurrentPageNumber == 4)
			{
				if(!State.EditingMode)
				{
					if(State.CurrentPage4EditingValue > 0)	
					{
						--State.CurrentPage4EditingValue;	
					}
				}
			}
		}
			
		if(State.DownBtn)
		{
			//++bt4;
			if(State.CurrentPageNumber == 0)
			{
				if(State.CurrentPage1BottomResistor < 8)
				{
					++State.CurrentPage1BottomResistor;	
				}		
			}
			else if(State.CurrentPageNumber == 1)
			{
				if(((State.ChosenPage2Resistor < 4) && (State.ResistorsCount == 9)) || ((State.ChosenPage2Resistor < 3) && (State.ResistorsCount == 4)))
					++State.ChosenPage2Resistor;	
				else if((State.ResistorsCount == 9) && (State.CurrentPage2ResistorsSet < 4))
				{
					++State.CurrentPage2ResistorsSet;
				}	
			}
			else if(State.CurrentPageNumber == 4)
			{
				if(!State.EditingMode)
				{
					if(State.CurrentPage4EditingValue < (PAGE4_VALUES_COUNT- 1))	
					{
						++State.CurrentPage4EditingValue;	
					}
				}
			}
		}
		if(State.EnterBtn)
		{
			//++bt5;
			if(State.CurrentPageNumber == 1)
			{
				State.CurrentPageNumber = 4;
			}
		}
			
		if(State.ClearBtn)
		{
			//++bt6;

			if((State.CurrentPageNumber == 4) && (!State.EditingMode))
			{
				State.CurrentPageNumber = 1;
			}

		}

		//Reset buttons state
		State.LeftBtn = false;
		State.RightBtn = false;
		State.UpBtn = false;
		State.DownBtn = false;
		State.EnterBtn = false;
		State.ClearBtn = false;
		
		if(!State.LeftBtnFlag && !State.RightBtnFlag && !State.UpBtnFlag && !State.DownBtnFlag && !State.EnterBtnFlag && !State.ClearBtnFlag)
		{
			Timer4Disable();	
		}	

		if(!State.LeftBtnDownFlag && !State.RightBtnDownFlag && !State.UpBtnDownFlag && !State.DownBtnDownFlag && !State.EnterBtnDownFlag && !State.ClearBtnDownFlag)
		{
			Timer2Disable();  
		}	
	}
	//------------------------------------------------------------------------------------------
	void DisplayResistorInfoPage(float RNom, float R, float P, float RAC, const char *RCat)
	{
		DisplayHead(1);

		disp1color_printf(PAGE4_VALUES_LEFT, PAGE4_VALUES_TOP, FONTID_6X8M,  " R���:  %g ��", RNom);
		disp1color_printf(PAGE4_VALUES_LEFT, PAGE4_VALUES_TOP + PAGE4_VALUES_VERTICAL_STEP, FONTID_6X8M,  " R�:        %g ��", R);
		disp1color_printf(PAGE4_VALUES_LEFT, PAGE4_VALUES_TOP + 2*PAGE4_VALUES_VERTICAL_STEP, FONTID_6X8M,  " P���:  %g ��", P);
		disp1color_printf(PAGE4_VALUES_LEFT, PAGE4_VALUES_TOP + 3*PAGE4_VALUES_VERTICAL_STEP, FONTID_6X8M,  " �.�.:  %g", RAC);
		disp1color_printf(PAGE4_VALUES_LEFT, PAGE4_VALUES_TOP + 4*PAGE4_VALUES_VERTICAL_STEP, FONTID_6X8M,  " ������: %s", RCat);
		
		//Draw choose rectangle
		disp1color_DrawRectangle(PAGE4_VALUES_LEFT - PAGE4_VALUES_LEFT_RECTANGLES_OFFSET, 
								 PAGE4_VALUES_TOP + 2 + State.CurrentPage4EditingValue*PAGE4_VALUES_VERTICAL_STEP, 
								 PAGE4_VALUES_LEFT - PAGE4_VALUES_LEFT_RECTANGLES_OFFSET + 4, 
								 PAGE4_VALUES_TOP + 4 + State.CurrentPage4EditingValue*PAGE4_VALUES_VERTICAL_STEP);

		//Draw top triangle
		if(State.CurrentPage4EditingValue > 0)
			disp1color_printf(PAGE4_VALUES_LEFT - PAGE4_VALUES_LEFT_TRIANGLES_OFFSET, PAGE4_VALUES_TOP, FONTID_6X8M, "%c", 0x83);
				
		//Draw bottom triangle
		if(State.CurrentPage4EditingValue < (PAGE4_VALUES_COUNT - 1))
			disp1color_printf(PAGE4_VALUES_LEFT - PAGE4_VALUES_LEFT_TRIANGLES_OFFSET, PAGE4_VALUES_TOP + 4*PAGE4_VALUES_VERTICAL_STEP, FONTID_6X8M, "%c", 0x84);
		
		
		//disp1color_printf(100, 54, FONTID_6X8M, "����");
		//disp1color_DrawRectangle(97, 51, 125, 63);
	}
