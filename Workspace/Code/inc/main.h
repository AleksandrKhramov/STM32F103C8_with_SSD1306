#ifndef mainH
#define mainH

#include "stm32f10x.h"
#include "stdbool.h"
#include "SSD1306.h"
#include "disp1color.h"
#include "font.h"


#define SEL_RECT_L                  10
#define SEL_RECT_T                  16
#define MODE_RECT_L                 3
#define MODE_RECT_T                 3


//Initialization functions
void GPIO_Init(void);
void RCC_Init(void);
void SPI1_Init(void);

//Interconnection functions
void SPI1_Write(uint8_t *pBuff, uint16_t BuffLen);

//User functions
void delay(uint32_t time);
void DelayMicro(uint32_t time);
void delay_ms(uint32_t time);
	
//Common purpose functions

//SSD1306 functions
void SSD1306_GPIO_init(void);

#endif
