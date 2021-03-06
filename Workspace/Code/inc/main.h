#ifndef mainH
#define mainH

#include "stm32f10x.h"
#include "stdbool.h"
#include "SSD1306.h"
#include "disp1color.h"
#include "font.h"
#include "state.h"
#include <stdio.h>                    

#define MODE_RECT_L                             3
#define MODE_RECT_T                             3

#define BUTTON_ITERATION_COUNT                  3
#define BUTTON_ITERATION_DOWN_COUNT             50

//Page1
#define FIRST_PAGE                              0
#define PAGE1_BOTTOM_WRITTING_LEFT              30

//Page2
#define SECOND_PAGE                             1
#define PAGE2_RESISTORS_LEFT                    25
#define PAGE2_RESISTORS_LEFT_RECTANGLES_OFFSET  10
#define PAGE2_RESISTORS_LEFT_TRIANGLES_OFFSET   20
#define PAGE2_RESISTORS_TOP                     13
#define PAGE2_RESISTORS_VERTICAL_STEP           11

//Page3
#define THIRD_PAGE                              2

//Page4
#define RESISTORS_INFO_PAGE                     4
#define PAGE4_VALUES_LEFT                       25
#define PAGE4_VALUES_LEFT_RECTANGLES_OFFSET     10
#define PAGE4_VALUES_TOP                        14
#define PAGE4_VALUES_VERTICAL_STEP              10
#define PAGE4_VALUES_COUNT                      5
#define PAGE4_VALUES_LEFT_TRIANGLES_OFFSET      20

//Head types
#define HEAD_WORK_MODE                          0
#define HEAD_TEMPERATURE                        1 

#define Timer2Enable();                         TIM2->CR1 |= TIM_CR1_CEN;
#define Timer2Disable();                        TIM2->CR1 &= ~TIM_CR1_CEN;
#define IsTimer2Enabled()                       (TIM2->CR1 & TIM_CR1_CEN)

#define Timer3Enable();                         TIM3->CR1 |= TIM_CR1_CEN;
#define Timer3Disable();                        TIM3->CR1 &= ~TIM_CR1_CEN;
#define IsTimer3Enabled()                       (TIM3->CR1 & TIM_CR1_CEN)

#define Timer4Enable();                         TIM4->CR1 |= TIM_CR1_CEN;
#define Timer4Disable();                        TIM4->CR1 &= ~TIM_CR1_CEN;
#define IsTimer4Enabled()                       (TIM4->CR1 & TIM_CR1_CEN)

//Modbus Handle errors
#define COUNT_LESS_THAN_MINIMUM                 1
#define CRC_ERROR                               2

//Modbus functions
#define READ_MODBUS_FUNCTION                    0x03
#define WRITE_MODBUS_FUNCTION                   0x06    

//Modbus first Registers 
#define GENERAL_PURPOSE_BEGIN_REGISTER          0x01
#define GENERAL_PURPOSE_END_REGISTER            0x01
#define RESISTORS_DESCRIPTION_BEGIN_REGISTER    0x10
#define RESISTORS_DESCRIPTION_END_REGISTER      0x18

//Modbus second Registers 
#define NOMINAL_VALUE_REGISTER                  0x00      
#define REAL_VALUE_REGISTER                     0x01
#define POWER_REGISTER                          0x02
#define ACCURACY_CLASS_REGISTER                 0x03
#define CATEGORY_REGISTER                       0x04

//MODBUS Data ID 
#define FLOAT_ID                                0x00
#define STRING_ID                               0x01

//Initialization functions
void GPIO_Init(void);
void RCC_Init(void);
void SPI1_Init(void);
void SPI2_Init(void);
void USART3_Init(void);
void TIM1_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);
void IWDG_Init(uint16_t tw);

//Interconnection functions
void SPI1_Write(uint8_t *pBuff, uint16_t BuffLen);

//Working functions
void delay(uint32_t time);
void DelayMicro(uint32_t time);
void delay_ms(uint32_t time);
void UpdateScreen(void);
void DisplayHead(uint8_t HeadType);
void DisplayResistorInfoPage(float RNom, float R, float P, float RAC, const char *RCat);
void HandButtonsClicks(void);

	
//SSD1306 functions
void SSD1306_GPIO_init(void);

#endif
