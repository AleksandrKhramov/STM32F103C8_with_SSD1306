#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdbool.h"

#ifndef mainH
#define mainH
//Initialization functions
void GPIO_Init(void);
void RCC_Init(void);
void SPI1_Init(void);

//Interconnection functions
SPI1_Write(uint8_t *pBuff, uint16_t BuffLen);

//Common purpose functions

//SSD1306 functions
void SSD1306_GPIO_init(void);

//FreeRTOS tasks
//void vTaskLed(void *argument);
#endif