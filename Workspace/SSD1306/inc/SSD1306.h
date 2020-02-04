#include "main.h"

#ifndef SSD1306H
#define SSD1306H

#define STM32F100


//Макроподстановки пинов контроллера
#ifdef STM32F100
    #define SSD1306_RESET_HIGH();               GPIOA->BSRR |= GPIO_BSRR_BS3;
    #define SSD1306_RESET_LOW();                GPIOA->BSRR |= GPIO_BSRR_BR3;

    #define SSD1306_DC_HIGH();                  GPIOA->BSRR |= GPIO_BSRR_BS1;
    #define SSD1306_DC_LOW();                   GPIOA->BSRR |= GPIO_BSRR_BR1;

    #define SSD1306_CS_HIGH();                  GPIOA->BSRR |= GPIO_BSRR_BS2;
    #define SSD1306_CS_LOW();                   GPIOA->BSRR |= GPIO_BSRR_BR2;  
#elif STM32F103
    #define SSD1306_RESET_HIGH();               GPIOA->BSRR |= GPIO_BSRR_BS3;
    #define SSD1306_RESET_LOW();                GPIOA->BSRR |= GPIO_BSRR_BR3;

    #define SSD1306_DC_HIGH();                  GPIOA->BSRR |= GPIO_BSRR_BS1;
    #define SSD1306_DC_LOW();                   GPIOA->BSRR |= GPIO_BSRR_BR1;

    #define SSD1306_CS_HIGH();                  GPIOA->BSRR |= GPIO_BSRR_BS2;
    #define SSD1306_CS_LOW();                   GPIOA->BSRR |= GPIO_BSRR_BR2; 
#else
    #error Требуется выбрать контроллер
#endif

//==============================================================================
// Параметры дисплея
//==============================================================================

#define SSD1306_Width 128
#define SSD1306_Height 64

//==============================================================================
// Коды команд дисплея
//==============================================================================
// Команды из раздела Fundamental
#define SSD1306_CMD_SetContrast                 0x81
#define SSD1306_CMD_AllPixRAM                   0xA4
#define SSD1306_CMD_AllPixOn                    0xA5
#define SSD1306_CMD_SetInverseOff               0xA6
#define SSD1306_CMD_SetInverseOn                0xA7
#define SSD1306_CMD_Sleep                       0xAE
#define SSD1306_CMD_Wake                        0xAF

// Команды из раздела Scrolling
#define SSD1306_CMD_DeactivateScroll            0x2E

// Команды из раздела Addressing Setting
// Команда выбора режима автосдвига указателя в памяти кадра
#define SSD1306_CMD_SetMemAdressingMode         0x20    
// Команды выбора диапазона изменения страницы и колонки при автосдвиге указателя в памяти кадра
// Применяется для режимов автосдвига SSD1306_Adressing_Horizontal и SSD1306_Adressing_Vertical
#define SSD1306_CMD_SetColumnAddr               0x21
#define SSD1306_CMD_SetPageAddr                 0x22
// Команды выбора страницы и диапазона изменения колонки при автосдвиге указателя в памяти кадра
// Применяется для режима автосдвига SSD1306_Adressing_Page
#define SSD1306_CMD_PageAddrMode_SetPage        0xB0
#define SSD1306_CMD_PageAddrMode_StartColumnLo  0x00
#define SSD1306_CMD_PageAddrMode_StartColumnHi  0x10

// Команды из раздела Hardware Configuration
#define SSD1306_CMD_SetDisplayStartLine         0x40
#define SSD1306_CMD_SetSegmentRemap             0xA0
#define SSD1306_CMD_SetMultiplexRatio           0xA8
#define SSD1306_CMD_SetCOMoutScanDirection      0xC0 
#define SSD1306_CMD_SetDisplayOffset            0xD3
#define SSD1306_CMD_SetCOMPinsConfig            0xDA
  
// Команды из раздела Timing & Driving Scheme Setting
#define SSD1306_CMD_SetDisplayClockDivider      0xD5
#define SSD1306_CMD_SetPrechargePeriod          0xD9
#define SSD1306_CMD_SetVCOMHDeselectLevel       0xDB

// Команды из раздела Charge Pump
#define SSD1306_CMD_ChargePumpSetting           0x8D
//==============================================================================
// Режимы автоматического сдвига указателя в памяти кадра ssd1306
#define SSD1306_Adressing_Horizontal            0       // Сначала инкремент по горизонтали, затем инкремент по вертикали
#define SSD1306_Adressing_Vertical              1       // Сначала инкремент по вертикали, затем инкремент по горизонтали
#define SSD1306_Adressing_Page                  2       // Инкремент только по горизонтали

//Использовать после инициализации SPI
void WriteCmd1306(uint8_t Cmd, uint8_t *pBuff, uint16_t BuffLen);
void WriteData1306(uint8_t *pBuff, uint16_t BuffLen);
// Процедура инициализации дисплея на контроллере ssd1306
void SSD1306_Init(void);
// Процедура переводит дисплей в режим сна
void SSD1306_Sleep(void);
// Процедура выводит дисплей из режима сна
void SSD1306_Wake(void);
// Процедура включает инверсию дисплея
void SSD1306_SetInverseOn(void);
// Процедура отключает инверсию дисплея
void SSD1306_SetInverseOff(void);
// Процедура включает все пиксели дисплея (Тест индикатора)
void SSD1306_AllPixOn(void);
// Процедура отключает тест дисплея и выводит на него картинку из буфера кадра в ssd1306
void SSD1306_AllPixRAM(void);
// Процедура устанавливает параметр контрастности (0-255)
void SSD1306_SetContrast(uint8_t Value);
// Процедура устанавливает начальный и конечный индекс колонки 
// для автосмещения указателя в памяти кадра при чтении записи.
void SSD1306_SetColumns(uint8_t Start, uint8_t End);
// Процедура устанавливает начальный и конечный индекс страницы 
// для автосмещения указателя в памяти кадра при чтении записи.
void SSD1306_SetPages(uint8_t Start, uint8_t End);
// Процедура передаёт в дисплей буфер кадра из массива pBuff
void SSD1306_DisplayFullUpdate(uint8_t *pBuff, uint16_t BuffLen);


void SSD1306_SetDisplayOffset(uint8_t Offset);
void SSD1306_SetDisplayClockDivider(uint8_t DCLKdiv, uint8_t Fosc);
void SSD1306_ChargePumpSetting(uint8_t Value);
void SSD1306_SetMultiplexRatio(uint8_t Mux);
void SSD1306_SetDisplayStartLine(uint8_t Line);
void SSD1306_SetSegmentRemap(uint8_t Value);
void SSD1306_SetMemAdressingMode(uint8_t Mode);
void SSD1306_SetCOMoutScanDirection(uint8_t Value);
void SSD1306_SetCOMPinsConfig(uint8_t AltCOMpinConfig, uint8_t LeftRightRemap);
void SSD1306_SetPrechargePeriod(uint8_t Phase1period, uint8_t Phase2period);
void SSD1306_SetVCOMHDeselectLevel(uint8_t Code);
void SSD1306_DeactivateScroll(void);

#endif

