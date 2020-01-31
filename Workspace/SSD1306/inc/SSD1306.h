#include "main.h"

#ifndef SSD1306H
#define SSD1306H

#define STM32F100


//���������������� ����� �����������
#ifdef STM32F100
    #define SSD1306_RESET_HIGH();               GPIOA->BSRR |= GPIO_BSRR_BS3;
    #define SSD1306_RESET_LOW();                GPIOA->BSRR |= GPIO_BSRR_BR3;

    #define SSD1306_DC_HIGH();                  GPIOA->BSRR |= GPIO_BSRR_BS1;
    #define SSD1306_DC_LOW();                   GPIOA->BSRR |= GPIO_BSRR_BR1;

    #define SSD1306_CS_HIGH();                  GPIOA->BSRR |= GPIO_BSRR_BS2;
    #define SSD1306_CS_LOW();                   GPIOA->BSRR |= GPIO_BSRR_BR2;  
#elif STM32F103
    #error ��������� �����������
#else
    #error ��������� ������� ����������
#endif

//==============================================================================
// ��������� �������
//==============================================================================

#define SSD1306_Width 128
#define SSD1306_Height 64

//==============================================================================
// ���� ������ �������
//==============================================================================
// ������� �� ������� Fundamental
#define SSD1306_CMD_SetContrast                 0x81
#define SSD1306_CMD_AllPixRAM                   0xA4
#define SSD1306_CMD_AllPixOn                    0xA5
#define SSD1306_CMD_SetInverseOff               0xA6
#define SSD1306_CMD_SetInverseOn                0xA7
#define SSD1306_CMD_Sleep                       0xAE
#define SSD1306_CMD_Wake                        0xAF

// ������� �� ������� Scrolling
#define SSD1306_CMD_DeactivateScroll            0x2E

// ������� �� ������� Addressing Setting
// ������� ������ ������ ���������� ��������� � ������ �����
#define SSD1306_CMD_SetMemAdressingMode         0x20    
// ������� ������ ��������� ��������� �������� � ������� ��� ���������� ��������� � ������ �����
// ����������� ��� ������� ���������� SSD1306_Adressing_Horizontal � SSD1306_Adressing_Vertical
#define SSD1306_CMD_SetColumnAddr               0x21
#define SSD1306_CMD_SetPageAddr                 0x22
// ������� ������ �������� � ��������� ��������� ������� ��� ���������� ��������� � ������ �����
// ����������� ��� ������ ���������� SSD1306_Adressing_Page
#define SSD1306_CMD_PageAddrMode_SetPage        0xB0
#define SSD1306_CMD_PageAddrMode_StartColumnLo  0x00
#define SSD1306_CMD_PageAddrMode_StartColumnHi  0x10

// ������� �� ������� Hardware Configuration
#define SSD1306_CMD_SetDisplayStartLine         0x40
#define SSD1306_CMD_SetSegmentRemap             0xA0
#define SSD1306_CMD_SetMultiplexRatio           0xA8
#define SSD1306_CMD_SetCOMoutScanDirection      0xC0 
#define SSD1306_CMD_SetDisplayOffset            0xD3
#define SSD1306_CMD_SetCOMPinsConfig            0xDA
  
// ������� �� ������� Timing & Driving Scheme Setting
#define SSD1306_CMD_SetDisplayClockDivider      0xD5
#define SSD1306_CMD_SetPrechargePeriod          0xD9
#define SSD1306_CMD_SetVCOMHDeselectLevel       0xDB

// ������� �� ������� Charge Pump
#define SSD1306_CMD_ChargePumpSetting           0x8D
//==============================================================================
// ������ ��������������� ������ ��������� � ������ ����� ssd1306
#define SSD1306_Adressing_Horizontal            0       // ������� ��������� �� �����������, ����� ��������� �� ���������
#define SSD1306_Adressing_Vertical              1       // ������� ��������� �� ���������, ����� ��������� �� �����������
#define SSD1306_Adressing_Page                  2       // ��������� ������ �� �����������

//������������ ����� ������������� SPI
void WriteCmd1306(uint8_t Cmd, uint8_t *pBuff, uint16_t BuffLen);
void WriteData1306(uint8_t *pBuff, uint16_t BuffLen);
// ��������� ������������� ������� �� ����������� ssd1306
void SSD1306_Init(void);
// ��������� ��������� ������� � ����� ���
void SSD1306_Sleep(void);
// ��������� ������� ������� �� ������ ���
void SSD1306_Wake(void);
// ��������� �������� �������� �������
void SSD1306_SetInverseOn(void);
// ��������� ��������� �������� �������
void SSD1306_SetInverseOff(void);
// ��������� �������� ��� ������� ������� (���� ����������)
void SSD1306_AllPixOn(void);
// ��������� ��������� ���� ������� � ������� �� ���� �������� �� ������ ����� � ssd1306
void SSD1306_AllPixRAM(void);
// ��������� ������������� �������� ������������� (0-255)
void SSD1306_SetContrast(uint8_t Value);
// ��������� ������������� ��������� � �������� ������ ������� 
// ��� ������������ ��������� � ������ ����� ��� ������ ������.
void SSD1306_SetColumns(uint8_t Start, uint8_t End);
// ��������� ������������� ��������� � �������� ������ �������� 
// ��� ������������ ��������� � ������ ����� ��� ������ ������.
void SSD1306_SetPages(uint8_t Start, uint8_t End);
// ��������� ������� � ������� ����� ����� �� ������� pBuff
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

