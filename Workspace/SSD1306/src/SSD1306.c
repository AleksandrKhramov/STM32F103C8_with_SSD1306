#include "SSD1306.h"

void WriteCmd1306(uint8_t Cmd, uint8_t *pBuff, uint16_t BuffLen)
{
    SSD1306_DC_LOW();//*
    SSD1306_CS_LOW();//*
    SPI_Write(&Cmd, 1);//*
    SPI_Write(pBuff, BuffLen);//*
    SSD1306_CS_HIGH();//*
}  
//--------------------------------------------------------------------------------
void WriteData1306(unsigned char dat)
{
    SSD1306_DC_HIGH();//*
    SSD1306_CS_LOW();//*
    SPI_send8b(pBuff, BuffLen);//*
    SSD1306_CS_HIGH();//*
    SSD1306_DC_LOW();//*  
}
//--------------------------------------------------------------------------------
void InitSSD1306(void)
{
    SSD1306_GPIO_init();//*

    // Сброс контроллера дисплея ssd1306 ножкой Reset
    #if (SSD1306_ResetPinUsed)      // Включено управление ножкой reset ssd1306
    SSD1306_RESET_HIGH();
    delay_ms(2);
    SSD1306_RESET_LOW();  // Роняем ножку reset в 0 на 10 мс
    delay_ms(15);
    SSD1306_RESET_HIGH();
    #endif
    
    // Шлём команды инициализации ssd1306
    SSD1306_Sleep();//*
    SSD1306_SetDisplayClockDivider(1, 8);//*
    SSD1306_SetMultiplexRatio(SSD1306_Height);//*
    SSD1306_SetDisplayOffset(0);//*
    SSD1306_SetDisplayStartLine(0);//*
    SSD1306_ChargePumpSetting(1);//*
    SSD1306_SetMemAdressingMode(SSD1306_Adressing_Horizontal);//*
    SSD1306_SetSegmentRemap(0);   //*        // *меняет направление заполнение матрицы из буфера кадра (вертикаль/горизонталь)
    SSD1306_SetCOMoutScanDirection(0);//*    // *переворачивает оторбражение на матрице (только по вертикали)
    
    if ((SSD1306_Width == 128) && (SSD1306_Height == 32))
        SSD1306_SetCOMPinsConfig(0, 0);//*
    else  if ((SSD1306_Width == 128) && (SSD1306_Height == 64))
        SSD1306_SetCOMPinsConfig(1, 0);//*
    else  if ((SSD1306_Width == 96) && (SSD1306_Height == 16))
        SSD1306_SetCOMPinsConfig(0, 0);//*
    
    SSD1306_SetContrast(127);//*
    SSD1306_SetPrechargePeriod(2, 2);//*
    SSD1306_SetVCOMHDeselectLevel(0x40);//*
    SSD1306_AllPixRAM();//*
    SSD1306_SetInverseOff();//*
    SSD1306_DeactivateScroll();//*
    SSD1306_Wake();//*
}
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------

