#ifndef SSD1306H
#define SSD1306H
//Использовать после инициализации SPI
void WriteCmd1306(unsigned char cmd);
void WriteData1306(unsigned char dat);
void SetAddr1306(unsigned char page,unsigned char lCol,unsigned char hCol);
void HomeAddr1306(void);
//void Full_on(void);
void InitSSD1306(void);
#endif

