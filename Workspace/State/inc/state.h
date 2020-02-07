#include "main.h"

#ifndef stateH
#define stateH

#include "stdbool.h"


struct  
{
	//Buttons vaiable
	bool LeftBtn;
	bool RightBtn;
	bool UpBtn;
	bool DownBtn;
	bool EnterBtn;
	bool ClearBtn;

    //Pages
    uint8_t CurrentPageNumber;

    //Measured resistance
    char R1[8];
    char R2[8];
    char R3[8];
    char R4[8];
    char R5[8];
    char R6[8];
    char R7[8];
    char R8[8];
    char R9[8];

    uint8_t ResistorsCount;

} State;

//”станавливает все значени€ состо€ни€ по умолчанию
void ResetState(void);

#endif
