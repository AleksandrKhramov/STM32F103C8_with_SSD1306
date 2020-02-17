#include "main.h"

#ifndef stateH
#define stateH

#include "stdbool.h"
struct StateType  
{
	//Buttons counters
	uint8_t LeftBtnCounter;
	uint8_t RightBtnCounter;
	uint8_t UpBtnCounter;
	uint8_t DownBtnCounter;
	uint8_t EnterBtnCounter;
	uint8_t ClearBtnCounter;

	//Buttons vaiable
	bool LeftBtn : 1;
	bool RightBtn : 1;
	bool UpBtn : 1;
	bool DownBtn : 1;
	bool EnterBtn : 1;
	bool ClearBtn : 1;

	//NVIC buttons flags
    bool LeftBtnFlag : 1;
	bool RightBtnFlag : 1;
	bool UpBtnFlag : 1;
	bool DownBtnFlag : 1;
	bool EnterBtnFlag : 1;
	bool ClearBtnFlag : 1;

    //Pages
    uint8_t CurrentPageNumber : 4;

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
};

//”станавливает все значени€ состо€ни€ по умолчанию
void ResetState(void);

#endif
