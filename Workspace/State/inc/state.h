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

    //Measured resistances
    float R1;
    float R2;
    float R3;
    float R4;
    float R5;
    float R6;
    float R7;
    float R8;
    float R9;

	//Temperature
	float Temperature;

	//Resistors display information
    uint8_t ResistorsCount : 3;
	uint8_t CurrentPage1BottomResistor : 3;

	bool ReadyToWork : 1;

	bool EditingMode : 1;
};

//”станавливает все значени€ состо€ни€ по умолчанию
void ResetState(void);

#endif
