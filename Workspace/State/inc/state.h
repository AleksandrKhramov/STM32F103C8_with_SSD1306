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

	//NVIC buttons flags
    bool LeftBtnDownFlag : 1;
	bool RightBtnDownFlag : 1;
	bool UpBtnDownFlag : 1;
	bool DownBtnDownFlag : 1;
	bool EnterBtnDownFlag : 1;
	bool ClearBtnDownFlag : 1;

    //Pages
    uint8_t CurrentPageNumber : 5;

	bool EditingMode : 1;

	//Nominals
	float R1Nom;
	float R2Nom;
	float R3Nom;
	float R4Nom;
	float R5Nom;
	float R6Nom;
	float R7Nom;
	float R8Nom;
	float R9Nom;

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

	//Powers
	float P1;
    float P2;
    float P3;
    float P4;
    float P5;
    float P6;
    float P7;
    float P8;
    float P9;

	//Categories
	const char* R1Cat;
	const char* R2Cat;
	const char* R3Cat;
	const char* R4Cat;
	const char* R5Cat;
	const char* R6Cat;
	const char* R7Cat;
	const char* R8Cat;
	const char* R9Cat;

	//Accuracy classes
	float R1AccuracyClass;
	float R2AccuracyClass;
	float R3AccuracyClass;
	float R4AccuracyClass;
	float R5AccuracyClass;
	float R6AccuracyClass;
	float R7AccuracyClass;
	float R8AccuracyClass;
	float R9AccuracyClass;

	//Temperature
	float Temperature;

	//Resistors display information
    uint8_t ResistorsCount : 4;
	uint8_t CurrentPage1BottomResistor : 4;

	bool ReadyToWork : 1;

	uint8_t CurrentPage2ResistorsSet : 3;
	uint8_t ChosenPage2Resistor : 3;

	bool ModeRectangle : 1;

	uint8_t Brigtness;

	uint8_t CurrentPage4EditingValue;
};

//”станавливает все значени€ состо€ни€ по умолчанию
void ResetState(void);

#endif
