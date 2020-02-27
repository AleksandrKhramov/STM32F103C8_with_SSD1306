#include "state.h"

struct StateType State;

//Устанавливает все значения состояния по умолчанию
void ResetState(void)
{
	State.LeftBtnCounter = 0;
	State.RightBtnCounter = 0;
	State.UpBtnCounter = 0;
	State.DownBtnCounter = 0;
	State.EnterBtnCounter = 0;
	State.ClearBtnCounter = 0;

	State.LeftBtn = false;
	State.RightBtn = false;
	State.UpBtn = false;
	State.DownBtn = false;
	State.EnterBtn = false;
	State.ClearBtn = false;

	State.LeftBtnFlag = false;
	State.RightBtnFlag = false;
	State.UpBtnFlag = false;
	State.DownBtnFlag = false;
	State.EnterBtnFlag = false;
	State.ClearBtnFlag = false;

	State.R1Nom = 0.001;
    State.R2Nom = 0.01;
    State.R3Nom = 0.1;
    State.R4Nom = 1;
    State.R5Nom = 10;
    State.R6Nom = 100;
    State.R7Nom = 1000;
    State.R8Nom = 10000;
    State.R9Nom = 100000;

	State.R1 = 0.001;
    State.R2 = 0.01;
    State.R3 = 0.1;
    State.R4 = 1;
    State.R5 = 10;
    State.R6 = 100;
    State.R7 = 1000;
    State.R8 = 10000;
    State.R9 = 100000;

	State.R1AccuracyClass = 0.0005;
	State.R2AccuracyClass = 0.0005;
	State.R3AccuracyClass = 0.0005;
	State.R4AccuracyClass = 0.0005;
	State.R5AccuracyClass = 0.0005;
	State.R6AccuracyClass = 0.0005;
	State.R7AccuracyClass = 0.0005;
	State.R8AccuracyClass = 0.0005;
	State.R9AccuracyClass = 0.0005;

	State.P1 = 0;
    State.P2 = 0;
    State.P3 = 0;
    State.P4 = 0;
    State.P5 = 0;
    State.P6 = 0;
    State.P7 = 0;
    State.P8 = 0;
    State.P9 = 0;

	State.R1Cat = "нет";
	State.R2Cat = "нет";
	State.R3Cat = "нет";
	State.R4Cat = "нет";
	State.R5Cat = "нет";
	State.R6Cat = "нет";
	State.R7Cat = "нет";
	State.R8Cat = "нет";
	State.R9Cat = "нет";

	State.CurrentPageNumber = 2;

	State.Temperature = 32.321;

	State.ResistorsCount = 9;

	State.CurrentPage2ResistorsSet = 0;

	State.CurrentPage1BottomResistor = 0;

	State.ReadyToWork = false;

	State.EditingMode = false;

	State.ModeRectangle = false;

	State.Brigtness = 100;

	State.CurrentPage4EditingValue = 0;
}
