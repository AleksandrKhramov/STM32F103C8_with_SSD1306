#include "state.h"

struct StateType State;

//”станавливает все значени€ состо€ни€ по умолчанию
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

	State.R1 = 0;
    State.R2 = 0;
    State.R3 = 0;
    State.R4 = 0;
    State.R5 = 0;
    State.R6 = 0;
    State.R7 = 0;
    State.R8 = 0;
    State.R9 = 0;

	State.CurrentPageNumber = 0;

	State.Temperature = 0;

	State.ResistorsCount = 0;

	State.CurrentPage1BottomResistor = 0;

	State.ReadyToWork = false;

	State.EditingMode = false;

}
