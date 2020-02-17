#include "state.h"

struct StateType State;

//”станавливает все значени€ состо€ни€ по умолчанию
void ResetState(void)
{
	State.LeftBtn = false;
	State.RightBtn = false;
	State.UpBtn = false;
	State.DownBtn = false;
	State.EnterBtn = false;
	State.ClearBtn = false;
}
