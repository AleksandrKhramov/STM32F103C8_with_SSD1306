#include "state.h"

struct StateType State;

//������������� ��� �������� ��������� �� ���������
void ResetState(void)
{
	State.LeftBtn = false;
	State.RightBtn = false;
	State.UpBtn = false;
	State.DownBtn = false;
	State.EnterBtn = false;
	State.ClearBtn = false;
}
