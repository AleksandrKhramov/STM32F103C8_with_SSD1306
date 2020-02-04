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

} State;

//������������� ��� �������� ��������� �� ���������
void ResetState(void);

#endif