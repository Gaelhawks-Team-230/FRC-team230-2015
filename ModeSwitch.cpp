#include "WPILib.h"
#include "Common.h"
#include "ModeSwitch.h"

#ifndef CALIBRATION

ModeSwitch::ModeSwitch()
{
	di_automode_0 = new DigitalInput(DIGITAL_AUTOMODE_ZERO);
	di_automode_1 = new DigitalInput(DIGITAL_AUTOMODE_ONE);
	di_automode_2 = new DigitalInput(DIGITAL_AUTOMODE_TWO);
	di_automode_3 = new DigitalInput(DIGITAL_AUTOMODE_THREE);
}

int ModeSwitch::Get()
{
	int value = 0;

	value = !(di_automode_0->Get()) + (!(di_automode_1->Get()) * 2) + (!(di_automode_2->Get()) * 4);
	//value = (!(di_automode_0->Get())) + (!(di_automode_1->Get()) * 2) + (!(di_automode_2->Get()) * 4) + (!(di_automode_3->Get()) * 8);

	return value;
}

void ModeSwitch::UpdateDash()
{
	SmartDashboard::PutNumber("Auto mode: ", Get());
}
#endif

