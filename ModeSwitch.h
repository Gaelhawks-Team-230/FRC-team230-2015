#ifndef SRC_MODESWITCH_H_
#define SRC_MODESWITCH_H_
/*
 * ModeSwitch.h
 *
 *  Created on: Jan 7, 2015
 *      Author: Gaelhawks
 */
class ModeSwitch
{
	private:
		DigitalInput *di_automode_0;
		DigitalInput *di_automode_1;
		DigitalInput *di_automode_2;
		DigitalInput *di_automode_3;

   	public:
		ModeSwitch();
		int Get(void);
		void UpdateDash(void);
};



#endif /* SRC_MODESWITCH_H_ */
