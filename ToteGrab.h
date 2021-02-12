/*
 * ToteGrab.h
 *
 *  Created on: Jan 25, 2015
 *      Author: kcs26_000
 */

#ifndef TOTEGRAB_H_
#define TOTEGRAB_H_

#define TOTE_GRAB_ELEVATOR_HEIGHT			(27.0)
#define ELEVATOR_GET_TOTE_HEIGHT			(4.0)
#define TOTE_PICKUP_HEIGHT					(16.0)

class ToteGrab
{
private:
#ifdef PRACTICE_BOT
	Talon *wheel;
#else
	VictorSP *wheel;
#endif

	DigitalInput *limitSwitch;
	DoubleSolenoid *totePistons;
	bool holdingTote;
	bool pistonSqueezed;
	typedef enum {IdleReleased, ElevatorUp, CheckHeight, CloseToteGrab, IdlePressed, OpenToteGrab} grabStages;
	grabStages grabStage;
	bool inAutomation;

public:
	ToteGrab(void);
	void LocalReset(void);
	void StopAll(void);
	void StopMotor(void);
	void GetTote(void);
	void EjectTote(void);
	void ReleaseTote(void);
	void SqueezeTote(void);
	void ToteAcquireAutomation(bool btnPressed);
	bool HasTote(void);
	bool IsInAutomation(void);
	void UpdateDash(void);
	void Service(void);
};



#endif /* TOTEGRAB_H_ */
