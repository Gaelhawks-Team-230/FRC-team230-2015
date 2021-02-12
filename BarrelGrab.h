/*
 * BarrelGrab.h
 *
 *  Created on: Jan 19, 2015
 *      Author: jacob_000
 */

#ifndef SRC_BARRELGRAB_H_
#define SRC_BARRELGRAB_H_

#define MAX_TROLLEY_POSITION		(3.80)
#define HALF_TROLLEY_POSITION		(2.46)
#define MIN_TROLLEY_POSITION		(1.20)
#define TILTED_MIN_TROLLEY			(1.42)
#define TROLLEY_MAX_RATE_UP			(0.75)
#define TROLLEY_MAX_RATE_DOWN		(0.75)
#define TROLLEY_INCHES_PER_VOLT	(1.0)
#define TROLLEY_OFFSET_INCHES	(0.0)

#define TROLLEY_CLAW_LIMIT		(2.95)
#define TROLLEY_K_VEL			(0.5)
#define TROLLEY_K_VEL_INT		(1.0)
//#define CONTROL_LOOP_DEADBAND	(0.1)

//control system constants
#define TROLLEY_TAU						(0.02)
#define TROLLEY_K_ROBOT					(1.45)
#define TROLLEY_K_BANDWIDTH				(10.0)

class BarrelGrab
{
private:

#ifdef PRACTICE_BOT
	Talon *trolleyMotor;
#else
	VictorSP *trolleyMotor;
#endif

	AnalogInput *trolleyPot;
	DoubleSolenoid *claw;
	DoubleSolenoid *wrist;

	float lastPos;
	float errorInt;

	bool isPinched;
	bool isWristUp;
	bool inAutomation;

public:
	BarrelGrab();
	void Service(void);
	void LocalReset(void);
	void StopAll(void);
	void UpdateDash(void);
	void JoystickMove(float power);
	void ControlledMove(float power);
	void ClawPinch(void);
	void ClawRelease(void);
	void WristUp(void);
	void WristDown(void);
	void ControlSystemReset(void);
	bool IsWristUp(void);
	bool IsPinched(void);
	float GetTrolleyHeight(void);
	float VelControl(float);
	void SetAutomation(bool automationOn);
	//virtual ~BarrelGrab();
};

#endif /* SRC_BARRELGRAB_H_ */
