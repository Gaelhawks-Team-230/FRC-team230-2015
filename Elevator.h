#include "WPILib.h"
#include "common.h"

#ifndef _ELEVATOR_H
#define _ELEVATOR_H

// Elevator.h	Created on: January 18th, 2015	Author: Chris Lipscomb

#define MAX_ELEVATOR_SPEED_DOWN				(20.0) //(15.0)
#define MAX_ELEVATOR_SPEED_UP				(15.0) //(15.0)
#define MIN_ELEVATOR_HEIGHT					(1.0)
#define MAX_ELEVATOR_HEIGHT					(58.0) //(2442 counts)
#define LEVEL_HEIGHT						(12.0)
#define LIFT_JOYSTICK_DEADBAND 				(0.3)
#define AT_GOAL_DEADBAND					(0.3)

#define GAP_LIMIT							(2.0) //inches?
#define TOTE_GRAB_LIMIT						(15.0)

#define MIN_VALUE_DOWN						(-6.0)
#define MAX_VALUE_DOWN						(6.0)
#define MIN_VALUE_UP						(1.0)
#define MAX_VALUE_UP						(1.0)
#define K_ELEVATOR_POS						(1.0)

//control system velocity constants
#define ELEVATOR_TAU						(0.015)
#define ELEVATOR_K_ROBOT					(39.0)
#define ELEVATOR_K_BANDWIDTH				(20.0)

//control system position constants
#define ELEVATOR_VLIMIT						(15.0) //(12.0) //inches per second - ONLY used by GoToHeight
#define ELEVATOR_KP							(5.0)
#define ELEVATOR_KFF						(0.6)

class Elevator
{
private:
#ifdef PRACTICE_BOT
	Talon *liftMotor;
#else
	VictorSP *liftMotor;
#endif

	Encoder *heightEncoder;
	DigitalInput *toteSensor;

	bool atGoalHeight;
	bool motorStall;

	float motorCmd;
	float lastPos;
	float errorInt;

	double height;
	double level;
	double inchesperpulse;
	double currLevel;
	double goalHeight;
	double Pcmd;

	bool startedFromTheBottom;
	bool nowWeHere;

public:
	Elevator(void);
	void ManualMove(float input);
	void LocalReset(void);
	void StopAll(void);
	void GoToHeight(double destinationHeight, bool Init = false);
	double GetHeight();
	void UpdateDash(void);
	void Service(void);
	bool IsAtLevel(int levelTest);
	int FindLevel();
	bool tooClose();
	void StartFromTheBottom(void);
	void ControlSystemReset (void);
	float VelControl(float velCmd);
	bool atGoal(void);
	void StopSeekingHeight(void);

};

#endif
