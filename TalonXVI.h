#ifndef SRC_TALONXVI_H_
#define SRC_TALONXVI_H_

#include "Common.h"

#ifndef CALIBRATION
#include "BarrelGrab.h"
#include "DriveTrain.h"
#include "ModeSwitch.h"
#include "ToteGrab.h"
#include "Elevator.h"
#include "Crane.h"
#include "Autopilot.h"
#include "CameraServer.h"
#include "USBCamera.h"
#endif

/*
 *
 * Autonomous Defines
 */

#define FIRST_PAUSE_TIME			((int)(1.0 * (N1SEC)))
#define BARREL_PAUSE_TIME			((int)(0.3 * (N1SEC)))
#define SECOND_PAUSE_TIME			((int)(0.25 * (N1SEC)))
#define AUTO_CRANE_DRIVE			((int)(0.3 * (N1SEC)))
#define AUTO_SLIDE_TIME				((int)(3.0 * (N1SEC)))
#define AUTO_DRIVE_TIME				((int)(2.0 * (N1SEC)))
#define AUTO_DROP_DRIVE_TIME		((int)(2.1 * (N1SEC)))//((int)(2.15 * (N1SEC))) //((int)(1.55 * (N1SEC)))
#define AUTO_DRIVE_ONLY_TIME		((int)(1.0 * (N1SEC)))
#define AUTO_90DEGROT_TIME			((int)(1.57 * (N1SEC)))
#define AUTO_90DEGROT_TIMEX2		((int)(0.785 * (N1SEC)))
#define AUTO_45DEGROT_TIMEX2		((int)(0.393 * (N1SEC)))
#define AUTO_180DEGROT_TIME			((int)(1.57 * (N1SEC)))	//((int)(3.1 * (N1SEC)))
#define AUTO_TROLLEY_HEIGHT			2.0
#define AUTO_REGRAB_TROLLEY_HEIGHT	1.8
#define AUTO_TROLLEY_BACK_UP		1.5
#define AUTO_ELEVATOR_STACK			(19.0) //(21.0) //height for elevator to go above a tote to stack
#define AUTO_ELEVATOR_MOVE			(10.0) //height for elevator to go above a tote to stack
#define AUTO_ELEVATOR_BOTTOM		(0.25) //height for elevator to go above a tote to stack
#define AUTO_GRAB_DRIVE_CMD			(0.4)

#define AUTO_BACK_TIME				((int)(0.75 * (N1SEC)))
#define AUTO_BARREL_BACKUP_TIME		((int)(1.5 * (N1SEC)))
#define AUTO_WAIT_FOR_PINCH			((int)(0.20 * (N1SEC)))
#define AUTO_WAIT_FOR_TOTE			((int)(0.15 * (N1SEC)))
#define AUTO_WAIT_FOR_LIFT			((int)(0.3 * (N1SEC)))

#define AUTO_DRIVE_TO_TOTE			(84.0)
#define AUTO_DRIVE_SECOND_TOTE		(39.0) //Drive distance to second tote
#define AUTO_DRIVE_THIRD_TOTE		(150.0) //Drive distance to third tote
#define AUTO_DRIVE_AUTOZONE			(60.0) //Drive distance to autozone
#define AUTO_DRIVE_AUTOZONE_BACK	(-145.0) //Drive distance back to autozone
#define AUTO_DRIVE_DROP_STACK       (-12.0)

#define AUTO_STRAFE_RIGHT			(28.0)
#define AUTO_STRAFE_LEFT			(-32.0)
#define AUTO_SECOND_STRAFE_RIGHT	(34.0)
#define AUTO_SECOND_STRAFE_LEFT		(-38.0)
#define AUTO_NORTH_TO_TOTE			(85.0)

/*
 * TalonXVI.h
 *
 *  Created on: Jan 7, 2015
 *      Author: Gaelhawks
 */
class TalonXVI : public SampleRobot
{
public:
	Joystick *driveStick; // only joystick
	Joystick *gamepad; // only joystick

#ifdef CALIBRATION

#ifdef PRACTICE_BOT
	Talon *calDevice;
#else
	VictorSP *calDevice;
#endif
#else
	DriveTrain *drive; // robot drive system
	BarrelGrab *barrelGrabber;
	ToteGrab *toteGrabber; //*planetFitness
	ModeSwitch *automodeSwitch;
	Elevator *elevator; //*youRaiseMeUp
	Crane *crane;
	Autopilot *pilot;
	//MAX Navigation *location;
#endif
	bool firstTime;
	bool cameraInitialized;

	int autoMode;
	int autoStage;

	double loopTime;
	double startTime;
	double curTime;
	double waitTime;
	bool loopOverTime;
	int NumLoopsOverTime;

	bool thumbButtonPressed;

	//Auto Variables
	//const double autoMode1DriveEast[MAX_AUTO1_DRIVE_CMDS] = {0.0, 0.0, 0.0, 0.0, 0.0};
	//const double autoMode1DriveNorth[MAX_AUTO1_DRIVE_CMDS] = {0.0, 0.0, 0.0, 0.0, 0.0};

	int dashCounter;

	TalonXVI(void);
	void Autonomous(void);
	void Disabled(void);
	void OperatorControl(void);
	void Test(void);
	void RobotInit() override;
	void Omnicide(void);
	void CommunityService(void);
	void ServiceDash(void);

#ifndef CALIBRATION
	void AutoModeToteStackSimple(void);
	void AutoModeTwoTote(void);
	void AutoModeContainersFromStepFlat(void);// autoMode 3
	void AutoModeContainersFromStepTilt(void);
	void AutoModeGrabAndMove(void);		// autoMode 1
	void AutoModeMove(void);	// autoMode 2
	void AutoModeAutoPilotTest();
	void AutoModeTotesAndBarrels();
	void AutoModeTotesWithStrafe();
	void AutoModeTwoTotesAndBarrel();
	void AutoModeTest();
	void AutoModeToteStackPilot();	// autoMode 4
	void AutoModeJustGrabBarrelFlat();
	void AutoModeJustGrabBarrelTilt();
#endif

	static double Limit(double, double, double);
};



#endif /* SRC_TALONXVI_H_ */
