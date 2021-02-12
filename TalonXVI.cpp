#include "WPILib.h"
#include "TalonXVI.h"


/*
** TalonXVI... aka Planet Fitness
**  "We pick things up and put things down"
*/
TalonXVI::TalonXVI()
{
	driveStick = new Joystick(0);
	gamepad = new Joystick(1);
#ifdef CALIBRATION
#ifdef PRACTICE_BOT
	calDevice = new Talon(PWM_CALDEVICE);
#else
	calDevice = new VictorSP(PWM_CALDEVICE);
#endif
#else
	drive = new DriveTrain();
	barrelGrabber = new BarrelGrab();
	automodeSwitch = new ModeSwitch();
	toteGrabber = new ToteGrab();
	elevator = new Elevator();
	crane = new Crane();
	pilot = new Autopilot();
	//MAX location = new Navigation();
#ifdef USE_USBCAMERA
	//camServer = CameraServer::GetInstance();
#endif
#endif

	firstTime = true;
	cameraInitialized = false;
	autoMode = 0;
	autoStage = 0;

	dashCounter = 0;
	loopTime = LOOPTIME;
	startTime = 0.0;
	curTime = 0.0;
	waitTime = 0.0;
	loopOverTime = false;
	thumbButtonPressed = false;
	NumLoopsOverTime = 0;
}



void TalonXVI::RobotInit()
{
#ifdef USE_USBCAMERA
	if(!cameraInitialized)
	{
		printf("CameraServer being started\n");
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") was verified through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
		cameraInitialized = true;
	}
#endif
}


/*
** Drive left & right motors for 2 seconds then stopA
 */
void TalonXVI::Autonomous()
{
#ifndef CALIBRATION

	autoMode = automodeSwitch->Get();
	int loopCounter=0;
	autoStage = 0;

	if(firstTime)
	{
		elevator->StartFromTheBottom();
		firstTime = false;
	}

	//resets all control systems
	elevator->ControlSystemReset();
	barrelGrabber->ControlSystemReset();
	crane->ControlSystemReset();
	pilot->StopAll();

	printf("Auto Mode %d\n", autoMode);


	double initialStartTime;
	initialStartTime = GetTime();
	startTime = initialStartTime - loopTime;
	drive->GyroOn();

	while (IsAutonomous() && IsEnabled())
	{
		startTime += loopTime;
		loopCounter++;

#ifdef PRACTICE_BOT
		//autoMode = 3;
#endif
		switch (autoMode)
		{
			case 0:
			default:
				break;

			case 1:
				//AutoModeGrabAndMove();
				AutoModeTwoTote();
				break;

			case 2:
				AutoModeMove();
				break;

			case 3:
				AutoModeContainersFromStepFlat();
				break;

			case 4:
				//AutoModeTest();
				AutoModeToteStackPilot();
				//AutoModeAutoPilotTest();
				//AutoModeTotesAndBarrels();
				break;

			case 5:
				AutoModeJustGrabBarrelFlat();
				break;

			case 6:
				AutoModeContainersFromStepTilt();
				break;

			case 7:
				AutoModeJustGrabBarrelTilt();
				break;

/*
			case 8:
				AutoModeTwoTote();
				break;
*/


		}

		/*
		** COMMUNITY SERVICE
		*/
		//drive->JoystickDrive(driveStick->GetX(), driveStick->GetY(), driveStick->GetZ(), driveStick->GetTrigger());
		CommunityService();
		ServiceDash();


		// ENFORCE LOOP TIMING
		curTime = GetTime();
		waitTime = loopTime - (curTime - startTime);
		if (waitTime < 0.0)
		{
			printf("WARNING! LOOP IS OVERTIME by %f seconds\n", waitTime*-1);
			loopOverTime = true;
		}
		else
		{
			Wait(waitTime);
			loopOverTime = false;

#ifdef CHECK_LOOPTIMING
			endTime = GetTime();
			totalLoopTime = endTime - startTime;
			printf("startTime: %f  curTime : %f  endTime: %f	Wait Time: %f  This Loop Time: %f  Total Delta Time: %f [avg: %f] \n",
					startTime, curTime, endTime, waitTime, totalLoopTime, endTime-initialStartTime, (endTime-initialStartTime)/loopCounter);
#endif
		}
	}
	Omnicide();
#endif //CALIBRATION
}


void TalonXVI::OperatorControl()
{
	int loopCounter=0;
	//bool init_flag = true;

#ifndef CALIBRATION
	if(firstTime)
	{
		elevator->StartFromTheBottom();
		firstTime = false;
	}

	//resets all control systems
	elevator->ControlSystemReset();
	barrelGrabber->ControlSystemReset();
	crane->ControlSystemReset();


	drive->GyroOn();
	thumbButtonPressed = false;


#endif
#ifdef CHECK_LOOPTIMING
	double endTime;
	double totalLoopTime;
#endif

	double initialStartTime;
	initialStartTime = GetTime();
	startTime = initialStartTime - loopTime;

	while (IsOperatorControl() && IsEnabled())
	{
		startTime += loopTime;
		loopCounter++;
#ifdef CALIBRATION
		float cal_command_x = (driveStick->GetY());
		if(cal_command_x < -0.5)
			cal_command_x =-1.0;
		else if(cal_command_x > .5)
			cal_command_x = 1.0;
		else
			cal_command_x = 0.0;

		printf("calibrating PWM %d -- %f\n", PWM_CALDEVICE, cal_command_x);
		calDevice->Set(cal_command_x);
#else

		/*
		** JOYSTICK/GAMEPAD BUTTONS
		*/

#ifdef USE_GYRO
		if (driveStick->GetRawButton(ENABLE_GYRO_BUTTON))
		{
			drive->GyroOn();
		}
		else if (driveStick->GetRawButton(DISABLE_GYRO_BUTTON))
		{
			drive->GyroOff();
		}
#endif

		//
		// BARREL OPERATIONS = Crane + BarrelGrab
		//
		barrelGrabber->JoystickMove(gamepad->GetRawAxis(GAMEPAD_TROLLEY));
		crane->GetBarrelAutomation(gamepad->GetRawButton(BUTTON_GET_BARREL_FLAT), gamepad->GetRawButton(BUTTON_GET_BARREL_TILT));
#ifdef USE_CRANE
		if(gamepad->GetPOV(0) == DPAD_CRANE_UP)
		{
			//printf("joystick DPAD crane UP\n");
			crane->JoystickMove(1.0);//change rate in the crane class, keep this 1
		}
		else if(gamepad->GetPOV(0) == DPAD_CRANE_DOWN)
		{
			//printf("joystick DPAD crane DOWN\n");
			crane->JoystickMove(-1.0);//change rate in the crane class, keep this -1
		}
		else
		{
			//printf("joystick DPAD crane STOP\n");
			crane->JoystickMove(0.0);
		}
#endif
#ifdef USE_BARRELGRAB
		if(gamepad->GetPOV(0) == DPAD_WRIST_UP)
		{
			//printf("calling WristUp\n");
			barrelGrabber->WristUp();
		}
		else if(gamepad->GetPOV(0) == DPAD_WRIST_DOWN)
		{
			//printf("calling WristDown\n");
			barrelGrabber->WristDown();
		}

		if(gamepad->GetRawButton(BUTTON_CLAW_PINCH))
		{
			//printf("calling ClawPinch\n");
			barrelGrabber->ClawPinch();
		}

		if(gamepad->GetRawButton(BUTTON_CLAW_RELEASE))
		{
			//printf("calling ClawRelease\n");
			barrelGrabber->ClawRelease();
		}
#endif
		//
		// TOTE OPERATIONS
		//
		if(driveStick->GetRawButton(DRIVER_ELEVATOR_UP))
		{
			elevator->ManualMove(-1.0);
		}
		else if(driveStick->GetRawButton(DRIVER_ELEVATOR_DOWN))
		{
			elevator->ManualMove(1.0);
		}
		else
		{
			elevator->ManualMove(gamepad->GetRawAxis(GAMEPAD_ELEVATOR));
		}
		// TESTING ONLY!!!!
        /*
		if(gamepad->GetRawButton(4))
		{
			if(init_flag)
				elevator->GoToHeight(27, init_flag);
			if(!elevator->atGoal())
			{
				elevator->GoToHeight(27, init_flag);
			}
			init_flag = false;
		}
		else
		{
			init_flag = true;
			elevator->StopSeekingHeight();
		}
		*/

		if(gamepad->GetRawButton(BUTTON_GET_TOTE) || driveStick->GetRawButton(DRIVER_GET_TOTE))
		{
			//printf("calling GetTote\n");
			toteGrabber->GetTote();
		}
		if(gamepad->GetRawButton(BUTTON_EJECT_TOTE) || driveStick->GetRawButton(DRIVER_EJECT_TOTE))
		{
			//printf("calling EjectTote\n");
			toteGrabber->EjectTote();
		}
		if(gamepad->GetRawButton(STOP_TOTE_MOTOR) || driveStick->GetRawButton(DRIVER_STOP_MOTOR))
		{
			toteGrabber->StopMotor();
		}



		if(gamepad->GetRawButton(BUTTON_SQUEEZE_TOTE) || driveStick->GetRawButton(DRIVER_SQUEEZE_TOTE) || driveStick->GetRawButton(THUMB_BUTTON))
		{
			//printf("calling SqueezeTote\n");
			toteGrabber->SqueezeTote();

		}
		if(gamepad->GetRawButton(BUTTON_RELEASE_TOTE) || driveStick->GetRawButton(DRIVER_RELEASE_TOTE) ||
				(thumbButtonPressed && !(driveStick->GetRawButton(THUMB_BUTTON))))
		{
			//printf("calling ReleaseTote\n");
			toteGrabber->ReleaseTote();
		}
		thumbButtonPressed = driveStick->GetRawButton(THUMB_BUTTON);



		/*
		** COMMUNITY SERVICE
		*/
		//victorTest->Set(driveStick->GetX());
		drive->JoystickDrive(driveStick->GetX(), driveStick->GetY(), driveStick->GetZ(), driveStick->GetTrigger());
		CommunityService();
		ServiceDash();
#endif //Calibration

		// ENFORCE LOOP TIMING
		curTime = GetTime();
		waitTime = loopTime - (curTime - startTime);
		if (waitTime < 0.0)
		{
			printf("WARNING! LOOP IS OVERTIME by %f seconds\n", waitTime*-1);
			loopOverTime = true;
			NumLoopsOverTime++;
		}
		else
		{
		    Wait(waitTime);
			loopOverTime = false;

#ifdef CHECK_LOOPTIMING
			endTime = GetTime();
			totalLoopTime = endTime - startTime;
			printf("startTime: %f  curTime : %f  endTime: %f	Wait Time: %f  This Loop Time: %f  Total Delta Time: %f [avg: %f] \n",
					startTime, curTime, endTime, waitTime, totalLoopTime, endTime-initialStartTime, (endTime-initialStartTime)/loopCounter);
#endif
		}
	}
	Omnicide();
}

/*
** Runs during test mode
*/
void TalonXVI::Test()
{
}

/*
** Runs when disabled
*/
void TalonXVI::Disabled()
{
	/*
	Wait(1.0);

	while (!IsEnabled())
	{
		//autoMode = (int) SmartDashboard::GetNumber("Auto Mode (1-2): ");
		//SmartDashboard::PutNumber("Auto Mode printout", autoMode);

		// give it some time to catch its breath
		Wait(0.5);
	}
	*/
}

void TalonXVI::Omnicide()
{
#ifndef CALIBRATION
	drive->StopAll();
	toteGrabber->StopAll();
	elevator->StopAll();
	barrelGrabber->StopAll();
	pilot->StopAll();
	crane->StopAll();
#endif
}


void TalonXVI::CommunityService()
{
#ifndef CALIBRATION
	toteGrabber->Service();
	barrelGrabber->Service();
	//pilot->Service();
	elevator->Service();
	crane->Service();
	// NOTE: DriveTrain and ModeSwitch do not have Service functions
#endif
}

void TalonXVI::ServiceDash()
{
#ifndef CALIBRATION
	if (dashCounter == 10)
	{
		//printf("Updating Dash \n");
		SmartDashboard::PutBoolean("Loop OverTime: ", loopOverTime);
		SmartDashboard::PutNumber("Number of Loops OverTime: ", NumLoopsOverTime);
		toteGrabber->UpdateDash();
		barrelGrabber->UpdateDash();
#ifdef USE_GYRO
		drive->UpdateDash();
#endif
		automodeSwitch->UpdateDash();
		elevator->UpdateDash();
		pilot->UpdateDash();
		crane->UpdateDash();

		dashCounter = 0;
	}
	else
	{
		dashCounter++;
	}
#endif
}

double TalonXVI::Limit(double min, double max, double curVal)
{
	if (curVal > max)
			return max;
	if (curVal < min)
			return min;
	return curVal;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////  AUTONOMOUS MODES  //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CALIBRATION

void TalonXVI::AutoModeToteStackSimple() // three tote stack with help
{
	static float xcmd;
	static float ycmd;
	static float zcmd;
	static bool piloting;
	static int loopcount = 0;

	switch(autoStage)
	{
		case 0: //START: begin lift first tote
			piloting = false;
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			pilot->StopAll(); // ensure reset
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
			loopcount = 0;
			autoStage++;
			break;

		case 1: // wait for the tote to be off the ground
			piloting = false;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			loopcount++;
			if(loopcount >= FIRST_PAUSE_TIME) //after the wait, set driving goals
			{
				piloting = true;
				pilot->SetGoals(AUTO_DRIVE_TO_TOTE, 0.0, 0.0);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 2: //wait for tote and autopilot goals, grab second tote
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			toteGrabber->GetTote();
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 3:  //when tote is grabbed, send elevator down
			piloting = false;
			elevator->ManualMove(0.0);
			if(toteGrabber->HasTote())
			{
				autoStage++;
				loopcount = 0;
			}
			break;

		case 4: // move forward a little and get the tote
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = AUTO_GRAB_DRIVE_CMD; ycmd = 0.0;  zcmd = 0.0;
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				//pilot->StopAll();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 5: //ensure elevator is all the way down
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(elevator->atGoal())
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 6: // wait for the tote to be off the ground
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_LIFT) //after the wait, set driving goals
			{
				piloting = true;
				pilot->SetGoals(AUTO_DRIVE_TO_TOTE, 0.0, 0.0);
				toteGrabber->GetTote();
				autoStage++;
				loopcount = 0;
			}
			break;

		case 7: //wait for tote and autopilot goals, grab third tote
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			toteGrabber->GetTote();
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 8:  //when tote is grabbed, send elevator down
			piloting = false;
			elevator->ManualMove(0.0);
			if(toteGrabber->HasTote())
			{
				autoStage++;
				loopcount = 0;
			}
			break;

		case 9: // move forward a little and get the tote
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = AUTO_GRAB_DRIVE_CMD; ycmd = 0.0;  zcmd = 0.0;
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				pilot->StopAll();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 10: //ensure elevator is all the way down
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(elevator->atGoal())
			{
				elevator->GoToHeight(AUTO_ELEVATOR_MOVE, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 11: //ensure elevator is all the way up
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_MOVE, false);
			if(elevator->atGoal())
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = -2.0;
				loopcount = 0;
				autoStage++;
			}
			break;

		case 12: //rotate 90 deg
			piloting = false;
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_90DEGROT_TIMEX2)
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = 0.0;
				pilot->StopAll();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 13: //drive to autozone...
			piloting = true;
			pilot->SetGoals(AUTO_DRIVE_AUTOZONE_BACK, 0.0, 0.0); //DRIVING BACKWARDS INTO AUTOZONE
			elevator->ManualMove(0.0);
			loopcount = 0;
			autoStage++;
			break;

		case 14:	// when arrive at goal, move elevator to bottom
			piloting = true;
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_DROP_DRIVE_TIME)
			{
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
			}
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				loopcount = 0;
			}
			break;

		case 15:	// when arrive at goal, move elevator to bottom
			piloting = true;
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				loopcount = 0;
			}
			if (elevator->atGoal())
			{
				elevator->ManualMove(0.0);
				autoStage++;
			}
			break;


		default:
			break;

	}
	if(piloting)
	{
		//printf("stage %d piloting\n", autoStage);
		pilot->Service();
	}
	else
	{
		//printf("stage %d driving\n", autoStage);
		drive->DriveControl(xcmd, ycmd, zcmd);
	}
}

void TalonXVI::AutoModeTotesWithStrafe() // three tote stack with help
{
	static float xcmd;
	static float ycmd;
	static float zcmd;
	static bool piloting;
	static int loopcount = 0;

	switch(autoStage)
	{
		case 0: //START: begin lift first tote
			piloting = false;
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			pilot->StopAll(); // ensure reset
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
			loopcount = 0;
			autoStage++;
			break;

		case 1: // wait for the tote to be off the ground
			pilot->StopAll();
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			pilot->SetGoals(0.0, AUTO_STRAFE_RIGHT, 0.0);
			autoStage++;
			loopcount = 0;
			break;

		case 2: //wait for tote and autopilot goals, grab second tote
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->EjectTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 3:
			piloting = true;
			pilot->SetGoals(AUTO_NORTH_TO_TOTE, AUTO_STRAFE_LEFT, 0.0);
			loopcount++;
			if(loopcount >= AUTO_DRIVE_ONLY_TIME)
			{
				toteGrabber->StopMotor();
				toteGrabber->GetTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 4:  //when tote is grabbed, send elevator down
			piloting = true;
			if(pilot->IsAtGoal())
			{

				pilot->StopAll();
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 5:
			elevator->ManualMove(0.0);
			if(toteGrabber->HasTote())
			{
				autoStage++;
				loopcount = 0;
			}
			break;

		case 6: // move forward a little and get the tote
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = AUTO_GRAB_DRIVE_CMD; ycmd = 0.0;  zcmd = 0.0;
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				//pilot->StopAll();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 7: //ensure elevator is all the way down
			piloting = true;
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(elevator->atGoal())
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 8: // wait for the tote to be off the ground
			pilot->StopAll();
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			pilot->SetGoals(0.0, AUTO_SECOND_STRAFE_RIGHT, 0.0);
			autoStage++;
			loopcount = 0;
			break;

		case 9: //wait for tote and autopilot goals, grab second tote
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->EjectTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 10:
			piloting = true;
			pilot->SetGoals(AUTO_NORTH_TO_TOTE, AUTO_SECOND_STRAFE_LEFT, 0.0);
			loopcount++;
			if(loopcount >= AUTO_DRIVE_ONLY_TIME)
			{
				toteGrabber->StopMotor();
				toteGrabber->GetTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 11:  //when tote is grabbed, send elevator down
			piloting = true;
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 12:  //when tote is grabbed, send elevator down
			piloting = false;
			elevator->ManualMove(0.0);
			if(toteGrabber->HasTote())
			{
				autoStage++;
				loopcount = 0;
			}
			break;

		case 13: // move forward a little and get the tote
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = AUTO_GRAB_DRIVE_CMD; ycmd = 0.0;  zcmd = 0.0;
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				pilot->StopAll();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 14: //ensure elevator is all the way down
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(elevator->atGoal())
			{
				elevator->GoToHeight(AUTO_ELEVATOR_MOVE, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 15: //ensure elevator is all the way up
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_MOVE, false);
			if(elevator->atGoal())
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = -2.0;
				loopcount = 0;
				autoStage++;
			}
			break;

		case 16: //rotate 90 deg
			piloting = false;
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_90DEGROT_TIMEX2)
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = 0.0;
				pilot->StopAll();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 17: //drive to autozone...
			piloting = true;
			pilot->SetGoals(AUTO_DRIVE_AUTOZONE_BACK, 0.0, 0.0); //DRIVING BACKWARDS INTO AUTOZONE
			elevator->ManualMove(0.0);
			loopcount = 0;
			autoStage++;
			break;

		case 18:	// when arrive at goal, move elevator to bottom
			piloting = true;
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_DROP_DRIVE_TIME)
			{
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
			}
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				loopcount = 0;
			}
			break;

		case 19:	// when arrive at goal, move elevator to bottom
			piloting = true;
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				loopcount = 0;
			}
			if (elevator->atGoal())
			{
				elevator->ManualMove(0.0);
				autoStage++;
			}
			break;


		default:
			break;

	}
	if(piloting)
	{
		//printf("stage %d piloting\n", autoStage);
		pilot->Service();
	}
	else
	{
		//printf("stage %d driving\n", autoStage);
		drive->DriveControl(xcmd, ycmd, zcmd);
	}
}


void TalonXVI::AutoModeContainersFromStepFlat()
{
	static int loopCount = 0;
	static bool buttonASim = false;
	static bool buttonYSim = false;
	static float xcmd;
	static float ycmd;
	static float zcmd;


	switch(autoStage)
	{
		case 0://start moving claw into position
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			buttonASim = true;
			buttonYSim = false;
			loopCount = 0;
            autoStage++;
			break;

		case 1: //drive backwards
			xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
			loopCount = 0;
			autoStage++;
			break;

		case 2: // stop driving after short distance
			loopCount++;
			if(loopCount >= AUTO_CRANE_DRIVE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 3:// wait for claw in position, start picking up barrel
			if(crane->IsDoneExtending())
			{
				buttonASim = false;
				autoStage++;
			}
			break;

		case 4: //drive backwards
				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				loopCount = 0;
				autoStage++;
				break;

		case 5: // stop driving after short distance
			loopCount++;
			if(loopCount >= GET_BARREL_DELAY)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 6:	// wait for barrel lifted, start drop and regrab
			if(crane->IsDoneRetracting())
			{
				loopCount = 0;
				barrelGrabber->ControlledMove(0.0);
				autoStage++;
			}
			break;

		case 7: // bring barrel down on tote, and release
			barrelGrabber->ControlledMove(-0.5);
			//printf("Trolley: %f (top is 3.84)     \n", trolleyHeight);
			if(barrelGrabber->GetTrolleyHeight() <= AUTO_REGRAB_TROLLEY_HEIGHT)
			{
				barrelGrabber->ControlledMove(0.0);
				barrelGrabber->ClawRelease();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 8:	// bring trolley down further and pinch again
			loopCount++;
			if(loopCount >= AUTO_WAIT_FOR_PINCH)
			{
				barrelGrabber->ControlledMove(-0.5);
				if(barrelGrabber->GetTrolleyHeight() <= MIN_TROLLEY_POSITION)
				{
					barrelGrabber->ControlledMove(0.0);
					barrelGrabber->ClawPinch();
					loopCount = 0;
					autoStage++;
				}
			}
			break;

		case 9: // regrab barrel and move trolley up
			loopCount++;
			if(loopCount >= AUTO_WAIT_FOR_PINCH)
			{
				barrelGrabber->ControlledMove(0.5);
				if((barrelGrabber->GetTrolleyHeight()) >= AUTO_TROLLEY_BACK_UP)
				{
					barrelGrabber->ControlledMove(0.0);
					loopCount = 0;
					autoStage++;
				}

			}
			break;

		case 10: // when done moving, stop
			xcmd = 0.5; ycmd = 0.0; zcmd = 0.0;
			loopCount++;
			barrelGrabber->ControlledMove(0.0);
			if(loopCount >= AUTO_BARREL_BACKUP_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			}
			break;

		default:
			barrelGrabber->ControlledMove(0.0);
			break;
	}

	// note that this automation includes calls to both crane and trolley control system
	crane->GetBarrelAutomation(buttonASim, buttonYSim);
	drive->DriveControl(xcmd, ycmd, zcmd);
	//printf("stage %d, drive %f, %f, %f\n", autoStage, xcmd, ycmd, zcmd);
}

void TalonXVI::AutoModeJustGrabBarrelFlat()
{
	static int loopCount = 0;
	static bool buttonASim = false;
	static bool buttonYSim = false;
	static float xcmd;
	static float ycmd;
	static float zcmd;


	switch(autoStage)
	{
		case 0://start moving claw into position
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			buttonASim = true;
			buttonYSim = false;
			loopCount = 0;
			autoStage++;
			break;

		case 1: //drive backwards
			xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
			loopCount = 0;
			autoStage++;
			break;

		case 2: // stop driving after short distance
			loopCount++;
			if(loopCount >= AUTO_CRANE_DRIVE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 3:// wait for claw in position, start picking up barrel
			if(crane->IsDoneExtending())
			{
				buttonASim = true;
				barrelGrabber->ClawPinch();
				autoStage++;
			}
			break;

		case 4: //drive backwards
				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				loopCount = 0;
				autoStage++;
				break;

		case 5: // stop driving after short distance
			loopCount++;
			if(loopCount >= GET_BARREL_DELAY)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		default:
			break;
	}

	// note that this automation includes calls to both crane and trolley control system
	crane->GetBarrelAutomation(buttonASim, buttonYSim);
	drive->DriveControl(xcmd, ycmd, zcmd);
	//printf("stage %d, drive %f, %f, %f\n", autoStage, xcmd, ycmd, zcmd);
}

void TalonXVI::AutoModeContainersFromStepTilt()
{
	static int loopCount = 0;
	static bool buttonASim = false;
	static bool buttonYSim = false;
	static float xcmd;
	static float ycmd;
	static float zcmd;


	switch(autoStage)
	{
		case 0://start moving claw into position
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			buttonASim = false;
			buttonYSim = true;
			loopCount = 0;
            autoStage++;
			break;

		case 1: //drive backwards
			xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
			loopCount = 0;
			autoStage++;
			break;

		case 2: // stop driving after short distance
			loopCount++;
			if(loopCount >= AUTO_CRANE_DRIVE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 3:// wait for claw in position, start picking up barrel
			if(crane->IsDoneExtending())
			{
				buttonYSim = false;
				autoStage++;
			}
			break;

		case 4: //drive backwards
				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				loopCount = 0;
				autoStage++;
				break;

		case 5: // stop driving after short distance
			loopCount++;
			if(loopCount >= GET_BARREL_DELAY)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 6:	// wait for barrel lifted, start drop and regrab
			if(crane->IsDoneRetracting())
			{
				loopCount = 0;
				barrelGrabber->ControlledMove(0.0);
				autoStage++;
			}
			break;

		case 7: // bring barrel down on tote, and release
			barrelGrabber->ControlledMove(-0.5);
			//printf("Trolley: %f (top is 3.84)     \n", trolleyHeight);
			if(barrelGrabber->GetTrolleyHeight() <= AUTO_REGRAB_TROLLEY_HEIGHT)
			{
				barrelGrabber->ControlledMove(0.0);
				barrelGrabber->ClawRelease();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 8:	// bring trolley down further and pinch again
			loopCount++;
			if(loopCount >= AUTO_WAIT_FOR_PINCH)
			{
				barrelGrabber->ControlledMove(-0.5);
				if(barrelGrabber->GetTrolleyHeight() <= TILTED_MIN_TROLLEY)
				{
					barrelGrabber->ControlledMove(0.0);
					barrelGrabber->ClawPinch();
					loopCount = 0;
					autoStage++;
				}
			}
			break;

		case 9: // regrab barrel and move trolley up
			loopCount++;
			if(loopCount >= AUTO_WAIT_FOR_PINCH)
			{
				barrelGrabber->ControlledMove(0.5);
				if((barrelGrabber->GetTrolleyHeight()) >= AUTO_TROLLEY_BACK_UP)
				{
					barrelGrabber->ControlledMove(0.0);
					loopCount = 0;
					autoStage++;
				}

			}
			break;
/*
		case 10: // when done moving, stop
			xcmd = 0.5; ycmd = 0.0; zcmd = 0.0;
			loopCount++;
			barrelGrabber->ControlledMove(0.0);
			if(loopCount >= AUTO_BARREL_BACKUP_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			}
			break;
*/
		default:
			barrelGrabber->ControlledMove(0.0);
			break;
	}

	// note that this automation includes calls to both crane and trolley control system
	crane->GetBarrelAutomation(buttonASim, buttonYSim);
	drive->DriveControl(xcmd, ycmd, zcmd);
	//printf("stage %d, drive %f, %f, %f\n", autoStage, xcmd, ycmd, zcmd);
}

void TalonXVI::AutoModeJustGrabBarrelTilt()
{
	static int loopCount = 0;
	static bool buttonASim = false;
	static bool buttonYSim = false;
	static float xcmd;
	static float ycmd;
	static float zcmd;


	switch(autoStage)
	{
		case 0://start moving claw into position
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			buttonASim = false;
			buttonYSim = true;
			loopCount = 0;
			autoStage++;
			break;

		case 1: //drive backwards
			xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
			loopCount = 0;
			autoStage++;
			break;

		case 2: // stop driving after short distance
			loopCount++;
			if(loopCount >= AUTO_CRANE_DRIVE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 3:// wait for claw in position, start picking up barrel
			if(crane->IsDoneExtending())
			{
				buttonYSim = true;
				barrelGrabber->ClawPinch();
				autoStage++;
			}
			break;

		case 4: //drive backwards
				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				loopCount = 0;
				autoStage++;
				break;

		case 5: // stop driving after short distance
			loopCount++;
			if(loopCount >= GET_BARREL_DELAY)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		default:
			break;
	}

	// note that this automation includes calls to both crane and trolley control system
	crane->GetBarrelAutomation(buttonASim, buttonYSim);
	drive->DriveControl(xcmd, ycmd, zcmd);
	//printf("stage %d, drive %f, %f, %f\n", autoStage, xcmd, ycmd, zcmd);
}

void TalonXVI::AutoModeGrabAndMove()
{
	static float trolleyMotorCmd = 0.0;
	static int loopcount = 0;
	static float trolleyHeight;
	static float xcmd;
	static float ycmd;
	static float zcmd;

	switch(autoStage)
	{
		case 0: //grab tote
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			elevator->GoToHeight(15.0, true);
			barrelGrabber->ClawRelease();
			trolleyMotorCmd = 0.0;
			autoStage++;
			break;

		case 1: //wait for elevator at height
			elevator->GoToHeight(15.0, false);
			barrelGrabber->WristDown();
			if(elevator->atGoal())
			{
				autoStage++;
			}
			break;

		case 2: //drive backwards
			elevator->ManualMove(0.0);
			xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
			loopcount = 0;
			autoStage++;
			break;

		case 3: // stop driving after short distance
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_BACK_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 4: //grab barrel
			elevator->ManualMove(0.0);
			barrelGrabber->ClawPinch();
			autoStage++;
			break;

		case 5: //lift barrel
			elevator->ManualMove(0.0);
			trolleyMotorCmd = 0.5;
			trolleyHeight = barrelGrabber->GetTrolleyHeight();
			//printf("Trolley: %f (top is 3.84)     \n", trolleyHeight);
			if(trolleyHeight >= AUTO_TROLLEY_HEIGHT)
			{
				trolleyMotorCmd = 0.0;
				autoStage++;
			}
			break;
		case 6: //rotate sideways
			elevator->ManualMove(0.0);
			xcmd = 0.0; ycmd = 0.0;  zcmd = 1.0;
			loopcount = 0;
			autoStage++;
			break;

		case 7: //stop in autozone
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_90DEGROT_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 8: //drive forward
			elevator->ManualMove(0.0);
			xcmd = 0.50; ycmd = 0.0;  zcmd = 0.0;
			loopcount = 0;
			autoStage++;
			break;

		case 9: //stop in autozone
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_DRIVE_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		default:
			break;
	}
	barrelGrabber->ControlledMove(trolleyMotorCmd);
	drive->DriveControl(xcmd, ycmd, zcmd);
}

void TalonXVI::AutoModeMove()
{
	static int loopcount = 0;
	static float xcmd;
	static float ycmd;
	static float zcmd;

	switch(autoStage)
	{
		case 0: //drive forward
			xcmd = 0.50; ycmd = 0.0;  zcmd = 0.0;
			loopcount = 0;
			autoStage++;
			break;

		case 1: //stop in autozone
			loopcount++;
			if(loopcount >= AUTO_DRIVE_ONLY_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		default:
			break;
	}
	drive->DriveControl(xcmd, ycmd, zcmd);
}


void TalonXVI::AutoModeTotesAndBarrels()
{
	static float trolleyMotorCmd = 0.0;
	static int loopcount = 0;
	static float xcmd;
	static float ycmd;
	static float zcmd;
	static bool piloting;

	switch(autoStage)
	{
		case 0: //grab tote
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);  //was 15.0
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			barrelGrabber->ClawRelease();
			trolleyMotorCmd = 0.0;
			loopcount = 0;
			autoStage++;
			break;

		case 1: // wait for the tote to be off the ground
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			barrelGrabber->WristDown();
			loopcount++;
			if(loopcount >= FIRST_PAUSE_TIME) //after the wait, set driving goals
			{
				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				loopcount = 0;
				autoStage++;
			}
			break;

		case 2: //wait for elevator at goal and done driving
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);	//was 15.0
			loopcount++;
			//printf("stage %d loopcount %d\n", autoStage, loopcount);
			if(loopcount >= AUTO_BACK_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			}
			if(elevator->atGoal() && (loopcount >= AUTO_BACK_TIME))
			{
				//printf("done driving back, loopcount %d\n", loopcount);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 3: //grab barrel
			piloting = false;
			elevator->ManualMove(0.0);
			barrelGrabber->ClawPinch();
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_PINCH)
			{
				loopcount = 0;
				autoStage++;
			}
			break;

		case 4: //lift barrel
			piloting = false;
			elevator->ManualMove(0.0);
			trolleyMotorCmd = 0.75;	//was 0.5
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			autoStage++;
			break;

		case 5: // wait for the barrel to be off the ground
			piloting = false;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			loopcount++;
			if(loopcount >= BARREL_PAUSE_TIME) //after the wait, set driving goals
			{
//				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
				loopcount = 0;
			}
			break;

		case 6: //spin 180
			piloting = false;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			xcmd = 0.0; ycmd = 0.0;  zcmd = 2.0;
			loopcount = 0;
			autoStage++;
			break;

		case 7: //stop rotating
			piloting = false;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			loopcount++;
			if(loopcount >= AUTO_180DEGROT_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				loopcount = 0;
				pilot->StopAll();
				autoStage++;
			}
			break;

		case 8: //drive forward to second tote
			piloting = true;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			//printf("stage %d Driving to %f\n", autoStage, AUTO_DRIVE_SECOND_TOTE);
			pilot->SetGoals(AUTO_DRIVE_SECOND_TOTE, 0.0, 0.0);
			loopcount = 0;
			autoStage++;
			break;

		case 9:	// when arrive at goal, begin squeeze tote
			piloting = true;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 10: // wait to until have tote, start to move elevator down
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			if(toteGrabber->HasTote())
			{
				loopcount = 0;
				autoStage++;
			}
			break;

		case 11: // move forward a little and get the tote
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = 0.25; ycmd = 0.0;  zcmd = 0.0;
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				pilot->StopAll();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(1.5, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 12:  //elevator going down...
			piloting = false;
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			elevator->GoToHeight(1.5, false);
			if(elevator->atGoal())
			{
				//printf("picking up tote 2\n");
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
				autoStage++;
			}
			break;

		case 13: // wait for the tote to be off the ground
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			loopcount++;
			if(loopcount >= FIRST_PAUSE_TIME) //after the wait, set rotate
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = -2.0;
				autoStage++;
				loopcount = 0;
			}
			break;

		case 14: //rotate 180
			piloting = false;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			loopcount++;
			if(loopcount >= AUTO_180DEGROT_TIME)
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = 0.0;
				pilot->StopAll();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 15: //drive forward to 3rd tote
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			//printf("stage %d Driving to %f\n", autoStage, AUTO_DRIVE_THIRD_TOTE);
			pilot->SetGoals(AUTO_DRIVE_THIRD_TOTE, 3.0, 0.0);
			loopcount = 0;
			autoStage++;
			break;

		case 16: // when arrive at goal, begin squeeze tote
			piloting = true;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 17: // wait to until have tote, move elevator down
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			if(toteGrabber->HasTote())
			{
				loopcount = 0;
				autoStage++;
			}
			break;

		case 18: // move forward a little and get the tote
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = 0.25; ycmd = 0.0;  zcmd = 0.0;
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				pilot->StopAll();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(1.5, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 19: //elevator going down...
			piloting = false;
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			elevator->GoToHeight(1.5, false);
			if(elevator->atGoal())
			{
				//printf("picking up tote 3!!!\n");
				elevator->GoToHeight(10.0, true);
				autoStage++;
			}
			break;

		case 20: //elevator picking up 3rd tote, wait for at height
			piloting = false;
			elevator->GoToHeight(10.0, false);
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
				xcmd = 0.0; ycmd = 0.0; zcmd = 2.0;
				loopcount = 0;
				autoStage++;
			}
			break;

		case 21: //rotate 90 deg
			piloting = false;
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_90DEGROT_TIMEX2)
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = 0.0;
				pilot->StopAll();
				loopcount = 0;
				pilot->StopAll();
				autoStage++;
			}
			break;

		case 22: //drive to autozone...
			piloting = true;
			//printf("stage %d Driving to %f\n", autoStage, AUTO_DRIVE_AUTOZONE);
			pilot->SetGoals(AUTO_DRIVE_AUTOZONE, 0.0, 0.0);
			elevator->ManualMove(0.0);
			loopcount = 0;
			autoStage++;
			break;

		case 23:	// when arrive at goal, move elevator to bottom
			piloting = true;
			elevator->ManualMove(0.0);
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				elevator->GoToHeight(0.5, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 24: //wait for elevator to get to goal
			piloting = false;
			elevator->GoToHeight(0.5, false);
			xcmd = 0.0; ycmd = 0.0; zcmd = 0.0;
			if(elevator->atGoal())
			{
				loopcount = 0;
				autoStage++;
			}
			break;

		case 25: //drive away from totes
			piloting = true;
			elevator->ManualMove(0.0);
			//printf("stage %d Driving to %f\n", autoStage, AUTO_DRIVE_DROP_STACK);
			pilot->SetGoals(AUTO_DRIVE_DROP_STACK, 0.0, 0.0);
			loopcount = 0;
			autoStage++;
			break;

		case 26:	// when arrive at goal, move elevator to bottom
			piloting = true;
			elevator->ManualMove(0.0);
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
			}
			break;

		default:
			break;
	}

	if(piloting)
	{
		//printf("stage %d piloting\n", autoStage);
		pilot->Service();
	}
	else
	{
		//printf("stage %d driving\n", autoStage);
		drive->DriveControl(xcmd, ycmd, zcmd);
	}
	barrelGrabber->ControlledMove(trolleyMotorCmd);
	elevator->Service();
}

#ifdef UNUSED_AUTOMODES
void TalonXVI::AutoModeAutoPilotTest()
{
	switch(autoStage)
	{
		case 0:
			pilot->SetGoals(50.0, 0.0, 0.0);
			printf("Setting Goals to 24 and 12\n");
			autoStage++;
			break;
		case 1:
			if(pilot->IsAtGoal())
			{
				printf("Autopilot At Goal\n");
			}
			break;
	}

	pilot->Service();
}

void TalonXVI::AutoModeTwoTotesAndBarrel()
{
	static float trolleyMotorCmd = 0.0;
	static int loopcount = 0;
	static float xcmd;
	static float ycmd;
	static float zcmd;
	static bool piloting;

	switch(autoStage)
	{
		case 0: //grab tote
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);  //was 15.0
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			barrelGrabber->ClawRelease();
			trolleyMotorCmd = 0.0;
			loopcount = 0;
			autoStage++;
			break;

		case 1: // wait for the tote to be off the ground
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			barrelGrabber->WristDown();
			loopcount++;
			if(loopcount >= FIRST_PAUSE_TIME) //after the wait, set driving goals
			{
				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				loopcount = 0;
				autoStage++;
			}
			break;

		case 2: //wait for elevator at goal and done driving
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);	//was 15.0
			loopcount++;
			//printf("stage %d loopcount %d\n", autoStage, loopcount);
			if(loopcount >= AUTO_BACK_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			}
			if(elevator->atGoal() && (loopcount >= AUTO_BACK_TIME))
			{
				//printf("done driving back, loopcount %d\n", loopcount);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 3: //grab barrel
			piloting = false;
			elevator->ManualMove(0.0);
			barrelGrabber->ClawPinch();
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_PINCH)
			{
				loopcount = 0;
				autoStage++;
			}
			break;

		case 4: //lift barrel
			piloting = false;
			elevator->ManualMove(0.0);
			trolleyMotorCmd = 0.75;	//was 0.5
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			autoStage++;
			break;

		case 5: // wait for the barrel to be off the ground
			piloting = false;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			loopcount++;
			if(loopcount >= BARREL_PAUSE_TIME) //after the wait, set driving goals
			{
//				xcmd = -0.25; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
				loopcount = 0;
			}
			break;

		case 6: //check for trolley at goal, but start sping and move on to next stage
			piloting = false;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			xcmd = 0.0; ycmd = 0.0;  zcmd = 2.0;
			loopcount = 0;
			autoStage++;
			break;

		case 7: //spin 180 - stop rotating
			piloting = false;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			loopcount++;
			if(loopcount >= AUTO_180DEGROT_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				loopcount = 0;
				pilot->StopAll();
				autoStage++;
			}
			break;

		case 8: //drive forward to second tote (also making sure to stop trolley when at goal)
			piloting = true;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			//printf("stage %d Driving to %f\n", autoStage, AUTO_DRIVE_SECOND_TOTE);
			pilot->SetGoals(AUTO_DRIVE_SECOND_TOTE, 0.0, 0.0);
			loopcount = 0;
			autoStage++;
			break;

		case 9:	// when arrive at goal, begin squeeze tote  (also making sure to stop trolley when at goal)
			piloting = true;
			elevator->ManualMove(0.0);
			//printf("Trolley: %f  motorcmd %f     \n", barrelGrabber->GetTrolleyHeight(), trolleyMotorCmd);
			if(trolleyMotorCmd != 0.0 && barrelGrabber->GetTrolleyHeight() >= AUTO_TROLLEY_HEIGHT)
			{
				//printf("stage %d stopping Trolley\n", autoStage);
				trolleyMotorCmd = 0.0;
			}
			if(pilot->IsAtGoal())
			{
				pilot->StopAll();
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 10: // wait to until have tote, start to move elevator down
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			if(toteGrabber->HasTote())
			{
				loopcount = 0;
				autoStage++;
			}
			break;

		case 11: // move forward a little and get the tote
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = 0.25; ycmd = 0.0;  zcmd = 0.0;
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				pilot->StopAll();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(1.5, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 12:  //elevator going down...
			piloting = false;
			xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
			elevator->GoToHeight(1.5, false);
			if(elevator->atGoal())
			{
				//printf("picking up tote 2\n");
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
				autoStage++;
			}
			break;

		case 13: // wait for the tote to be off the ground
			piloting = false;
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			loopcount++;
			if(loopcount >= FIRST_PAUSE_TIME) //after the wait, set rotate
			{
				xcmd = 0.0; ycmd = 0.0; zcmd = -2.0;
				autoStage++;
				loopcount = 0;
			}
			break;

		case 14: //stop in autozone
			piloting = false;
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_90DEGROT_TIMEX2)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

		case 15: //drive forward
			piloting = false;
			elevator->ManualMove(0.0);
			xcmd = 0.50; ycmd = 0.0;  zcmd = 0.0;
			loopcount = 0;
			autoStage++;
			break;

		case 16: //stop in autozone
			piloting = false;
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_DRIVE_TIME)
			{
				xcmd = 0.0; ycmd = 0.0;  zcmd = 0.0;
				autoStage++;
			}
			break;

	}
	if(piloting)
	{
		//printf("stage %d piloting\n", autoStage);
		pilot->Service();
	}
	else
	{
		//printf("stage %d driving\n", autoStage);
		drive->DriveControl(xcmd, ycmd, zcmd);
	}
	barrelGrabber->ControlledMove(trolleyMotorCmd);
	elevator->Service();

}

void TalonXVI::AutoModeTest()
{
	static int loopcount = 0;

	switch(autoStage)
	{
		case 0: //elevator picking up 3rd tote, wait for at height
				//drive->DriveControl(0.0, 0.0, 2.0);
				pilot->SetGoals(10.0, 30.0, 0.785);

				loopcount = 0;
				autoStage++;
			break;

		case 1: //rotate 45 deg

			loopcount++;

			//drive->DriveControl(0.0, 0.0, 2.0);
			if(loopcount >= 50)
			{
				//drive->DriveControl(0.0, 0.0, 0.0);
				//pilot->StopAll();
				pilot->SetGoals(60.0, 0.0, 0.0);
				loopcount = 0;
				autoStage++;
			}
			break;
		case 2:
			pilot->SetGoals(60.0, 0.0, 0.0);
			loopcount++;
			autoStage++;
			break;
		case 3:
			loopcount++;
			break;

		default:
			break;
	}
	printf("Loops %d ", loopcount);
	pilot->Service();
	printf("\n");
}
#endif

void TalonXVI::AutoModeTwoTote()
{
	static int loopcount = 0;

		switch(autoStage)
		{
			case 0: //START: begin lift first tote
				pilot->StopAll(); // ensure reset
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
				loopcount = 0;
				autoStage++;
				break;

			case 1: // wait for the tote to be off the ground			tote one
				if(elevator->atGoal())
				{
					elevator->ManualMove(0.0);
				}
				else
				{
					elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
				}
				loopcount++;

				if(loopcount >= 25) //50 //after the wait, set driving goals
				{
					pilot->SetGoals(-5.0, 0.0, 0.785);
					autoStage++;
					loopcount = 0;
				}
				break;

			case 2: // wait for back-up and rotate
				loopcount++;
				toteGrabber->EjectTote();
				if(elevator->atGoal())
				{
					elevator->ManualMove(0.0);
				}
				else
				{
					elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
				}
				if(loopcount >= 25) //50)
				{
					pilot->SetGoals(40.0, 40.0, 0.785);
					autoStage++;
					loopcount = 0;
				}
				break;

			case 3: // wait for drive forward at angle
				loopcount++;
				if(elevator->atGoal())
				{
					elevator->ManualMove(0.0);
				}
				else
				{
					elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
				}
				if(loopcount >= 30) //40) //50)
				{
					pilot->SetGoals(40.0, 00.0, 0.0);
					autoStage++;
					loopcount = 0;
				}
				break;

			case 4: // wait for rotate back and move forward 		move barrel ONE
				loopcount++;
				elevator->ManualMove(0.0);
				if(loopcount >= 50) //50)
				{
					pilot->SetGoals(93.0, -10.0, 0.0); //pilot->SetGoals(89.0, -5.0, 0.0);
					autoStage++;
					toteGrabber->GetTote();
					loopcount = 0;
				}
				break;

			case 5: // wait for drive up to totes (2)
				loopcount++;
				if(loopcount >= 85) //80)
				{
					toteGrabber->GetTote();
					toteGrabber->SqueezeTote();
					pilot->ForcedReset();
					loopcount = 0;
					autoStage++;
				}
				break;

			case 6: //wait for small drive to grab tote
				loopcount++;
				if(loopcount >= 10 && toteGrabber->HasTote())
				{
					pilot->SetGoals(100.0, -5.0, 0.0); //pilot->SetGoals(93.0, -5.0, 0.0);
					loopcount = 0;
					autoStage++;
				}
				break;


			case 7: // move forward a little and get the tote
				elevator->ManualMove(0.0);
				loopcount++;
				if(loopcount >= AUTO_WAIT_FOR_TOTE)
				{
					toteGrabber->ReleaseTote();
					toteGrabber->StopMotor();
					elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
					loopcount = 0;
					autoStage++;
				}
				break;

			case 8: //ensure elevator is all the way down
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
				if(elevator->atGoal())
				{
					elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
					loopcount = 0;
					autoStage++;
					//pilot->StopAll();
				}
				break;

			case 9: // wait for the tote to be off the ground		tote TWO
				loopcount++;
				if(elevator->atGoal())
				{
					elevator->ManualMove(0.0);
					autoStage++;
				}
				else
				{
					elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
				}
				break;

			case 10: // keep driving and leave stack behind
				loopcount++;
				if(loopcount >= 110)
				{
					pilot->ForcedReset();
					autoStage++;
					loopcount = 0;
				}
				break;

			default:
				break;
		}
		pilot->Service();
		printf("autoStage: %d \n", autoStage);
	}

void TalonXVI::AutoModeToteStackPilot() // three tote stack with help
{
	static int loopcount = 0;

	switch(autoStage)
	{
		case 0: //START: begin lift first tote
			pilot->StopAll(); // ensure reset
			elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
			loopcount = 0;
			autoStage++;
			break;

		case 1: // wait for the tote to be off the ground			tote one
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			loopcount++;

			if(loopcount >= 25) //50 //after the wait, set driving goals
			{
				pilot->SetGoals(-5.0, 0.0, 0.785);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 2: // wait for back-up and rotate
			loopcount++;
			toteGrabber->EjectTote();
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(loopcount >= 25) //50)
			{
				pilot->SetGoals(40.0, 40.0, 0.785);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 3: // wait for drive forward at angle
			loopcount++;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(loopcount >= 30) //40) //50)
			{
				pilot->SetGoals(40.0, 00.0, 0.0);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 4: // wait for rotate back and move forward 		move barrel ONE
			loopcount++;
			elevator->ManualMove(0.0);
			if(loopcount >= 50) //50)
			{
				pilot->SetGoals(93.0, -10.0, 0.0); //pilot->SetGoals(89.0, -5.0, 0.0);
				autoStage++;
				toteGrabber->GetTote();
				loopcount = 0;
			}
			break;

		case 5: // wait for drive up to totes (2)
			loopcount++;
			if(loopcount >= 85) //80)
			{
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				pilot->ForcedReset();
				loopcount = 0;
				autoStage++;
			}
			break;

		case 6: //wait for small drive to grab tote
			loopcount++;
			if(loopcount >= 10 && toteGrabber->HasTote())
			{
				pilot->SetGoals(97.0, -10.0, 0.0); //pilot->SetGoals(93.0, -5.0, 0.0);
				loopcount = 0;
				autoStage++;
			}
			break;


		case 7: // move forward a little and get the tote
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 8: //ensure elevator is all the way down
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(elevator->atGoal())
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, true);
				loopcount = 0;
				autoStage++;
				//pilot->StopAll();
			}
			break;

		case 9: // wait for the tote to be off the ground		tote TWO
			loopcount++;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(loopcount >= 30) //50) //after the wait, set driving goals
			{
				pilot->SetGoals(97.0, -10.0, 0.785);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 10: // wait for rotate
			loopcount++;
			toteGrabber->EjectTote();
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(loopcount >= 30) //50)
			{
				//pilot->SetGoals(120.0, 25.0, 0.785);
				pilot->SetGoals(135.0, 30.0, 0.785);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 11: // wait for drive forward at angle
			loopcount++;
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
			}
			else
			{
				elevator->GoToHeight(AUTO_ELEVATOR_STACK, false);
			}
			if(loopcount >= 30) //50)
			{
				pilot->SetGoals(135.0, -20.0, 0.0);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 12: // wait for rotate back and move forward		moving barrel TWO
			loopcount++;
			elevator->ManualMove(0.0);
			if(loopcount >= 50) //after the wait, set driving goals
			{
				pilot->SetGoals(190.0, -20.0, 0.0);
				autoStage++;
				toteGrabber->GetTote();
				loopcount = 0;
			}
			break;

		case 13: // wait to drive up to totes
			loopcount++;
			if(loopcount >= 85) //100)
			{
				toteGrabber->GetTote();
				toteGrabber->SqueezeTote();
				pilot->SetGoals(195.0, -20.0, 0.0);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 14: //wait for small drive
			loopcount++;
			if(loopcount >= 10 && toteGrabber->HasTote())
			{
				toteGrabber->GetTote();
				loopcount = 0;
				autoStage++;
			}
			break;


		case 15: // move forward a little and get the tote
			elevator->ManualMove(0.0);
			loopcount++;
			if(loopcount >= AUTO_WAIT_FOR_TOTE)
			{
				pilot->ForcedReset();
				toteGrabber->ReleaseTote();
				toteGrabber->StopMotor();
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
				loopcount = 0;
				autoStage++;
			}
			break;

		case 16: //ensure elevator is all the way down					tote THREE
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(elevator->atGoal())
			{
				elevator->GoToHeight(AUTO_ELEVATOR_MOVE, true);
				loopcount = 0;
				autoStage++;
				pilot->ForcedReset();
			}
			break;

		case 17: // wait for the tote to be off the ground - rotate and goto AutoZone
			elevator->GoToHeight(AUTO_ELEVATOR_MOVE, false);
			if(elevator->atGoal())
			{
				elevator->ManualMove(0.0);
				pilot->SetGoals(215.0, 120.0, -1.57);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 18: // wait for a short time and start dropping the tote stack
			loopcount++;
			elevator->ManualMove(0.0);
			if(loopcount >= AUTO_DROP_DRIVE_TIME)
			{
				elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, true);
				autoStage++;
				loopcount = 0;
			}
			break;

		case 19: // keep driving and leave stack behind
			loopcount++;
			elevator->GoToHeight(AUTO_ELEVATOR_BOTTOM, false);
			if(loopcount >= 110)
			{
 			    pilot->ForcedReset();
				autoStage++;
				loopcount = 0;
			}
			break;

		default:
			break;
	}
	pilot->Service();
	printf("autoStage: %d \n", autoStage);
}
#endif // !CALIBRATION

START_ROBOT_CLASS(TalonXVI);
