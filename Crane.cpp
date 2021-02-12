#include "WPILib.h"
#include "TalonXVI.h"
#include "Crane.h"

#ifdef USE_CRANE
#ifndef CALIBRATION
Crane::Crane(void)
{
	//variables
#ifdef PRACTICE_BOT
	winchMotor = new Talon(PWM_CRANE);
#else
	winchMotor= new VictorSP(PWM_CRANE);
#endif

	armEncoder= new AnalogInput(ANALOG_CRANE);

	pauseCount = 0;
	lastPos = 0.0;
	errorInt = 0.0;
	lastCmd = 0.0;

	getBarrelStage = idleReleased;
	inAutomation = false;
}

float Crane::GetArmAngle()
{
	float angle = (armEncoder->GetVoltage() * DEGREES_PER_VOLT) - OFFSET_DEGREES;//degrees per volt calculated based on 152 degree travel / 4 volt travel
	return angle;
}

void Crane::ArmStop()
{
	winchMotor-> Set(0.0);
}

void Crane::LocalReset()
{
	ArmStop();
}

void Crane::JoystickMove(float power)
{
	if (inAutomation)
		return;

	if (fabs(power) <= JOYSTICK_DEADBAND) //no deadband in automation
	{
		power = 0.0;
	}
	ControlledMove(power);
}

void Crane::ControlledMove(float power)
{
	float motorCmd;
	float angle = GetArmAngle();

	if(power > 0.0)
	{
		if (angle <= ANGLE_UPPER_LIMIT)
		{
			motorCmd = 0.0;
		}

		else
		{
			motorCmd = CRANE_MAX_SPEED_UP;
		}
	}
	else if(power < 0.0) //Necessary because power can be equal to 0.
	{
		if(angle >= ANGLE_LOWER_LIMIT)
		{
			motorCmd = 0.0;
		}
		else
		{
			motorCmd = -1.0 * CRANE_MAX_SPEED_DOWN;
		}
	}
	else
	{
		motorCmd = 0.0;  // setting command to 0.0 if power is 0.0
	}

#ifdef USE_CRANE_CONTROL_SYSTEM
	motorCmd = VelControl(motorCmd);   //alters for control system
#else
	// for testing without control system
	if (motorCmd > 1.0)
		motorCmd = 1.0;
	if (motorCmd < -1.0)
		motorCmd = -1.0;
	//printf("crane winchMotor output %f \n",motorCmd);
#endif

	winchMotor->Set(motorCmd);
	//printf("Crane moving at %f (power %f)\n", motorCmd , power);
}

void Crane::Service()
{

}

void Crane::GetBarrelAutomation(bool getBarrelFlatButton, bool getBarrelTiltButton)
{
	bool getBarrelButton;
	static float craneMotorCmd = 0.0;
	static float trolleyMotorCmd = 0.0;
	static int loopCount = 0;

	getBarrelButton = getBarrelFlatButton || getBarrelTiltButton;
	if(getBarrelFlatButton)
	{
		craneAutoGoal = BARRELGRAB_FLAT_LOWER_LIMIT;
	}
	else if(getBarrelTiltButton)
	{
		craneAutoGoal = BARRELGRAB_TILT_LOWER_LIMIT;
	}
	switch (getBarrelStage)
	{
		case idleReleased: //if button pressed, lower crane; else, do nothing
		{
			if (getBarrelButton)
			{
				craneMotorCmd = 0.0;
				trolleyMotorCmd = 0.0;
				inAutomation = true;
				GLOBAL_BARRELGRAB->SetAutomation(inAutomation);
				printf("Button Pressed. Going to stage lowerCrane\n");
				getBarrelStage = lowerCrane;
			}
			else
			{
				inAutomation = false;
				GLOBAL_BARRELGRAB->SetAutomation(inAutomation);
			}
			break;
		}

		case lowerCrane: //lower the crane to middle limit
		{
			if(getBarrelButton)
			{
				GLOBAL_BARRELGRAB->WristUp();
				craneMotorCmd = GET_BARREL_CRANE_DOWN_CMD; //Crane Down
				if(GetArmAngle() >= CRANE_CLAW_LIMIT)
				{
					printf("Going to stage extendAll\n");
					getBarrelStage = extendAll;
				}
			}
			else
			{
				printf("Going to stage idleReleased\n");
				getBarrelStage = idleReleased;
			}
			break;
		}

		case extendAll: //Open claw, wrist up, trolley to top
		{
			if(getBarrelButton)
			{
				GLOBAL_BARRELGRAB->ClawRelease();

				trolleyMotorCmd = GET_BARREL_TROLLEY_UP_CMD;

				printf("Going to stage pauseAll\n");
				getBarrelStage = pauseAllPressed;
			}
			else
			{
				printf("Going to stage idleReleased\n");
				craneMotorCmd = 0.0;
				trolleyMotorCmd = 0.0;
				getBarrelStage = idleReleased;
			}
		}
		break;

		case pauseAllPressed: //stop the trolley and crane when limits are reached
		{
			if (getBarrelButton)
			{
				trolleyMotorCmd = GET_BARREL_TROLLEY_UP_CMD;
				printf("Trolley: %f 3.84      Crane: %f 3.4\n", GLOBAL_BARRELGRAB->GetTrolleyHeight(), GetArmAngle());
				int trolleyHeight = GLOBAL_BARRELGRAB->GetTrolleyHeight();
				int armAngle = GetArmAngle();
				if(trolleyHeight >= MAX_TROLLEY_POSITION)
				{
					trolleyMotorCmd = 0.0;
				}

				if(armAngle >= (craneAutoGoal))
				{
					craneMotorCmd = 0.0; 
				}

				if((trolleyHeight >= MAX_TROLLEY_POSITION) && (armAngle >= (craneAutoGoal)))
				{
					loopCount = 0;
					printf("Going to stage idlePressed\n");
					getBarrelStage = idlePressed;
				}
			}
			else
			{
				printf("Going to stage retractAll\n");
				getBarrelStage = clawGrab;
			}
		}
		break;

		case idlePressed: //wait until button is released
		{
			if(!getBarrelButton)
			{
				loopCount = 0;
				printf("Going to stage clawGrab\n");
				getBarrelStage = clawGrab;
			}
		}
		break;

		case clawGrab: //Claw Pinch
		{
			if(!getBarrelButton)
			{
				GLOBAL_BARRELGRAB->ClawPinch();
				loopCount++;
				if(loopCount > GET_BARREL_DELAY)
				{
					loopCount = 0;
					printf("Going to stage retractAll\n");
					getBarrelStage = retractAll;
				}
			}
			else
			{
				GLOBAL_BARRELGRAB->ClawRelease();
				getBarrelStage = idlePressed;
			}
		}
		break;

		case retractAll: //Raise crane. When crane is halfway up, wrist down and trolley down
		{
			if(!getBarrelButton)
			{
				//Wait some time?
				craneMotorCmd = GET_BARREL_CRANE_UP_CMD; //ManualMove(1.0, false);
				printf("Going Up\n");

				if(GetArmAngle() <= HALF_ANGLE)
				{
					GLOBAL_BARRELGRAB->WristDown();
					trolleyMotorCmd = GET_BARREL_TROLLEY_DOWN_CMD;

					printf("Going to stage pauseAllPressed\n");
					getBarrelStage = pauseAllReleased;
				}
			}
			else
			{
				getBarrelStage = extendAll;
			}
		}
		break;

		case pauseAllReleased:
		{
			if(!getBarrelButton)
			{
				trolleyMotorCmd = GET_BARREL_TROLLEY_DOWN_CMD;
				craneMotorCmd = GET_BARREL_CRANE_UP_CMD;
				float trolleyHeight = GLOBAL_BARRELGRAB->GetTrolleyHeight();
				float armAngle = GetArmAngle();
				printf("Trolley: %f 2.46      Crane: %f 61.5\n", trolleyHeight, armAngle);
				if((trolleyHeight <= HALF_TROLLEY_POSITION))
				{
					printf("trolley stopping\n");
					trolleyMotorCmd = 0.0; //GLOBAL_BARRELGRAB->ManualMove(0.0, false);
				}

				if (armAngle <= (ANGLE_UPPER_LIMIT + 2.0))
				{
					printf("crane stopping\n");
					craneMotorCmd = 0.0; //ManualMove(0.0, false);
				}

				if((trolleyHeight <= HALF_TROLLEY_POSITION) && (armAngle <= (ANGLE_UPPER_LIMIT + 2.0)))
				{
					printf("Going to stage idleReleased\n");
					getBarrelStage = idleReleased;
				}
			}
			else
			{
				getBarrelStage = idlePressed;
			}
		}
        break;

		default: 
			printf("WARNING! invalid stage!\n");
			getBarrelStage = idleReleased;
			break;
	}

	if (inAutomation)
	{
		ControlledMove(craneMotorCmd);
		GLOBAL_BARRELGRAB->ControlledMove(trolleyMotorCmd);
	}
}

bool Crane::IsDoneExtending()
{
	float trolleyHeight = GLOBAL_BARRELGRAB->GetTrolleyHeight();
	float armAngle = GetArmAngle();

	if(((getBarrelStage == idlePressed) ||
	    (getBarrelStage == pauseAllPressed && trolleyHeight >= (MAX_TROLLEY_POSITION - 0.1) && armAngle >= craneAutoGoal))
        && inAutomation)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Crane::IsDoneRetracting()
{
	if((getBarrelStage == idleReleased) && inAutomation)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Crane::InAutomation()
{
	return inAutomation;
}

void Crane::StopAll()
{
	ArmStop();
	inAutomation = false;
	getBarrelStage = idleReleased;
}

void Crane::ControlSystemReset()
{
	errorInt = 0.0;
	lastPos = -(GetArmAngle()); // same as reading currentPos in VelControl
}

float Crane::VelControl(float velCmd)
{
	float velError = 0.0;
	float currentPos = 0.0;
	float currentVel = 0.0;


	currentPos = -(GetArmAngle());

	currentVel = (currentPos - lastPos) / LOOPTIME;
	currentVel = TalonXVI::Limit(-30.0, 30.0 ,currentVel);

/*
	printf("position %f ", currentPos);

	printf("velocity %f ", currentVel);

	printf("input %f", velCmd);*/

	velError = velCmd - currentVel;
	//printf("velocity Error %f", velError);

	errorInt = errorInt + (velError * LOOPTIME);
	//parameters need to be determined
	//errorInt = TalonXVI::Limit(-1.0, 1.0, errorInt);

	if(!velCmd && lastCmd)
	{
		errorInt = 0.0;
	}
	//printf("integral %f", errorInt);

	lastPos = currentPos;
	lastCmd = velCmd;


	return (CRANE_K_BANDWIDTH / CRANE_K_ROBOT * (CRANE_TAU * velError + errorInt));// tau was 0.02
	//return (TROLLEY_K_VEL * velError + errorInt * TROLLEY_K_VEL_INT); //bad
	//return velCmd;
}

void Crane::UpdateDash()
{
	SmartDashboard::PutNumber("Crane Angle", GetArmAngle());
}
#endif
#endif
