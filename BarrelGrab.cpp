/*
 * BarrelGrab.cpp
 *
 *  Created on: January 22, 2015
 *      Author: jacob_000
 */

#include "WPILib.h"
#include "TalonXVI.h"
#include "BarrelGrab.h"
#include "Crane.h"

#ifdef USE_BARRELGRAB
#ifndef CALIBRATION

//Constructor
BarrelGrab::BarrelGrab()
{
#ifdef PRACTICE_BOT
	trolleyMotor = new Talon(PWM_TROLLEY);
#else
	trolleyMotor = new VictorSP(PWM_TROLLEY);
#endif

	claw = new DoubleSolenoid(CLAW_PISTON1, CLAW_PISTON2);
	wrist = new DoubleSolenoid(WRIST_PISTON1, WRIST_PISTON2);
	trolleyPot = new AnalogInput(ANALOG_TROLLEY);
	LocalReset();
}

//Initializes Variables
void BarrelGrab::LocalReset()
{
	//trolleyHeight = 0.0;
	isPinched = false;
	isWristUp = true;
	lastPos = 0.0;
	errorInt = 0.0;
}

void BarrelGrab::Service()
{
	//trolleyHeight = trolleyPot->Get();
	if(GetTrolleyHeight() >= TROLLEY_CLAW_LIMIT && GLOBAL_CRANE->GetArmAngle() <= CRANE_CLAW_LIMIT)
	{
		WristDown();
	}
}

//Stops the Mechanism
void BarrelGrab::StopAll()
{
	trolleyMotor->Set(0.0);
	inAutomation = false;
}

void BarrelGrab::JoystickMove(float power)
{
	if (inAutomation)
		return;

	power = -1.0 * power; //Needs to be reversed because Joystick is opposite
	if(fabs(power) <= JOYSTICK_DEADBAND)
	{
		power = 0.0;
	}
	ControlledMove(power);
}

void BarrelGrab::ControlledMove(float power)
{
	float motorCmd;
	float trolleyHeight = GetTrolleyHeight();

	//is moving up
	if(power > 0.0)
	{
		//if at top
		if(trolleyHeight >= MAX_TROLLEY_POSITION)
		{
			motorCmd = 0.0;
		}
		else
		{
			motorCmd = power * TROLLEY_MAX_RATE_UP;
		}
	}
	//is moving down
	else if(power < 0.0) //Necessary because power can be equal to 0.
	{
		//if at bottom
		if(trolleyHeight <= MIN_TROLLEY_POSITION)
		{
			motorCmd = 0.0;
		}
		else
		{
			motorCmd = power * TROLLEY_MAX_RATE_DOWN;
		}
	}
	else
	{
		motorCmd = 0.0;  // setting command to 0.0 if power is 0.0
	}


	motorCmd = -1.0 * motorCmd; //Motor moves backwards

#ifdef USE_TROLLEY_CONTROL_SYSTEM
	motorCmd = VelControl(motorCmd);  //control loop
#else
	// testing without control system
	if (motorCmd > 1.0)	// temporary
		motorCmd = 1.0;
	if (motorCmd < -1.0)
		motorCmd = -1.0;
	printf("trolley winchMotor output %f \n",motorCmd);
#endif
	trolleyMotor->Set(motorCmd);
	//printf("Trolley moving at %f (power %f)\n", motorCmd , power);
}

//Closes the claw
void BarrelGrab::ClawPinch()
{
	//printf("Claw PINCH\n");
	claw->Set(DoubleSolenoid::kReverse);
	isPinched = true;
}

//Opens the claw
void BarrelGrab::ClawRelease()
{
	//printf("Claw Release\n");
	claw->Set(DoubleSolenoid::kForward);
	isPinched = false;
}

//Moves the claw up
void BarrelGrab::WristUp()
{
	//printf("wrist UP\n");
	wrist->Set(DoubleSolenoid::kReverse);
	isWristUp = true;
}

//Moves the claw down
void BarrelGrab::WristDown()
{
	//printf("wrist DOWN\n");
	wrist->Set(DoubleSolenoid::kForward);
	isWristUp = false;
}

//Returns the variable isWristUp
bool BarrelGrab::IsWristUp()
{
	return isWristUp;
}

//Returns the variable isPinched
bool BarrelGrab::IsPinched()
{
	return isPinched;
}

float BarrelGrab::GetTrolleyHeight()
{
	float height = (trolleyPot->GetVoltage() * TROLLEY_INCHES_PER_VOLT) - TROLLEY_OFFSET_INCHES;
	return height;
}

void BarrelGrab::ControlSystemReset()
{
	errorInt = 0.0;
	lastPos = -1.0 * (trolleyPot->GetVoltage());  //must be done the same way as in VelControl
}

float BarrelGrab::VelControl(float velCmd)
{
	float velError = 0.0;
	float currentPos = 0.0;
	float currentVel = 0.0;

	currentPos = -1.0 * (trolleyPot->GetVoltage());
	//currentPos = (trolleyPot->Get() * 5.0);

	currentVel = (currentPos - lastPos) / LOOPTIME;
	currentVel = TalonXVI::Limit(-2.0, 2.0 ,currentVel);

	/*
	printf("position %f ", currentPos);
	printf("velocity %f ", currentVel);
	printf("command %f\n", velCmd);
	*/

	velError = velCmd - currentVel;
	//printf("velocity Error %f", velError);

	errorInt = errorInt + (velError * LOOPTIME);
	//parameters still need to be determined
	//errorInt = TalonXVI::Limit(-1.0, 1.0, errorInt);

	lastPos = currentPos;

	return (TROLLEY_K_BANDWIDTH / TROLLEY_K_ROBOT * (TROLLEY_TAU * velError + errorInt));// tau was 0.02
	//return (TROLLEY_K_VEL * velError + errorInt * TROLLEY_K_VEL_INT);
	//return velCmd;
}

void BarrelGrab::SetAutomation(bool automationOn)
{
	inAutomation = automationOn;
}


void BarrelGrab::UpdateDash()
{
	SmartDashboard::PutBoolean("Claw Pinched: ", IsPinched());
	SmartDashboard::PutBoolean("Wrist Up: ", IsWristUp());
	SmartDashboard::PutNumber("Trolley Height: ", GetTrolleyHeight());
}

//BarrelGrab::~BarrelGrab()
#endif
#endif


