/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: Max
 */
#include "WPILib.h"
#include "TalonXVI.h"
#include "DriveTrain.h"


#ifndef CALIBRATION
DriveTrain::DriveTrain()
{
#ifdef PRACTICE_BOT
	frontLeftMotor = new Talon(PWM_FRONT_LEFT);
	frontRightMotor = new Talon(PWM_FRONT_RIGHT);
	rearLeftMotor = new Talon(PWM_REAR_LEFT);
	rearRightMotor = new Talon(PWM_REAR_RIGHT);
#else
	frontLeftMotor = new VictorSP(PWM_FRONT_LEFT);
	frontRightMotor = new VictorSP(PWM_FRONT_RIGHT);
	rearLeftMotor = new VictorSP(PWM_REAR_LEFT);
	rearRightMotor = new VictorSP(PWM_REAR_RIGHT);
#endif

#ifdef USE_GYRO
	gyro = new Gyro(GYRO_ANALOG_INPUT);
#endif

	motorCommandFL = 0.0;
	motorCommandFR = 0.0;
	motorCommandRL = 0.0;
	motorCommandRR = 0.0;
	gyroOn = true;
	gyroErr = 0.0;
	gyroErrInt = 0.0;
	gyroVel = 0.0;
	gyroAngle = 0.0;
}

void DriveTrain::ResetGyro()
{
	gyro->Reset();
}

void DriveTrain::JoystickDrive(float joy_x, float joy_y, float joy_z, bool trigger)
{
	//printf("joystick %f %f %f \n", joy_x, joy_y, joy_z);
	//sends joystick values as appropriate motor direction commands
	if (fabs(joy_x) <= DRIVE_DEADBAND)
		joy_x = 0.0;
	else if(joy_x > 0.0)
		joy_x = (joy_x - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
	else
		joy_x = (joy_x + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);

	if (fabs(joy_y) <= DRIVE_DEADBAND)
		joy_y = 0.0;
	else if(joy_y > 0.0)
		joy_y = (joy_y - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
	else
		joy_y = (joy_y + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);

	if (fabs(joy_z) <= DRIVE_DEADBAND)
		joy_z = 0.0;
	else if(joy_z > 0.0)
		joy_z = (joy_z - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
	else
		joy_z = (joy_z + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
	joy_z = joy_z * 1.5; // make it turn faster with joystick please

	if(!trigger)
	{
		joy_z = 0;
	}

	DriveControl((joy_y * -1), joy_x, joy_z);
}

// positive command_x moves forward
// y right
void DriveTrain::DriveControl(float command_x, float command_y, float command_z)
{

	float modified_z;
	//printf("drive %f %f %f \n", command_x, command_y, command_z);

#ifdef USE_GYRO
	if (gyroOn)
	{
		modified_z = GyroControl(command_z);
	}
	else
#endif
	{
		modified_z = command_z * 0.25; //Need to make this higher
	}

	motorCommandFL = command_x + command_y + modified_z;
	frontLeftMotor->Set(motorCommandFL);

	motorCommandFR = command_x - command_y - modified_z;
	frontRightMotor->Set(-1.0*motorCommandFR);  // on opposite side of the robot, needs inverted command

	motorCommandRL = command_x - command_y + modified_z;
	rearLeftMotor->Set(motorCommandRL);

	motorCommandRR = command_x + command_y - modified_z;
	rearRightMotor->Set(-1.0*motorCommandRR);	// on opposite side of the robot, needs inverted command
	//printf("motors %f %f %f %f\n", motorCommandFL, motorCommandFR, motorCommandRL, motorCommandRR);
}

float DriveTrain::GyroControl(float zCmd)
{
#ifdef USE_GYRO
	gyroVel = GetGyroVel();

	gyroErr = zCmd - gyroVel;
	gyroErrInt = gyroErrInt + (gyroErr * LOOPTIME);
	float command_z = (K_GYRO / K_ROBOT) * (TAU_ROBOT * gyroErr + gyroErrInt);
	//printf("Gyro Angle: %f    Gyro Velocity: %f        Z Cmd: %f \n", GetGyroAngle() , gyroVel, zCmd);
	//return zCmd;
	return command_z;
#else
	return zCmd;
#endif
}

#ifdef USE_GYRO
float DriveTrain::GetGyroAngle() //Returns angle in radians
{
	float accessorGyroAngle = gyro->GetAngle() * DEG_TO_RAD;
	return accessorGyroAngle;
}

float DriveTrain::GetGyroVel()  //Returns velocity in radians/second
{
	float accessorGyroVel = gyro->GetRate() * DEG_TO_RAD;
	return accessorGyroVel;
}
#endif

void DriveTrain::UpdateDash()
{
	SmartDashboard::PutBoolean("Gyro", gyroOn);
#ifdef USE_GYRO
	SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
#endif
}

void DriveTrain::GyroOn()
{
#ifdef USE_GYRO
	gyroOn = true;
#else
	gyroOn = false;	// no gyro = always off
#endif
}

void DriveTrain::GyroOff()
{
	gyroOn = false;
}


void DriveTrain::Reset()
{
	motorCommandFL = 0.0;
	motorCommandFR = 0.0;
	motorCommandRL = 0.0;
	motorCommandRR = 0.0;
	gyroOn = false;
	DriveControl(0.0, 0.0, 0.0);
}

void DriveTrain::StopAll()
{
	Reset();
}
#endif
