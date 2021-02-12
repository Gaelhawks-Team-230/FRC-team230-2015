#include "WPILib.h"
#include "Common.h"


#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#define K_GYRO		(10.0)		//Kf
#define K_ROBOT		(5.1)		//K
#define TAU_ROBOT	(0.15)


class DriveTrain
{
public:
#ifdef PRACTICE_BOT
	Talon *frontLeftMotor, *frontRightMotor, *rearLeftMotor, *rearRightMotor;
#else
	VictorSP *frontLeftMotor, *frontRightMotor, *rearLeftMotor, *rearRightMotor;
#endif

#ifdef USE_GYRO
	Gyro *gyro;
#endif

	float motorCommandFL, motorCommandFR, motorCommandRL, motorCommandRR;

	bool gyroOn;
	float gyroVel;
	float gyroAngle;
	float gyroErr;
	float gyroErrInt; //Integral


	DriveTrain(void);
	void ResetGyro();
	void JoystickDrive(float command_x, float command_y, float command_z, bool trigger);
	void DriveControl(float command_x, float command_y, float command_z);
	void GyroOn();
	void GyroOff();
	float GyroControl(float zCmd);
	float GetGyroAngle();
	float GetGyroVel();
	void UpdateDash();
	void Reset();
	void StopAll();
};

#endif /* SRC_DRIVETRAIN_H_ */
