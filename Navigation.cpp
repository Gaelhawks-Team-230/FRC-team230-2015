#include <Math.h>
#include "WPILib.h"
#include "TalonXVI.h"

#include "Navigation.h"

#ifndef CALIBRATION

Navigation::Navigation()
{
	encoderFL = new Encoder(FRONT_LEFT_WHEEL_B, FRONT_LEFT_WHEEL_A);  //flipped
	encoderRL = new Encoder(REAR_LEFT_WHEEL_B, REAR_LEFT_WHEEL_A);    //flipped
	encoderFR = new Encoder(FRONT_RIGHT_WHEEL_A, FRONT_RIGHT_WHEEL_B);
	encoderRR = new Encoder(REAR_RIGHT_WHEEL_A, REAR_RIGHT_WHEEL_B);

	encoderFL->SetDistancePerPulse(WHEEL_DISTANCE_PER_PULSE);
	encoderRL->SetDistancePerPulse(WHEEL_DISTANCE_PER_PULSE);
	encoderFR->SetDistancePerPulse(WHEEL_DISTANCE_PER_PULSE);
	encoderRR->SetDistancePerPulse(WHEEL_DISTANCE_PER_PULSE);


	LocalReset();
}

void Navigation::LocalReset()
{
	dx = 0.0;
	dy = 0.0;
	dtheta = 0.0;

	theta = 0.0;
	//North = 0.0;
	//East = 0.0;

	//North = 0.0;
	//East = 0.0;

	oldDistanceA = 0.0;
	oldDistanceB = 0.0;
	oldDistanceC = 0.0;
	oldDistanceD = 0.0;


}


void Navigation::DeltaCalculations()
{
	double distanceA = 0.0;
	double distanceB = 0.0;
	double distanceC = 0.0;
	double distanceD = 0.0;

	distanceA = encoderFL->GetDistance(); //get encoder values
	distanceB = encoderRL->GetDistance();
	distanceC = encoderFR->GetDistance();
	distanceD = encoderRR->GetDistance();

	/*
	printf("encoderFL %f  ", distanceA);
	printf("encoderRL %f  ", distanceB);
	printf("encoderFR %f  ", distanceC);
	printf("encoderRR %f  \n", distanceD);
	*/

	double deltaDistanceA = distanceA - oldDistanceA; //calc error in distance
	double deltaDistanceB = distanceB - oldDistanceB;
	double deltaDistanceC = distanceC - oldDistanceC;
	double deltaDistanceD = distanceD - oldDistanceD;

	oldDistanceA = distanceA; //store values until next loop
	oldDistanceB = distanceB;
	oldDistanceC = distanceC;
	oldDistanceD = distanceD;

	dx = (deltaDistanceA + deltaDistanceB + deltaDistanceC + deltaDistanceD) * (K_WHEEL_CONSTANT / 4); //Mr. Spoldi's equations
	dy = (deltaDistanceA - deltaDistanceB - deltaDistanceC + deltaDistanceD) * (K_WHEEL_CONSTANT / 4);
#ifdef USE_GYRO
	theta = (double)(GLOBAL_DRIVE->GetGyroAngle()); //change in rotation in radians


#else
	dtheta = (deltaDistanceA + deltaDistanceB - deltaDistanceC - deltaDistanceD) * (K_WHEEL_CONSTANT / 4) * (1 / R_WHEEL_CONSTANT);
	theta = theta + dtheta; //integrate theta
#endif
}

/*double Navigation::GetDeltaX()
{
	return deltaX;
}

double Navigation::GetDeltaY()
{
	return deltaY;
}*/

double Navigation::GetVelX()
{
	//return (deltaX / LOOPTIME);
	return (dx / LOOPTIME);
}

double Navigation::GetVelY()
{
	//return (deltaY / LOOPTIME);
	return (dy / LOOPTIME);
}

/*double Navigation::GetDeltaTheta()
{
	return dtheta;
}*/

/*double Navigation::GetNorth()
{
	return North;
}

double Navigation::GetEast()
{
	return East;
}*/

double Navigation::GetTheta()
{
	return theta; //actual angle
}

void Navigation::RobotToField(double *deltaNorth, double *deltaEast)
{
	double theta2;
	theta2 = (double)(GLOBAL_DRIVE->GetGyroAngle()) + (double)(GLOBAL_DRIVE->GetGyroVel()) * LOOPTIME;
	*deltaNorth = cos(theta2)*dx - sin(theta2)*dy; // x, y to North, East
	*deltaEast = sin(theta2)*dx + cos(theta2)*dy;

	//*deltaNorth = North + dx;
	//*deltaEast = East + dy;
}

void Navigation::FieldToRobot(double deltaNorth, double deltaEast, double *deltaX, double *deltaY)
{
	double theta2;
	theta2 = (double)(GLOBAL_DRIVE->GetGyroAngle()) + (double)(GLOBAL_DRIVE->GetGyroVel()) * LOOPTIME;
	*deltaX =  cos(theta2)*deltaNorth + sin(theta2)*deltaEast; //North, East to x, y
	*deltaY = -sin(theta2)*deltaNorth + cos(theta2)*deltaEast;
	//*deltaX = deltaNorth;
	//*deltaY = deltaEast;
}

void Navigation::Abort()
{
	encoderFL->Reset(); //reset encoders
	encoderRL->Reset();
	encoderFR->Reset();
	encoderRR->Reset();

	LocalReset();
}


void Navigation::UpdateDash()
{

}

void Navigation::Service()
{

}

#endif
