#include "WPILib.h"
#include "TalonXVI.h"
#include "Navigation.h"
#include "TrajectoryPlanner.h"
#include "TrajectoryPlannerYaw.h"
#include "Autopilot.h"

#ifndef CALIBRATION

Autopilot::Autopilot()
{
	location = new Navigation(); //MAX remove
	plannerNorth = new TrajectoryPlanner();
	plannerEast = new TrajectoryPlanner();
	plannerYaw = new TrajectoryPlannerYaw();
	LocalReset();
}



void Autopilot::LocalReset()
{
	velGoalX = 0.0;
	velGoalY = 0.0;
	accelGoalX = 0.0;
	accelGoalY = 0.0;

	accelGoalNorth = 0.0;
	posGoalNorth = 0.0;
	velGoalNorth = 0.0;

	accelGoalEast = 0.0;
	posGoalEast = 0.0;
	velGoalEast = 0.0;

	accelGoalYaw = 0.0;
	posGoalYaw = 0.0;
	velGoalYaw = 0.0;

	velErrorIntX = 0.0;
	velErrorIntY = 0.0;

	North = 0.0;
	East = 0.0;

	deltaNorth = 0.0;
	deltaEast = 0.0;

	atGoal = true;
}

void Autopilot::StopAll()
{
	LocalReset();
	plannerNorth->Abort();
	plannerEast->Abort();
	plannerYaw->Abort();
	location->Abort();
	GLOBAL_DRIVE->DriveControl(0.0, 0.0, 0.0);
}

bool Autopilot::IsAtGoal()
{
	return atGoal;
}


void Autopilot::UpdateDash()
{


}


void Autopilot::Service()
{
	double xCmd = 0.0;
	double yCmd = 0.0;
	double zCmd = 0.0;

	AcquireGoals(); // receives goals (north, east) from trajectory planner
	CalcRobotError(); // calculates error (north, east) and converts to robot x,y.

	xCmd = (float)(ControlLoopX()); //generates motor command X
	yCmd = (float)(ControlLoopY()); //generates motor command Y
	zCmd = (float)(velGoalYaw);

	double offFromGoalNorth = plannerNorth->GetPositionCmd() - North;
	double offFromGoalEast = plannerEast->GetPositionCmd() - East;
	//double offFromGoalYaw = plannerYaw->GetPositionCmd() - GLOBAL_DRIVE->GetGyroAngle();

	//printf("offFromGoalNorth: %f offFromGoalEast: %f \n", offFromGoalNorth, offFromGoalEast);
	//printf("NorthInfo: %f %f %f EastInfo: %f %f %f \n", plannerNorth->GetPositionCmd(), posGoalNorth, location->GetNorth(),
														//plannerEast->GetPositionCmd(), posGoalEast, location->GetEast());

	//if(fabs(offFromGoalNorth) <= ERROR_THRESHOLD  && fabs(offFromGoalEast) <= ERROR_THRESHOLD && fabs(offFromGoalYaw) <= ERROR_THRESHOLD_YAW)
	//if(fabs(offFromGoalNorth) <= ERROR_THRESHOLD  && fabs(offFromGoalEast) <= ERROR_THRESHOLD && fabs(velGoalYaw) <= ERROR_THRESHOLD_YAW)
	if(fabs(velGoalNorth) <= ERROR_THRESHOLD  && fabs(velGoalEast) <= ERROR_THRESHOLD && fabs(velGoalYaw) <= ERROR_THRESHOLD_YAW)
	{
		atGoal = true;
	}
	else
	{
		atGoal = false;
	}


	/*static int loopCount = 0;
	loopCount++;
	if(loopCount > 50)
	{
		xCmd = 0.0;	yCmd = 0.0;
	}
	else
	{
		xCmd = 0.5;	yCmd = 0.0;
	}*/
	GLOBAL_DRIVE->DriveControl(xCmd, yCmd, zCmd);
	//printf(" pos: %f   vel: %f    cmd: %f\n",  location->GetNorth(), location->GetVelX(), xCmd);
}

double Autopilot::ControlLoopX()
{
	double velCmd = 0.0;
	double velError = 0.0;
	double motorCmd = 0.0;
	double feedForward = 0.0;

	feedForward = KACCEL_X * accelGoalX + KVEL_X * velGoalX;

	velCmd = errorX * K_POS_X + feedForward;

	printf("errorX %f ", errorX);

	velError = velCmd - (location->GetVelX());

	velErrorIntX = velErrorIntX + (velError * LOOPTIME);

	motorCmd = (KF_X / K_X) * ((TAU_X * velError) + velErrorIntX);

	//printf("velCmd: %f velErr %f motorCmd %f velocity %f velErrInt %f \n", velCmd, velError, motorCmd, location->GetVelX(), velErrorIntX);

	return motorCmd;
}

double Autopilot::ControlLoopY()
{
	double velCmd = 0.0;
	double velError = 0.0;
	double motorCmd = 0.0;
	double feedForward = 0.0;

	feedForward = KACCEL_Y * accelGoalY + KVEL_Y * velGoalY;

	velCmd = errorY * K_POS_Y + feedForward;

	//printf("errorY %f ", errorY);

	velError = velCmd - (location->GetVelY());

	velErrorIntY = velErrorIntY + (velError * LOOPTIME);

	motorCmd = (KF_Y / K_Y) * ((TAU_Y * velError) + velErrorIntY);

	return motorCmd;
}


void Autopilot::AcquireGoals() // stores life goals from trajectory planner
{
	plannerNorth->Calculate(&posGoalNorth, &velGoalNorth, &accelGoalNorth);
	//printf("PosGoalNorth: %f ", posGoalNorth);
	//printf("PosGoalNorth: %f     velGoalNorth: %f       accelGoalNorth: %f \n", posGoalNorth, velGoalNorth, accelGoalNorth);
	plannerEast->Calculate(&posGoalEast, &velGoalEast, &accelGoalEast);
	//printf("PosGoalEast: %f     velGoalEast: %f       accelGoalEast: %f \n", posGoalEast, velGoalEast, accelGoalEast);
	//printf("PosGoalEast: %f ", posGoalEast);
	plannerYaw->Calculate(&posGoalYaw, &velGoalYaw, &accelGoalYaw);
	//printf("PosGoalYaw: %f ", posGoalYaw);
}


void Autopilot::CalcRobotError() //converts current location to error in robot x,y
{
	location->DeltaCalculations(); //calculates position from wheel encoders
	location->RobotToField(&deltaNorth, &deltaEast);
	North = North + deltaNorth;
	East = East + deltaEast;



	//printf("posNorth %f   ", posNorth);
	//printf("posEast %f\n", posEast);
	//printf("posAngle %f\n", location->GetTheta());


	double errorNorth = posGoalNorth - North; // error
	double errorEast = posGoalEast - East;

	//printf("error North %f", errorNorth);
	//printf("error East %f", errorEast);

	location->FieldToRobot(errorNorth, errorEast, &errorX, &errorY); //conversion

	//errorX = location->GetDeltaX(); //receive error
	//errorY = location->GetDeltaY();

	//errorX = errorNorth;
	//errorY = errorEast;

	//printf("error x %f\n", errorX);

	//convert accel and vel
	location->FieldToRobot(velGoalNorth, velGoalEast, &velGoalX, &velGoalY);
	location->FieldToRobot(accelGoalNorth, accelGoalEast, &accelGoalX, &accelGoalY);

	/*location->FieldToRobot(accelGoalNorth, accelGoalEast);
	accelCmdX = location->GetDeltaX();
	accelCmdY = location->GetDeltaY();*/
	//accelCmdX = accelGoalNorth;
	//accelCmdY = accelGoalEast;

}

void Autopilot::SetGoals(double northGoal, double eastGoal, double yawGoal) //life goals
{
	plannerNorth->SetPositionCmd(northGoal);
	plannerEast->SetPositionCmd(eastGoal);
	plannerYaw->SetPositionCmd(yawGoal);

}

void Autopilot::ForcedReset()
{
	plannerNorth->ForcedReset(North);
	plannerEast->ForcedReset(East);
	plannerYaw->ForcedReset((double)(GLOBAL_DRIVE->GetGyroAngle()));
	velErrorIntX = 0.0;
	velErrorIntY = 0.0;

}

#endif


