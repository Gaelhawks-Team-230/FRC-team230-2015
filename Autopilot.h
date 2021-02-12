#ifndef _AUTOPILOT_H_
#define _AUTOPILOT_H_

#include "Navigation.h"
#include "TrajectoryPlanner.h"
#include "TrajectoryPlannerYaw.h"


#define K_POS_X					(2.5)
#define KF_X					(5.0)
#define K_X						(140.0)
#define TAU_X					(0.5)

#define K_POS_Y					(2.5)
#define KF_Y					(5.0)
#define K_Y						(140.0)
#define TAU_Y					(0.4)

#define KACCEL_X				(0.0)//(0.5 / 5.0)//(1.0 / 5.0)/
#define KACCEL_Y				(0.0)//(0.5 / 5.0)//(1.0 / 5.0)/

#define KVEL_X				(0.75)//(0.75)
#define KVEL_Y				(0.75)//(0.75)

#define ERROR_THRESHOLD			(0.5)//(1.0)//inches/sec
#define ERROR_THRESHOLD_YAW		(0.01)//radians/SEC

class Autopilot
{
public:

	Navigation *location;
	TrajectoryPlanner *plannerNorth;
	TrajectoryPlanner *plannerEast;
	TrajectoryPlannerYaw *plannerYaw;

private:

	double velGoalX;
	double velGoalY;
	double accelGoalX;
	double accelGoalY;

	double accelGoalNorth;
	double posGoalNorth;
	double velGoalNorth;

	double accelGoalEast;
	double posGoalEast;
	double velGoalEast;

	double accelGoalYaw;
	double posGoalYaw;
	double velGoalYaw;

	double velCmdX;
	double velCmdY;

	double accelCmdX;
	double accelCmdY;

	double errorX;
	double errorY;

	double North;
	double East;

	double deltaNorth;
	double deltaEast;

	double velErrorIntX;
	double velErrorIntY;

	bool atGoal;

public:

	Autopilot(void);
	void LocalReset(void);
	void StopAll(void);
	void UpdateDash(void);
	void Service(void);
	double ControlLoopX(void);
	double ControlLoopY(void);
	void AcquireGoals(void);
	void CalcRobotError(void);
	bool IsAtGoal(void);
	void ForcedReset (void);
	void SetGoals(double northGoal, double eastGoal, double yawGoal);
};


#endif
