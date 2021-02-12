
#ifndef NAVIGATION_H_
#define NAVIGATION_H_


#define WHEEL_DISTANCE_PER_PULSE			(0.052359878) //circumference
#define K_WHEEL_CONSTANT					(1.0) //determined experimentally
#define R_WHEEL_CONSTANT					(0.028016591) //determined experimentally


class Navigation
{

	private:

	Encoder *encoderFL;
	Encoder *encoderRL;
	Encoder *encoderFR;
	Encoder *encoderRR;

	double theta;
	//double North;
	//double East;

	//double deltaX; //field to robot
	//double deltaY;

	double dx; //from meccanum wheels
	double dy;
	double dtheta;

	double oldDistanceA; //front left
	double oldDistanceB; //front right
	double oldDistanceC; //back right
	double oldDistanceD; //back left


	public:
	Navigation();
	void Service();
	void DeltaCalculations();
	//double GetDeltaX();
	//double GetDeltaY();
	double GetVelX();
	double GetVelY();
	//double GetDeltaTheta();
	double GetNorth();
	double GetEast();
	double GetTheta();
	void RobotToField(double *deltaNorth, double *deltaEast);
	void FieldToRobot(double deltaNorth, double deltaEast, double *deltaX, double *deltaY);
	void Abort();
	void UpdateDash();
	void LocalReset();
};

#endif
