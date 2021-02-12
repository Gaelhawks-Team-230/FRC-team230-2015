#include "WPILib.h"
#include "TalonXVI.h"
#include "Elevator.h"

#ifndef CALIBRATION

Elevator::Elevator()
{
#ifdef PRACTICE_BOT
	liftMotor = new Talon(PWM_LIFT);
#else
	liftMotor = new VictorSP(PWM_LIFT);
#endif

	inchesperpulse = 0.01528; //measured at 0.015111;
	heightEncoder = new Encoder( ELEVATOR_ENCODER_B, ELEVATOR_ENCODER_A);  //reversed A and B to count in the correct direction
#ifdef USE_TOO_CLOSE_SENSOR
	toteSensor = new DigitalInput(DIGITAL_TOTE_SENSOR);
#endif
	heightEncoder->SetDistancePerPulse(inchesperpulse);
	LocalReset();

}

void Elevator::LocalReset()
{
	heightEncoder->Reset();
	atGoalHeight = true;
	goalHeight = 0.0;
	lastPos = 0.0;
	errorInt = 0.0;
	motorStall = false;
}

void Elevator::StopAll()
{
	motorCmd = 0.0;
	liftMotor->Set(0.0);
	atGoalHeight = true;
}

//Finds the current height of the elevator
double Elevator::GetHeight()
{
	return heightEncoder->GetDistance();
}

//Finds the level within which the current elevator height is found. The bottom of the level is considered the boundary for that level.
int Elevator::FindLevel()
{
	return (int)(GetHeight()/LEVEL_HEIGHT);
}

//Manually move the elevator up or down.
void Elevator::ManualMove(float power)
{
	//This will prevent the joystick from sending commands while seeking a height.
	if(!atGoalHeight)
	{
		return;
	}

	power = -1.0 * power; //Joystick is opposite of what we want
	if(fabs(power) > LIFT_JOYSTICK_DEADBAND)
	{
		//seekingHeight = false; // commented out because want service routine
		if(power > 0.0)
		{
			//If at or above maximum height
			if(GetHeight() >= MAX_ELEVATOR_HEIGHT || tooClose())
			{
				motorCmd = 0.0;
			}
			else //Not at the maximum height
			{
				motorCmd = power * MAX_ELEVATOR_SPEED_UP;
				// WTF - GLOBAL_TOTEGRAB->StopMotor();
			}
		}
		else
			//If at or below minimum height
			if(GetHeight() <= MIN_ELEVATOR_HEIGHT)
			{
				motorCmd = 0.0;
			}
			else //Not at the minimum height
			{
				motorCmd = power * MAX_ELEVATOR_SPEED_DOWN;
			}
		}
	else
	{
		motorCmd = 0.0;
	}

	motorCmd = VelControl(motorCmd);//alters for control system
	//printf("output %f \n",motorCmd);
	liftMotor->Set(motorCmd);
	//printf("Elevator moving at %f (power %f)\n", motorCmd , power);
}

//Go to a specific height, such as the bottom or a starting height.
void Elevator::GoToHeight(double destinationHeight, bool Init /*= false*/) // Init = false if nothing is specified
{
	float currentPosit;
	float PcmdV;
	float Perror;
	float Vcmd;
	float lastPcmd;
	float MotorCmd;
	currentPosit = GetHeight();
	if(Init)
	{
		atGoalHeight = false;
		Pcmd = currentPosit;
	}
	lastPcmd = Pcmd;
	Pcmd = Pcmd + TalonXVI::Limit(-ELEVATOR_VLIMIT*LOOPTIME, ELEVATOR_VLIMIT*LOOPTIME, destinationHeight - Pcmd);
	PcmdV = (Pcmd - lastPcmd)/LOOPTIME;
	Perror = Pcmd - currentPosit;
	Vcmd = ELEVATOR_KP*Perror + ELEVATOR_KFF*PcmdV;
	if(abs(destinationHeight - Pcmd) < 0.01)
	//if(0)
	{
		atGoalHeight = true;
		Vcmd = 0.0;
		printf("Elevator extremely close to goal, done moving.\n");
	}
	else
	{
		atGoalHeight = false;
		printf("Elevator not yet at goal, Current Height: %f \n", Pcmd);
	}
	MotorCmd = VelControl(Vcmd);
	liftMotor->Set(MotorCmd);
}

void Elevator::StopSeekingHeight()
{
	atGoalHeight = true;
	//printf("in StopSeekingHeight..........");
	ManualMove(0.0);
}

void Elevator::Service()
{

	if(GetHeight() <= TOTE_GRAB_LIMIT)
	{
		GLOBAL_TOTEGRAB->ReleaseTote();
		GLOBAL_TOTEGRAB->StopMotor();
		//printf("Releasing Tote");
	}
}

bool Elevator::atGoal()
{
	return atGoalHeight;
}

void Elevator::ControlSystemReset()
{
	errorInt = 0.0;
	lastPos = (GetHeight());  //same as currentPos in VelControl
}

float Elevator::VelControl(float velCmd)
{
	float velError = 0.0;
	float currentPos = 0.0;
	float currentVel = 0.0;
	float output = 0.0;

	if(velCmd < -0.1)
	{
		motorStall = false;
	}

	if(motorStall)
	{
		velCmd = 0.0;
	}

	currentPos = (GetHeight());
	//currentPos = (trolleyPot->Get() * 5.0);

	currentVel = (currentPos - lastPos) / LOOPTIME;
	currentVel = TalonXVI::Limit(-25.0, 20.0 ,currentVel);
/*FOR MIKE
	printf("position %f ", currentPos);

	printf("velocity %f ", currentVel);

	printf("input %f", velCmd);
	*/

	velError = velCmd - currentVel;
	//printf("velocity Error %f", velError);

	errorInt = errorInt + (velError * LOOPTIME);
	errorInt = TalonXVI::Limit(-4.0, 4.0, errorInt);

	//printf("integral %f", errorInt);

	lastPos = currentPos;
	output = ELEVATOR_K_BANDWIDTH / ELEVATOR_K_ROBOT * (ELEVATOR_TAU * velError + errorInt);

	if((currentVel <= 1.0) && (output >= 1.0))
	{
		output = 0.0;
		errorInt = -0.55;
		motorStall = true;
	}

	return (output);// tau was 0.02
	//return (TROLLEY_K_VEL * velError + errorInt * TROLLEY_K_VEL_INT); //bad
	//return velCmd;
}

//Tests if the current level is the level input for the Smart Dashboard.
bool Elevator::IsAtLevel(int levelTest)
{
	return (FindLevel() >= levelTest);
}

//Tests whether totes are too close to the top
bool Elevator::tooClose()
{
#ifdef USE_TOO_CLOSE_SENSOR
	return (toteSensor->Get());
#else
	return false;
#endif
}

void Elevator::StartFromTheBottom()
{
	int curCount = 10;
	int lastCount = 1000;
	int loopCount = 0;

	// start moving down very slowly
	liftMotor->Set(-0.25);
	Wait(0.1);

	while(curCount != lastCount)
	{
		lastCount = curCount;
		curCount = heightEncoder->Get();
		//printf("%d -- Last: %d Cur: %d \n", loopCount, lastCount, curCount);
		loopCount++;
		if (loopCount >= 1000)
		{
			break;
		}
		Wait(0.04);
	}

	//now at bottom
	printf("at Bottom!\n");
	liftMotor->Set(0.0);
	heightEncoder->Reset();
}


void Elevator::UpdateDash()
{
	SmartDashboard::PutNumber("Elevator Height: ", GetHeight()); //Get()
	//SmartDashboard::PutNumber("Elevator Raw: ", heightEncoder->GetRaw());
	//SmartDashboard::PutNumber("Elevator Distance: ", heightEncoder->GetDistance());
	SmartDashboard::PutNumber("Elevator Level: ", FindLevel());
	SmartDashboard::PutBoolean("6", IsAtLevel(6));
	SmartDashboard::PutBoolean("5", IsAtLevel(5));
	SmartDashboard::PutBoolean("4", IsAtLevel(4));
	SmartDashboard::PutBoolean("3", IsAtLevel(3));
	SmartDashboard::PutBoolean("2", IsAtLevel(2));
	SmartDashboard::PutBoolean("1", IsAtLevel(1));
	SmartDashboard::PutBoolean("0", IsAtLevel(0));
}
#endif
