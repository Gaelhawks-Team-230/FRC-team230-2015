#include "WPILib.h"
#include "TalonXVI.h"
#include "ToteGrab.h"

#ifndef CALIBRATION

ToteGrab::ToteGrab()
{
#ifdef PRACTICE_BOT
	wheel = new Talon(PWM_TOTEGRAB);
#else
	wheel = new VictorSP(PWM_TOTEGRAB);
#endif
	limitSwitch = new DigitalInput(DIGITAL_TOTEGRAB_SWITCH);
	totePistons = new DoubleSolenoid(TOTE_PISTON1, TOTE_PISTON2);
	LocalReset();
}

//checks every loop to make sure the tote has not fallen out
void ToteGrab::Service()
{
	if (holdingTote)
		GetTote();
}

void ToteGrab::LocalReset()
{
	wheel->Set(0.0);
	holdingTote = false;
	pistonSqueezed = false;
}

void ToteGrab::StopAll()
{
	LocalReset();
}

void ToteGrab::StopMotor()
{
	wheel->Set(0.0);
	holdingTote = false;
}

//spins wheels in reverse to pick up tote until limit switch is pressed
void ToteGrab::GetTote()
{
	holdingTote = true;
	if (limitSwitch->Get()) //limit switch is BACKWARDS
	{
		wheel->Set(-1.0);
	}
	else
	{
		wheel->Set(0.0);
	}
}

//spins wheels to let go of tote
void ToteGrab::EjectTote()
{
	holdingTote = false;
	wheel->Set(1.0);
}

//pushes pistons against the tote
void ToteGrab::SqueezeTote()
{
	//printf("SQUEEZE tote\n");
	totePistons->Set(DoubleSolenoid::kForward);
	pistonSqueezed = true;
}

//lets go of tote by bring the pistons in
void ToteGrab::ReleaseTote()
{
	//printf("Release tote\n");
	totePistons->Set(DoubleSolenoid::kReverse);
	pistonSqueezed = false;
}

void ToteGrab::ToteAcquireAutomation(bool btnPressed)
{
	switch(grabStage)
	{
		case IdleReleased:
		{
			if (btnPressed)
			{
				inAutomation = true;
				grabStage = ElevatorUp;
				printf ("Button pressed, going to stage ElevatorUp\n");
			}
			else
			{
				inAutomation = false;
			}
			break;
		}
		case ElevatorUp:
		{
			if (btnPressed)
			{
				if (GLOBAL_ELEVATOR->GetHeight() < TOTE_GRAB_ELEVATOR_HEIGHT)
				{
					GLOBAL_ELEVATOR->GoToHeight(TOTE_GRAB_ELEVATOR_HEIGHT, true);
					grabStage = CheckHeight;
					printf ("GoToHeight has been initialized.");
				}
				else
				{
					printf ("Elevator already above 27 inches, going to stage CloseToteGrab\n");
					grabStage = CloseToteGrab;
				}
			}
			else
			{
				GLOBAL_ELEVATOR->StopSeekingHeight();
				grabStage = IdleReleased;
				printf ("Button has been released, going to IdleReleased\n");
			}
			break;
		}
		case CheckHeight:
		{
			if(btnPressed)
			{
				if(GLOBAL_ELEVATOR->GetHeight() < TOTE_GRAB_ELEVATOR_HEIGHT)
				{
					GLOBAL_ELEVATOR->GoToHeight(TOTE_GRAB_ELEVATOR_HEIGHT);
					printf ("Elevator is moving up. Current Height:  %f \n", GLOBAL_ELEVATOR->GetHeight());
				}
				else
				{
					grabStage = CloseToteGrab;
					printf("Elevator is now at desired height, moving to stage CloseToteGrab\n");
				}
			}
			else
			{
				GLOBAL_ELEVATOR->StopSeekingHeight();
				grabStage = IdleReleased;
				printf ("Button has been released, going to IdleReleased\n");
			}
		}
		case CloseToteGrab:
		{
			if (btnPressed)
			{
			SqueezeTote();
			GetTote();
			grabStage = IdlePressed;
			printf ("Squeezing tote and wheels spinning, moving to stage IdlePressed\n");
			}
			else
			{
				grabStage = IdleReleased;
				printf ("button released, moving to IdleReleased\n");
			}
			break;
		}
		case IdlePressed:
		{
			if (btnPressed)
			{
				printf ("button is still pressed and will stay in this case\n");
			}
			else
			{
				grabStage = OpenToteGrab;
				printf ("button released, moving OpenToteGrab\n");
			}
			break;
		}
		case OpenToteGrab:
		{
			StopMotor();
			ReleaseTote();
			//grabStage = ElevatorDown;	//needed if we want to pick up tote after automation
			grabStage=IdleReleased;
			printf ("stopped motor and opened tote grabber, moving to IdleReleased\n");
			break;
		}
		/*case ElevatorDown:
		{
			if(GLOBAL_ELEVATOR->GetHeight() > ELEVATOR_GET_TOTE_HEIGHT)
			{
				GLOBAL_ELEVATOR->GoToHeight(ELEVATOR_GET_TOTE_HEIGHT);
			}
			else
			{
				grabStage = TotePickUp;
			}
			break;
		}
		case TotePickUp:
		{
			if(GLOBAL_ELEVATOR ->GetHeight() < TOTE_PICKUP_HEIGHT)
			{
				GLOBAL_ELEVATOR->GoToHeight(TOTE_PICKUP_HEIGHT);
			}
			else
			{
				grabStage=Idle;
			}
			break;
		}*/
	}
}

//will say if we have the tote
bool ToteGrab::HasTote()
{
	return (!(limitSwitch->Get()));
}

bool ToteGrab::IsInAutomation()
{
	return inAutomation;
}

//tells if robot has the tote or not on the dash
void ToteGrab::UpdateDash()
{
	//SmartDashboard::PutBoolean("Holding Tote", holdingTote);
	SmartDashboard::PutBoolean("Pistons Out", pistonSqueezed);
	SmartDashboard::PutBoolean("LimitSwitch Pressed", HasTote());
}
#endif
