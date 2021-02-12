#ifndef SRC_CRANE_H_
#define SRC_CRANE_H_

#ifdef PRACTICE_BOT
#define ANGLE_UPPER_LIMIT		(57.5)
#define ANGLE_LOWER_LIMIT		(135.5)
#define BARRELGRAB_FLAT_LOWER_LIMIT	(127.0)
#define BARRELGRAB_TILT_LOWER_LIMIT	(124.5)
#define CRANE_MAX_SPEED_UP		(20.0)
#define CRANE_MAX_SPEED_DOWN	(20.0)
#define CRANE_CLAW_LIMIT		(87.0)
#else
#define ANGLE_UPPER_LIMIT		(55.5)//(53.0) //(52.63)// degrees //(1.385) volts //Top
#define ANGLE_LOWER_LIMIT		(129.2)//degrees //(3.4) volts	//Bottom
#define BARRELGRAB_FLAT_LOWER_LIMIT	(126.0)
#define BARRELGRAB_TILT_LOWER_LIMIT	(123.5)
#define CRANE_MAX_SPEED_UP		(30.0)
#define CRANE_MAX_SPEED_DOWN	(30.0)
#define CRANE_CLAW_LIMIT		(87.0)
#endif

#define DEGREES_PER_VOLT		(38.0) //152 degrees / 4 volts
#define OFFSET_DEGREES			(0.0)

#define CRANE_K_BANDWIDTH		(2.0)
#define CRANE_K_ROBOT			(38.0)
#define CRANE_TAU				(0.1)

#define HALF_ANGLE			(110.0) //95.0)	//45 degrees (pi/4 radians)


#define GET_BARREL_ANGLE	(125.4)
#define GET_BARREL_DELAY	((int)(0.5 * (N1SEC)))

#define GET_BARREL_TROLLEY_UP_CMD	(2.0) //(1.33) //(1.0)
#define GET_BARREL_TROLLEY_DOWN_CMD	(-0.75)
#define GET_BARREL_CRANE_UP_CMD		(1.0)
#define GET_BARREL_CRANE_DOWN_CMD	(-1.0)


class Crane
{
	private:
#ifdef PRACTICE_BOT
		Talon *winchMotor;
#else
		VictorSP *winchMotor;
#endif
		AnalogInput *armEncoder;

		typedef enum {lowerCrane, extendAll, pauseAllPressed, idlePressed, clawGrab, retractAll, pauseAllReleased, idleReleased} getBarrelStages;
		getBarrelStages getBarrelStage;

		bool inAutomation;

		int pauseCount;
		float lastPos;
		float errorInt;
		float lastCmd;
		float craneAutoGoal;

	public:
		Crane (void);
		float GetArmAngle();
		void ArmStop (void);
		void LocalReset (void);
		void JoystickMove(float power);
		void ControlledMove(float power);
		void GetBarrelAutomation(bool pickUpFlatButton, bool pickUpTiltButton);
		bool InAutomation(void);
		bool IsDoneExtending(void);
		bool IsDoneRetracting(void);
		void Service (void);
		void StopAll (void);
		void UpdateDash (void);
		void ControlSystemReset (void);
		float VelControl (float velCmd);
};

#endif /* SRC_CRANE_H_ */
