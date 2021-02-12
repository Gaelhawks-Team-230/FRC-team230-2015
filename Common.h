/*
 * Common.h
 *
 *  Created on: Jan 10, 2015
 *      Author: Gaelhawks
 */

#ifndef COMMON_H_
#define COMMON_H_

/**********************\
 *  Feature Enablers  *
\**********************/
//#define	 CHECK_LOOPTIMING
#define USE_GYRO
//#define CALIBRATION
#define PRACTICE_BOT
//#define UNUSED_AUTOMODES
#define USE_CRANE
// turn off to calibrate Crane potentiometer
#define USE_CRANE_CONTROL_SYSTEM
#define USE_BARRELGRAB
// turn off to calibrate Trolley potentiometer
#define USE_TROLLEY_CONTROL_SYSTEM
//#define USE_TOO_CLOSE_SENSOR
//#define USE_USBCAMERA

 /********************\
  * Global Accessors *
 \********************/
#ifndef CALIBRATION
#define GLOBAL_DRIVE	        	(((TalonXVI&)(RobotBase::getInstance())).drive)
#define	GLOBAL_TOTEGRAB				(((TalonXVI&)(RobotBase::getInstance())).toteGrabber)
#define GLOBAL_CRANE				(((TalonXVI&)(RobotBase::getInstance())).crane)
#define GLOBAL_BARRELGRAB			(((TalonXVI&)(RobotBase::getInstance())).barrelGrabber)
#define GLOBAL_ELEVATOR				(((TalonXVI&)(RobotBase::getInstance())).elevator)
//MAX #define GLOBAL_NAV					(((TalonXVI&)(RobotBase::getInstance())).location)
#endif

/*******************\
 *  Global Values  *
\*******************/
#define PI	 						(3.14159265358979323846264338327950288)
#define RAD_TO_DEG 					(180.0/PI)
#define DEG_TO_RAD					(PI/180.0)
#define X_SCALING_FACTOR 			(1)
#define Y_SCALING_FACTOR			(0.65)

#define JOYSTICK_DEADBAND			(0.3)
#define DRIVE_DEADBAND				(0.1)

#define LOOPTIME 					(0.02)   // (0.005)   // (0.01)
#define SAMPLES_PER_SECOND			(1.0/LOOPTIME)
#define N1SEC  						((int) SAMPLES_PER_SECOND)


#define JOYSTICK_DEADBAND			(0.3)

typedef enum {
	USB_JOY_DRIVE	= 1,
	USB_GAMEPAD		= 2,
} usbs;

typedef enum {
	PWM_REAR_LEFT		= 0,
	PWM_FRONT_LEFT		= 1,
	PWM_REAR_RIGHT		= 2,
	PWM_FRONT_RIGHT 	= 3,
	PWM_LIFT			= 4, //Elevator
	PWM_CRANE			= 5,
	PWM_TROLLEY			= 6,
	PWM_TOTEGRAB	    = 7,
} pwms;

#define PWM_CALDEVICE		(PWM_CRANE)


typedef enum {
    GYRO_ANALOG_INPUT   = 0,
	ANALOG_TROLLEY		= 1,
	ANALOG_CRANE		= 2,
} analogs;

typedef enum {
	CLAW_PISTON1		= 0,
	CLAW_PISTON2		= 4,
	WRIST_PISTON1		= 1,
	WRIST_PISTON2		= 2,
	TOTE_PISTON1		= 3,
	TOTE_PISTON2		= 5,
}solenoids;

typedef enum {
	FRONT_LEFT_WHEEL_A		= 0,
	FRONT_LEFT_WHEEL_B		= 1,
	REAR_LEFT_WHEEL_A		= 2,
	REAR_LEFT_WHEEL_B		= 3,
	FRONT_RIGHT_WHEEL_A		= 4,
	FRONT_RIGHT_WHEEL_B		= 5,
	REAR_RIGHT_WHEEL_A		= 6,
	REAR_RIGHT_WHEEL_B		= 7,
	ELEVATOR_ENCODER_A		= 8,
	ELEVATOR_ENCODER_B 		= 9,
#ifdef USE_TOO_CLOSE_SENSOR
	DIGITAL_TOTE_SENSOR		= 13,	// sensor for tote at top of elevator
#endif
	DIGITAL_TOTEGRAB_SWITCH	= 14,	// limitswitch for tote in grabber (on floor)
	DIGITAL_AUTOMODE_ZERO	= 15,
	DIGITAL_AUTOMODE_ONE	= 16,
	DIGITAL_AUTOMODE_TWO	= 17,
	DIGITAL_AUTOMODE_THREE	= 18,  // which is labelled pwm0 on the MXP breakout
} digins;

////List of gamepad (USB_GAMEPAD above) button and axis assignments
// y-axis
#define DPAD_UP						(-1.0)
#define DPAD_DOWN					(1.0)
// x-axis
#define DPAD_LEFT					(-1.0)
#define DPAD_RIGHT					(1.0)

typedef enum
{
	THUMB_BUTTON			= 2,
	ENABLE_GYRO_BUTTON 		= 3,		//re-enable gyro function
	DISABLE_GYRO_BUTTON		= 4,		//allows the driver to turn of gyro
	DRIVER_ELEVATOR_DOWN	= 6,
	DRIVER_ELEVATOR_UP		= 5,
	DRIVER_GET_TOTE			= 7, //spin wheels in
	DRIVER_SQUEEZE_TOTE		= 8, //squeezes pistons
	DRIVER_STOP_MOTOR		= 9, //stop motor
	DRIVER_EJECT_TOTE		= 11, //run wheels backwards
	DRIVER_RELEASE_TOTE		= 12, //pistons open



} joystick_buttons;

typedef enum
{
	GAMEPAD_TROLLEY		= 1, // Left Joystick
	GAMEPAD_ELEVATOR	= 3, // Right Joystick
} gamepad_axes;

typedef enum
{
   DPAD_CRANE_UP  = 0,
   DPAD_CRANE_DOWN = 180,
   DPAD_WRIST_UP = 270,
   DPAD_WRIST_DOWN = 90,
} gamepad_pov;

typedef enum
{
	BUTTON_SQUEEZE_TOTE	= 1, // Button X
	BUTTON_GET_BARREL_FLAT	= 2, // Button A
	BUTTON_RELEASE_TOTE	= 3, // Button B
	BUTTON_GET_BARREL_TILT	= 4, // Button Y
	BUTTON_CLAW_RELEASE	= 5, // Left bumper
	BUTTON_EJECT_TOTE	= 6, // Right bumper
	BUTTON_CLAW_PINCH	= 7, // Left trigger
	BUTTON_GET_TOTE		= 8, // Right trigger
	STOP_TOTE_MOTOR		= 10,// Start Button
} gamepad_buttons;



#endif /* COMMON_H_ */
