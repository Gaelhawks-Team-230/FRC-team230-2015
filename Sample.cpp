#include "WPILib.h"
#include "Common.h"
#include "Sample.h"


// CONSTRUCTOR for Sample Class
// - put a description of the mechanism for this class here :)
SampleClass::SampleClass()
{
	// Create objects needed by this class
	// example: sampleMotor = new VictorSP(PWM_SAMPLE_VICTOR);

	// call the class initialization function
	LocalReset();
}


// Set initial values for ALL objects and variables
void SampleClass::LocalReset()
{
	// Initialize ALL new objects
	// example: sampleMotor->Set(0.0);

	// Initialize ALL local variables
	// example: isEnabled = false;
}

//
void SampleClass::StopAll()
{
	// Stop all motors
	// example: sampleMotor->Set(0.0);

	// Reset all local flags that control Service operations
	// example: serviceMode = IDLE;
}

// Create functions for needed operations, as an example
//void SampleClass::ManualMove(float cmd)
//{
//    sampleMotor->Set(cmd);
//}

// Update the display of all relevant information in your class
//  - this can include motor/pwm command values, flags, state, etc.
void SampleClass::UpdateDash()
{
	//examples:
	//SmartDashboard::PutNumber("Sample PWM: ", sampleMotor->Get());
	//SmartDashboard::PutBoolean("Sample Enabled: ", isEnabled);
}

// All classes need a Service function, even if
void SampleClass::Service()
{
}




