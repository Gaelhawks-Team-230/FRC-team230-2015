#ifndef SAMPLE_H_
#define SAMPLE_H_
/*
 * Sample.h
 *
 *  Created on: Jan 7, 2015
 *      Author: Gaelhawks
 */
class SampleClass
{
	private:
		// Create objects needed by this class
		// example: VictorSP *sampleMotor;

		// Declare local variables that need to be saved for future processing
		// example: bool isEnabled;
		// example: int serviceMode;

   	public:
		SampleClass(void);
		void LocalReset(void);
		void StopAll(void);
		// Define functions for needed operations, as an example
		// example: void ManualMove(float cmd);
		void UpdateDash(void);
		void Service(void);
};



#endif /* SAMPLE_H_ */
