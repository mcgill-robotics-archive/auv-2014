#ifndef DepthNode_h
#define DepthNode_h

// Filter Constants
double MESUREMENT_VARIANCE = 1.0;
double PROCESS_VARIANCE = 1e-5;

// Sensor-related Constants
int MAX_ANALOG = 1023  			// in bits
int MAX_ARDUINO_VOLT = 5000 	// in mV
double RESISTANCE = 100.0 		// In ohms. Set to the value of the resistor being used. Should be less than 250 ohms because 5V / 20mA (max current) = 250 omhs.
double BASE_CURRENT = 4.0
double CURRENT_RANGE = 16.0   	// from 4.0 mA to 20.0 mA
double MAX_DEPTH = 914.4      	// In centimeters. 30 feet
double OFFSET = 17.0          	// Just substract it. May need to be recalculated when circuit is built.

// Essentially a one-dimensinal kalman filter
class DepthNode
{
	public:
		DepthNode();
		double predict();
		void correct(double measurement);
	private:
		double kalmanGain, prioriState, posterioriState, errVariance;
};

#endif