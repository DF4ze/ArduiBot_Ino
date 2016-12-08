
#ifndef Voltage_h
#define Voltage_h


#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"	
#else
    #include "WProgram.h"
#endif





class Voltage{

  public:
    Voltage( int pin );
	
	bool check();
	float getVoltage();

  private:	
	int _pin;
	int voltageSamples;  				// Nombre de samples
	int voltageSamplingNb; 				// incremential number
	unsigned long voltageValuesSum;		// Somme des valeurs
	float voltage;						// Voltage moyen
	
	void init();
};
#endif