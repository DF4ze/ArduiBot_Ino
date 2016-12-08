
#ifndef UltraSonic_h
#define UltraSonic_h


#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"	
#else
    #include "WProgram.h"
#endif





class UltraSonic{

  public:
    UltraSonic( int pinTrig, int pinEcho );
    UltraSonic( int pinTrig, int pinEcho, int nbSamples );
    
	int getDistance();
	int getAVGDistance();
	int getAVGDistance( int nbSamples );
	
	void setTimeOut( int timeOut );
	void setTimeOutByDistance( int distance );

  private:	
	int _pinTrig;
	int _pinEcho;
	int _nbSamples;
	int _timeOut;
	
	void init();
};
#endif