
#ifndef WheelSpeed_h
#define WheelSpeed_h


#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"	
#else
    #include "WProgram.h"
#endif





class WheelSpeed{

  public:
    WheelSpeed( int pin, int nbHoles );
    WheelSpeed( int pin, int nbHoles, long period );
    WheelSpeed( int pin, int nbHoles, long period, int diam );
	WheelSpeed( int pin, int nbHoles, int diam );
	
	void check();
	double getTbyS();
	double getDistanceCm();
	double getSpeedKMH();
	
	
private:	
	void init();
	double calcTbyS();

	int _pin;
	int _nbHoles;
	int _diam;
	double _peri;
	long _count;
	long _countDist;
	long _timeStamp;
	long _lastTimeStamp;
	bool _isLow;
	long _period;
	double _TbyS;
	
	bool bDebug;
};
#endif