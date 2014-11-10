
#ifndef ReadSerial_h
#define ReadSerial_h
#endif

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"	
#else
    #include "WProgram.h"
#endif





class ReadSerial{

  public:
    ReadSerial();
	
	void setDelim( char aChar );
	void putAvailable( char aChar );
	bool isStringComplete( );
	String getInputString();
	
private:	
	String inputString;
	bool stringComplete;
	char cDelim;
	
	bool bDebug;
};
