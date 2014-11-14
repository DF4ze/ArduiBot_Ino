
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
	bool putAvailable( char aChar );
	bool isStringComplete( );
	String getInputString();
	void setBuffSize( int iSize );
	
private:	
	String pullString();

	String inputString;
	String buffStrings[10];
	int buffSize;
	bool stringComplete;
	char cDelim;
	int iWriteIndex;
	
	bool bDebug;
};
