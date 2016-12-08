
#ifndef ReadSerial_h
#define ReadSerial_h


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
	
	void setDelimCmd( char aChar );
	void setDelimParams( char aChar );
	bool putAvailable( char aChar );
	bool isStringComplete( );
	String getInputString();
	bool parseStringParams( String params, String saParams[], int& nbParams );
	bool parseFloatParams( String params, float faParams[], int& nbParams );
	bool parseIntParams( String params, int iaParams[], int& nbParams );
	int getMaxParams();
	
private:	
	String pullString();

	String inputString;
	String buffStrings[10];
	int buffSize;
	bool stringComplete;
	char cDelimCmd;
	char cDelimParams;
	int iWriteIndex;
	int maxParams;
	
	bool bDebug;
};
#endif