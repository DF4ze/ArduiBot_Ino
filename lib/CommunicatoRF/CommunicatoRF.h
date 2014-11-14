#include "ConnectoRF.h"


#ifndef CommunicatoRF_h
#define CommunicatoRF_h
#endif

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"	
#else
    #include "WProgram.h"
#endif









class CommunicatoRF{

  public:
    CommunicatoRF();
	
	void setAsMaster();
	void setAsSlave();
	void setPins(int a, int b);
	int connectionMgr();
	
	// temp
	void send( char* caCode );
	bool isAvailable();
	unsigned long getReceivedValue();
	void resetAvailable();
	
private:
	ConnectoRF cRF;
	
	bool bDebug;
};
