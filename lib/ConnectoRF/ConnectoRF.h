#include "myRCSwitch.h"


#ifndef ConnectoRF_h
#define ConnectoRF_h
#endif

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"	
#else
    #include "WProgram.h"
#endif



#define MAXUNSUCESSTRY 	5

////// Status
#define SEARCHING 	1
#define FINDED	 	2
#define CONNECTED 	3
#define MAYLOSTED 	4
#define LOSTED 		5

////// Vitesse de pooling en ms 
// pour le maintient en vie
#define POOLTIME 1000
// pour la connexion
#define DELAYREPEAT 200





class ConnectoRF{

  public:
    ConnectoRF();
	
    unsigned long getReceivedValue();
    void send(unsigned long Code );
	
	void setAsMaster();
	void setAsSlave();
	void setPins(int a, int b);
	int connectionMgr();
	
	
private:
	void init();
	unsigned long getAvailableReceivedControl();
	int masterConnection();
	int slaveConnection();
	bool isSlaveCmd( unsigned long ulMessage );
	bool isMasterCmd( unsigned long ulMessage );

	
	myRCSwitch mySwitchRec;
	myRCSwitch mySwitchTra;

	int iStatus; 
	int iUnsuccessTry;
	unsigned long ulTimePool;
	unsigned long ulTimeMaxSpeed;
	bool bMaster;
	
	unsigned long aiMasterCmd[3];
	unsigned long aiSlaveCmd[3];
	
	bool bDebug;
};
