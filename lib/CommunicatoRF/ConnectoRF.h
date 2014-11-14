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
#define POOLTIME 2000
// pour la connexion
#define DELAYREPEAT 200





class ConnectoRF{

  public:
    ConnectoRF();
	
    unsigned long getReceivedValue();
    void send(unsigned long Code );
	void send(char * Code );
	void resetAvailable();
	bool isAvailable();
	
	void setAsMaster();
	void setAsSlave();
	void setPins(int a, int b);
	int connectionMgr();
	// char* stringToCharArrayBin( String inputSerial );	
	
private:
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
	bool bAvailable;
	
	unsigned long aiMasterCmd[3];
	int aiMasterCmdCount;
	unsigned long aiSlaveCmd[3];
	int aiSlaveCmdCount;
	
	//Plage reservee aux commandes internes
	unsigned long MINRANGECMD ; // a partir de quel valeur il s'agit d'un msg perso et non une commande

	unsigned long HELLOIAMCTRL;
	unsigned long HELLOCTRL;

	unsigned long AREUOK;
	unsigned long IMOK;

	unsigned long STILLHERE;
	unsigned long IAMHERE;
	
	bool bDebug;
};
