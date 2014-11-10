#include <ConnectoRF.h>


#define PIN_RECEIVE 0
#define PIN_TRANSMIT 8


bool bDebug = true;
int iLastStatus = 0;
ConnectoRF crfMaster = ConnectoRF( );


void setup() {
  Serial.begin(9600);
  if( bDebug ){
		Serial.println( "Initialisation du port serie" );
		Serial.println( "enableReceive" );
  }else{
		Serial.println( "[Initialising]" );
  }
  
  crfMaster.setPins( PIN_RECEIVE, PIN_TRANSMIT );
  crfMaster.setAsMaster();
  
  
  if( bDebug )	Serial.println( "[Searching]"  );
}


void loop(){

	unsigned long time1 = millis();
	int iStatus = crfMaster.connectionMgr();
	unsigned long time2 = millis();

	if( iLastStatus != iStatus ){
		Serial.print("Status : ");
		Serial.println(iStatus);
		Serial.print("time sending : ");
		Serial.println(time2-time1);
		iLastStatus = iStatus;
	}
	
}