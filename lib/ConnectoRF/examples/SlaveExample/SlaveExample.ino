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
  crfMaster.setAsSlave();
  
  
  if( bDebug )	Serial.println( "[Searching]"  );
}


void loop(){

	int iStatus = crfMaster.connectionMgr();

	if( iLastStatus != iStatus ){
		Serial.print("Status : ");
		Serial.println(iStatus);
		iLastStatus = iStatus;
	}
	
}
