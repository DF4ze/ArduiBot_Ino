#include <CommunicatoRF.h>


#define PIN_RECEIVE 0
#define PIN_TRANSMIT 8


bool bDebug = true;

int iLastStatus = 0;
unsigned long lastReceived = 0;
CommunicatoRF crfSlave = CommunicatoRF( );


void setup() {
  Serial.begin(9600);
  if( bDebug ){
		Serial.println( "Initialisation du port serie" );
		Serial.println( "enableReceive" );
  }else{
		Serial.println( "[Initialising]" );
  }
  
  crfSlave.setPins( PIN_RECEIVE, PIN_TRANSMIT );
  crfSlave.setAsSlave();
  
  
  if( bDebug )	Serial.println( "[Searching]"  );
}


void loop(){

	int iStatus = crfSlave.connectionMgr();

	if( iLastStatus != iStatus ){
		Serial.print("Status : ");
		Serial.println(iStatus);
		iLastStatus = iStatus;
	}
	
	if( crfSlave.isAvailable() ){
		unsigned long ulData;
		if( (ulData = crfSlave.getReceivedValue()) != 0 && ulData != lastReceived){
			crfSlave.resetAvailable(); /// ne marche pas...ou alors on recoit bel est bien plusieurs fois le meme message... ;)
			Serial.print("Data received");
			Serial.println(ulData);
			
			lastReceived = ulData;
		}
	}
}
