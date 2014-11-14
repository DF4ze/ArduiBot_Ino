
#include <CommunicatoRF.h>
#include <ReadSerial.h>



#define PIN_RECEIVE 0
#define PIN_TRANSMIT 8


bool bDebug = true;
int iLastStatus = 0;

CommunicatoRF crfMaster = CommunicatoRF( );
ReadSerial rs = ReadSerial();

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

	int iStatus = crfMaster.connectionMgr();

	if( iLastStatus != iStatus ){
		Serial.print("Status : ");
		Serial.println(iStatus);
		iLastStatus = iStatus;
	}
	

	if( iStatus >= 2 && iStatus <= 4 ){
		if ( rs.isStringComplete() ) {
			// on recupere la string du Serial
			String inputSerial = rs.getInputString();
 			Serial.print("-- will Send : ");
			Serial.println(inputSerial);

			// on la transforme en char*
			char caInput[ inputSerial.length()+1 ];
			inputSerial.toCharArray(caInput, inputSerial.length()+1) ;
			
			// on la transforme en UL
			unsigned long uli = strtoul( caInput, NULL, 10);

			// on la transforme en binaire... String
			String sCode = String(uli,BIN);
	 
			// on transforme en char*
			char caCode[ sCode.length()+1 ];
			sCode.toCharArray(caCode, sCode.length()+1) ;
			  
			// on envoi...!
			crfMaster.send( caCode );
		}
		
	}

}


void serialEvent() {
  while( Serial.available() ){
	// on ajoute au buffer
    if( rs.putAvailable( Serial.peek() ))
		// si aucun probleme on purge ce caractère
		Serial.read();
 }
}


// 05 34 46 45 62
// 09 50 98 42 01
// 0892135151
// 3244