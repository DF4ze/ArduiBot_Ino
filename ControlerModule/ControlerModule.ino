
#include <myRCSwitch.h>

#define PIN_RECEIVE 0
#define PIN_TRANSMIT 8

// max 4 294 967 295
// lo
#define HELLOIAMCTRL 	10
//lu
#define HELLOCTRL 		15

//ru
#define AREUOK 			20
//ok
#define IMOK 			25

// sh
#define STILLHERE		30
// ih
#define IAMHERE			35



#define UNSUCESSTRY 	5

myRCSwitch mySwitchRec = myRCSwitch();
myRCSwitch mySwitchTra = myRCSwitch();

bool bConnected = false;
bool bMayBeLost = false;

bool bHelloReceved = false;
bool bOkReceved = false;

int iUnsuccessTry = 0;


bool bDebug = false;

void setup() {
  Serial.begin(9600);
  if( bDebug ){
		Serial.println( "Initialisation du port série" );
		Serial.println( "enableReceive" );
  }else{
		Serial.println( "[Initialising]" );
  }
  mySwitchRec.enableReceive(PIN_RECEIVE);  // Receiver on inerrupt 0 => that is pin #2
   
  if( bDebug ){
	Serial.println( "enableTransmit" );
  }
  mySwitchTra.enableTransmit(PIN_TRANSMIT); // Transmitter is connected to Arduino Pin #8 
 
  if( !bDebug )	Serial.println( "[Searching]"  );
}

void loop() {

  ////////////////////////
  /// Partie réception
  unsigned long iReceived = 0;
  if (mySwitchRec.available()) {
    
    unsigned long value = mySwitchRec.getReceivedValue();
    
    if (value == 0) {
      Serial.print("[Unknown encoding]");
    } else {
      iReceived = mySwitchRec.getReceivedValue();
    }

    mySwitchRec.resetAvailable();
	
	
	if( iReceived == HELLOCTRL || iReceived == IMOK )
		iUnsuccessTry = 0;
  }
  
  ////////////////////////
  /// Partie envoi
	unsigned long iTransmit = 0;
	//////// Partie CONTROLER
	if( iReceived != HELLOIAMCTRL && iReceived != AREUOK && iReceived != STILLHERE ){
		if( !bConnected ){
			if( iReceived != HELLOCTRL && iReceived != IMOK ){
				if( iUnsuccessTry < UNSUCESSTRY ){
					// On prépare la var de transmit
					iTransmit= HELLOIAMCTRL;
					if( bDebug ){
						Serial.print( "Hello i'm controler : "  );
						Serial.print( iTransmit  );
					}
						
					// on l'envoie
					mySwitchTra.send(iTransmit, 32);
					
					// On comptabilise les essais
					iUnsuccessTry ++;
					
					if( bDebug ){
						Serial.print( " Try #"  );
						Serial.println( iUnsuccessTry  );
					}
				}else{
					if( bDebug ){
						Serial.println( "No Connection, Trying again... "  );
					}/* else{
						Serial.println( "[NoConnection]"  );
					} */
					iUnsuccessTry = 0;
				}
			}else if( iReceived == HELLOCTRL ){
				// On prépare la var de transmit
				iTransmit = AREUOK;
				if( bDebug ){
					Serial.print( "Are you ok? : "  );
					Serial.print( iTransmit  );
				}else
					Serial.println( "[ConnectionFinded]"  );
				// on l'envoie
				mySwitchTra.send(iTransmit, 32);
				
				// On comptabilise les essais
				iUnsuccessTry ++;
				
				if( bDebug ){
					Serial.print( " Try #"  );
					Serial.println( iUnsuccessTry  );
				}
				
			}else if( iReceived == IMOK ){
				bConnected = true;
				if( !bDebug ){
					Serial.println( "[Connected]" );
				}

			}
		}else{
			if( iReceived != IAMHERE ){
				if( iUnsuccessTry < UNSUCESSTRY ){
					// On prépare la var de transmit
					iTransmit = STILLHERE;
					if( bDebug ){
						Serial.print( "Are you still here? : "  );
						Serial.print( iTransmit  );
					}
					// on l'envoie
					mySwitchTra.send(iTransmit, 32);
					
					// On comptabilise les essais
					iUnsuccessTry ++;
					if( bDebug ){
						Serial.print( " Try #"  );
						Serial.println( iUnsuccessTry  );
					}
					
					if( !bDebug && iUnsuccessTry == 2 ){
						Serial.println( "[ConnectionMayBeLost]"  );
						bMayBeLost = true;
					}else if( bMayBeLost && iUnsuccessTry == 1){
						Serial.println( "[Connected]"  );
						bMayBeLost = false;
					}
				}else{
					if( bDebug ){
						Serial.println( "Connection lost, Trying again! "  );
					}else
						Serial.println( "[ConnectionLosted]"  );
					bMayBeLost = false;
					bConnected = false;
					iUnsuccessTry = 0;
				}
			}else{
				if( bDebug ){
					Serial.println( "OK It's here!"  );
				}
				iUnsuccessTry = 0;
			}
		}
		
		
	//////////// Partie RECEIVER
	}else{
		if( iReceived == HELLOIAMCTRL ){
			// On prépare la var de transmit
			iTransmit = HELLOCTRL;
			if( bDebug ){
				Serial.print( "*** Hello Mr Controler : "  );
				Serial.println( iTransmit  );
			}
		  
			// on l'envoie
			mySwitchTra.send(iTransmit, 32);
		
		}else if( iReceived == AREUOK ){
			// On prépare la var de transmit
			iTransmit = IMOK;
			if( bDebug ){
				Serial.print( "*** I'm OK Thanks =) : "  );
				Serial.println( iTransmit  );
			}
			// on l'envoie
			mySwitchTra.send(iTransmit, 32);
		
		}else if( iReceived == STILLHERE ){
			// On prépare la var de transmit
			iTransmit = IAMHERE;
			if( bDebug ){
				Serial.print( "*** Yes i'm here : "  );
				Serial.println( iTransmit  );
			}
		  
			// on l'envoie
			mySwitchTra.send(iTransmit, 32);
		
		}
	}
	delay(100);
}





