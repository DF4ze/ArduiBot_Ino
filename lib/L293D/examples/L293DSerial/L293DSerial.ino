#include <L293D.h>
#include <ReadSerial.h>

ReadSerial rs = ReadSerial();

L293D motors( 34, 32, 4, 30, 28, 3, false, false );

void setup() {
  rs.setDelimCmd( '*' );
  rs.setDelimParams( ',' );

  Serial.begin(9600);
  Serial.println( "Ready!" );
}

void loop()
{
  if ( rs.isStringComplete() ) {
	while( rs.isStringComplete() ){
		String data = rs.getInputString();
		const int maxParams = rs.getMaxParams();
		int iaParams[ maxParams ];
		int nbParams;
		if( rs.parseIntParams( data, iaParams, nbParams ) ){
			if( nbParams == 2 ){
				motors.setActionMotors( iaParams[0], iaParams[1] );
			}else
				Serial.println("error Param Number...");
		}else
			Serial.println("error Parsing...");
	}	  
  }	
	
	
	
}

void serialEvent() {
  // s'il y a des données réceptionnées
  while( Serial.available() ){
	// on ajoute au buffer
    if( rs.putAvailable( Serial.peek() ))
		// si aucun probleme on purge ce caractère
		Serial.read();
 }
}