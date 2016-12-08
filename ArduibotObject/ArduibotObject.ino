#include <ReadSerial.h>

ReadSerial rs = ReadSerial();


void setup() {
	rs.setDelimCmd( '*' );
	rs.setDelimParams( ',' );

	Serial.begin(9600);
	Serial.println( "Ready!" );
}

void loop() {
  // put your main code here, to run repeatedly:

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
