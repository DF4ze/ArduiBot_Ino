#include <ReadSerial.h>

ReadSerial rs = ReadSerial();

void setup() {
  Serial.begin(9600);
 
}

void loop()
{
  // On check si on a fini la reception des donn�es. (caract�re d'�chappement '*' par defaut)
  if ( rs.isStringComplete() ) {
	Serial.println("isStringComplete");
	Serial.println( sr.getInputString() );
  }
}

void serialEvent() {
  while( Serial.available() ){
	// on ajoute au buffer
    if( rs.putAvailable( Serial.peek() ))
		// si aucun probleme on purge ce caract�re
		Serial.read();
 }
}