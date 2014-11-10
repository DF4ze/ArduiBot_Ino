#include <ReadSerial.h>

ReadSerial sr = ReadSerial();

void setup() {
  Serial.begin(9600);
 
}

void loop()
{
  // On check si on a fini la reception des donn�es. (caract�re d'�chappement '*' par defaut)
  if ( sr.isStringComplete() ) {
	Serial.println( sr.getInputString() );
  }
}

void serialEvent() {
  while( Serial.available() ){
    sr.putAvailable( Serial.read() );
 }
}