#include <ReadSerial.h>

ReadSerial sr = ReadSerial();

void setup() {
  Serial.begin(9600);
 
}

void loop()
{
  // On check si on a fini la reception des données. (caractère d'échappement '*' par defaut)
  if ( sr.isStringComplete() ) {
	Serial.println( sr.getInputString() );
  }
}

void serialEvent() {
  while( Serial.available() ){
    sr.putAvailable( Serial.read() );
 }
}