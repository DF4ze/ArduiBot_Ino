#include <Voltage.h>


Voltage v(0);

void setup() {

  Serial.begin(9600);
  Serial.println( "Ready!" );
}

void loop()
{
	if( v.check() ){
		Serial.print( "Voltage : " );
		Serial.println( v.getVoltage() );
		Serial.println( "----------------------" );
	}
}

