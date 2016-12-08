#include <WheelSpeed.h>


WheelSpeed ws(22, 20, 66);

long timeStamp = millis();
long lastTimeStamp = millis();

void setup() {

  Serial.begin(9600);
  Serial.println( "Ready!" );
}

void loop()
{
	ws.check();
	
	
	timeStamp = millis();
	if( timeStamp - lastTimeStamp >= 1000 ){
		lastTimeStamp = timeStamp;
	
		Serial.print( "T/S : " );
		Serial.println( ws.getTbyS() );
		
		Serial.print( "Distance : " );
		Serial.println( ws.getDistanceCm() );
		
		Serial.print( "Speed : " );
		Serial.println( ws.getSpeedKMH() );
		Serial.println( "----------------------" );
		

	}
}

