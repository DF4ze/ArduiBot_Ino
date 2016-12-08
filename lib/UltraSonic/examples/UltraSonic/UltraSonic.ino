#include <UltraSonic.h>



UltraSonic us ( 11, 12 );
long timeStamp = millis();
long lastTimeStamp = millis();

void setup() {

  Serial.begin(9600);
  Serial.println( "Ready!" );
}

void loop()
{
	
	
	timeStamp = millis();
	if( timeStamp - lastTimeStamp >= 1000 ){
		lastTimeStamp = timeStamp;
	
		Serial.print( "Distance : " );
		long time1 = millis();
		int dist = us.getDistance();
		long time = millis() - time1;
		Serial.println( dist );
		Serial.println( time );
		
		Serial.print( "AVG Distance : " );
		time1 = millis();
		dist = us.getAVGDistance();
		time = millis() - time1;
		Serial.println( dist );
		Serial.println( time );
		
		Serial.println( "----------------------" );
		

	}
}

