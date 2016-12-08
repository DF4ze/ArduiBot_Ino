#include <L293D.h>


L293D motors( 34, 32, 4, 30, 28, 3, true, false );

void setup() {

  Serial.begin(9600);
  Serial.println( "Ready!" );
}

void loop()
{
	Serial.println( "Avance" );
	motors.setActionMotors( 100, 0 );
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );
	
	Serial.println( "Recule" );
	motors.setActionMotors( -100, 0 );
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );
	
	Serial.println( "Droite" );
	motors.setActionMotors( 0, 100 );
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );
	
	Serial.println( "Gauche" );
	motors.setActionMotors( 0, -100 );
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );
	
	Serial.println( "Over ++" );
	motors.setActionMotors( 500, 0 );
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );
	
	Serial.println( "Over --" );
	motors.setActionMotors( -500, 0 );
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );

	Serial.println( "Over Droite" );
	motors.setActionMotors( 0, 1000);
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );

	Serial.println( "Over Gauche" );
	motors.setActionMotors( 0, -1000);
	delay( 2000 );
	motors.stopMotors( );
	delay( 500 );

}

