#include <ReadSerial.h>

ReadSerial rs = ReadSerial();


void setup() {
  rs.setDelimCmd( '*' );
  rs.setDelimParams( ',' );

  Serial.begin(9600);
  Serial.println( "Ready!" );
}

void loop()
{
  // On check si on a fini la reception des données. (caractère d'échappement '*' par defaut)
  if ( rs.isStringComplete() ) {
	Serial.println("String is Completed : ");
	while( rs.isStringComplete() ){
		String data = rs.getInputString();
		Serial.println( "- "+data );
		
		const int maxParams = rs.getMaxParams();
		float faParams[ maxParams ];
		int nbParams;
		if( rs.parseParams( data, faParams, nbParams ) ){
			Serial.println( "\tParsing successful" );
			for( int i = 0; i < nbParams; i++ ){
				Serial.print( "\t- " );
				Serial.println( faParams[i] );
			}
		}else{
			Serial.println( "\tParsing with error(s)" );
			if( nbParams > 0 ){
				Serial.println( "\tBut Params found : " );
				for( int i = 0; i < nbParams; i++ ){
					Serial.print( "\t- " );
					Serial.println( faParams[i] );
				}
			}
		}
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