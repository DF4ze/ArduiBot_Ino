
const int MAX_PWM = 255;

const int MODE_MOTOR = 1;
const int MODE_SERVO = 2;
const int MODE_EXTRA = 3;
const int MODE_STATE = 4;

const int EXTRA_LIGHT = 1;

const int LIGHT_SPOT 	= 1;
const int LIGHT_LAZER 	= 2;
const int LIGHT_STROB 	= 3;

const int POS_MAX_HORI = 148;
const int POS_MAX_VERT = 147;
const int POS_MIN_HORI = 33;
const int POS_MIN_VERT = 50;

	
boolean bInvertX = false;
boolean bInvertY = false;

///////////////////////
// Port Série
String inputString = "";  		// Détient la chaine de caractère qui est réceptionnée
boolean stringComplete = false;	// Indicateur de fin de commande.
int maxParams = 5;				// nombre de parametres maximum qui peuvent etre receptionné dans une commande
String delim = ".";				// délimiteur de parametres



void setup() {
	// Port Serie
	Serial.begin(/* 115200 */ 9600 );
	Serial.println( "Ouverture du port serie" );

	Serial.println( "Ready !" );
	Serial.println( "" );
}

void loop() {
 	if( stringComplete ){
		// Pour test : Permet de savoir vraiment ce qui est retenu
 		Serial.println();
		Serial.print("-* ");
		Serial.print(inputString);
		Serial.println(" *-");
 		
		int iaParams[maxParams];
		int nbParams;
		if( parseParams( inputString, maxParams, iaParams, nbParams )){
			// Serial.print("IAPARAMS[0] : ");
			// Serial.println(iaParams[0]);
			// Serial.print("nbParams : ");
			// Serial.println(nbParams);
			if( !doAction(iaParams, nbParams) )
				Serial.println( "Erreur de Params" );
		}
		// On réinitialise la chaine:
		inputString = "";
		stringComplete = false;
	} 
}



void serialEvent() {
	// Tant que nous avons des données à réceptionner
	while( Serial.available() && stringComplete == false ){
		// Récupère un nouvel Octet:
		char inChar = Serial.read();
		
		// Si le caractère réceptionné est *
		// La boucle principale va pouvoir s'occuper du résultat.
		if( inChar == '*' ){
			stringComplete = true;
		}else
			// Sinon, on concatène l'octet à "inputString" ... si on n'est pas en mode stringComplete ... 
			// ca évite l'empilage de parametres qd ils arrivent trop rapidement..
			if( !stringComplete )
				if(inChar >= 32) // Ne concatène que des caractère imprimable.
					inputString += inChar;
	}

}




boolean parseParams( String params, int maxParams, int iaParams[], int& nbParams ){
    // Indicateur de validité des Params
    boolean ok = true;

	String saParams[ maxParams ];
	
	// On cherche les délimiteurs
	nbParams = 0;
	int oldpos = 0;
	int newpos = params.indexOf( delim );
	do{
		saParams[nbParams] = params.substring( oldpos, newpos );
		oldpos = newpos + delim.length();
		newpos = params.indexOf( delim, oldpos );
		nbParams++;
	}while( newpos > -1 && nbParams < maxParams );
	// ne pas oublier le dernier param qui n'a pas de délimiteur ;)
	saParams[nbParams] = params.substring( oldpos, params.length() );
	nbParams++;
	
	// transfert dans un tab de int
	for( int i=0; i < nbParams; i++ ){
		iaParams[i] = saParams[i].toInt();
	}

	return ok;
}

boolean doAction( int iaParams[], int nbParams ){
	boolean ok = true;
	if( iaParams[0] == MODE_MOTOR && nbParams == 3  ){
		int iVitesse = iaParams[1];
		int iDelta = iaParams[2];
		
		// vérification des limites
		iVitesse = normaliseSpeed( iVitesse );
		iDelta = normaliseDelta( iDelta );
				
		// faire l'action sur les moteurs
		actionMotors( iVitesse, iDelta );
		
	}else if( iaParams[0] == MODE_SERVO && nbParams == 3){
		int iPosH = iaParams[1];
		int iPosV = iaParams[2];
		
		iPosH = normaliseHori( iPosH );
		iPosV = normaliseVert( iPosV );
			
		actionServos( iPosH, iPosV );
		
	}else if( iaParams[0] == MODE_EXTRA && nbParams == 4){ 
		if( iaParams[1] == EXTRA_LIGHT ){
			actionLights( iaParams[2], iaParams[3] );
		}else
			ok = false;
		
	}else if( iaParams[0] == MODE_STATE && nbParams == 3){ 
		if( iaParams[1] == 0 && iaParams[2] == 0 )
			actionFullState();
			
		else if( iaParams[1] == EXTRA_LIGHT ){
			actionStateLight( iaParams[2] );
		}else
			ok = false;
		
	}else 
		ok = false;
		
	return ok;
}


int normaliseSpeed(int speed){
	if( speed > MAX_PWM )
		speed = MAX_PWM;
	else if( speed < -MAX_PWM )
		speed = -MAX_PWM;
		
	if( bInvertY )
		speed = -speed;
	return speed;
}
int normaliseDelta(int delta){
	if( delta > MAX_PWM*2 )
		delta = MAX_PWM*2;
	else if( delta < -MAX_PWM*2 )
		delta = -MAX_PWM*2;
		
	if( bInvertX )
		delta = -delta;
	return delta;
}

int normaliseHori(int iPosH){
	if( iPosH < POS_MIN_HORI )
		iPosH = POS_MIN_HORI;
	else if ( iPosH > POS_MAX_HORI )
		iPosH = POS_MAX_HORI;

	return iPosH;
}
int normaliseVert(int iPosV){
	if( iPosV < POS_MIN_VERT )
		iPosV = POS_MIN_VERT;
	else if( iPosV > POS_MAX_VERT )
		iPosV = POS_MAX_VERT;

	return iPosV;
}





void actionMotors( int iVitesse, int iDelta ){
	Serial.print( "Vitesse : " );
	Serial.println( iVitesse );
	Serial.print( "Delta : " );
	Serial.println( iDelta );
}

void actionServos( int iPosH, int iPosV ){
	Serial.print( "Hori : " );
	Serial.println( iPosH );
	Serial.print( "Vert : " );
	Serial.println( iPosV );
}

void actionLights( int iLight, int iValue ){
	Serial.print( "Light : " );
	Serial.println( iLight );
	Serial.print( "iValue : " );
	Serial.println( iValue );
	
}
	/**
	 * Sserial doit etre de la forme :
	 * [data]	[sensor]	[distance]		int
	 * 						[lumiere]		int
	 * 						[temperature]	int
	 * 
	 * 			[light]		[spot]		boolean
	 * 						[lazer]		boolean
	 * 						[strob]		boolean
	 * 			
	 * @param sSerial
	 * @return
	 */
void actionFullState(){
	actionStateLight( LIGHT_LAZER );
	actionStateLight( LIGHT_SPOT );
	actionStateLight( LIGHT_STROB );
}

void actionStateLight( int iLight ){
	if( iLight == LIGHT_LAZER ){
		Serial.print( "[data][light][lazer]" );
		Serial.println( "1" );
	}else if( iLight == LIGHT_SPOT ){
		Serial.print( "[data][light][spot]" );
		Serial.println( "1" );
	}else if( iLight == LIGHT_STROB ){
		Serial.print( "[data][light][strob]" );
		Serial.println( "1" );
	}
}









