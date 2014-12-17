#include <Servo.h>


//////////////////////
// PINS
// Moteurs Gauche
#define MG_S1_PIN   	22	// Pin Moteurs Gauche Sens 1
#define MG_S2_PIN  		23 	// Pin Moteurs Gauche Sens 2
#define MG_V_PIN   		3 	// Pin Moteurs Gauche Vitesse
// Moteurs Droite
#define MD_S1_PIN   	24  // Pin Moteurs Droite Sens 1
#define MD_S2_PIN   	25 	// Pin Moteurs Droite Sens 2
#define MD_V_PIN   		4 	// Pin Moteurs Droite Vitesse

// Servos Tourelle
#define S_HORI_PIN		10 	// Pin
#define S_VERT_PIN		11 	// Pin

// Lumière(s)
#define LIGHT_SPOT_PIN	52 	// pin de la lumière pour la caméra
#define LIGHT_STROB_PIN	53 	// pin de la lumière pour le Stromboscope
#define LIGHT_LAZER_PIN	7 	// pin de la lumière pour le Lazer
//////////////////////

//////////////////
// MODE
#define MODE_MOTOR 		1
#define MODE_SERVO 		2
#define MODE_EXTRA 		3
#define MODE_STATE 		4

#define EXTRA_LIGHT 	1

#define LIGHT_SPOT  	1
#define LIGHT_LAZER 	2
#define LIGHT_STROB 	3
//////////////////////////

//////////////////////
// CONTANTES 
// PWM
#define MAX_PWM		200		
#define MIN_PWM		107
// Valeurs de buttés pour ne pas que les servos forcent
#define POS_MIN_HORI 		33
#define POS_MAX_HORI 		148
#define POS_MIN_VERT 		50
#define POS_MAX_VERT		147
///////////////////



	
	
	
/////////////////
// MOTEURS
boolean bInvertX = false;
boolean bInvertY = false;
////////////////

///////////////////////
// Port Série
String inputString = "";  		// Détient la chaine de caractère qui est réceptionnée
boolean stringComplete = false;	// Indicateur de fin de commande.
int maxParams = 5;				// nombre de parametres maximum qui peuvent etre receptionné dans une commande
String delim = ".";				// délimiteur de parametres

///////////////////////
// Servo Tourelle
Servo tourH;
Servo tourV;

////////////////
// Lights
int Lights [4] = {0,0,0,0};








boolean Debug = true;


void setup() {
	// Port Serie
	Serial.begin(/* 115200 */ 9600 );
	Serial.println( "Ouverture du port serie" );

	tourH.attach( S_HORI_PIN );  	// attache le servo horizontal au pin  
	tourV.attach( S_VERT_PIN );  	// attache le servo vertical au pin  
	Serial.println( "Declaration des Servos Tourelle" );

	pinMode( MG_S1_PIN, OUTPUT);
	pinMode( MG_S2_PIN, OUTPUT);
	pinMode( MG_V_PIN, OUTPUT);
	pinMode( MD_S1_PIN, OUTPUT);
	pinMode( MD_S2_PIN, OUTPUT);
	pinMode( MD_V_PIN, OUTPUT);
	Serial.println( "Declaration des Moteurs" );
	
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
		iVitesse = normalisePWM( iVitesse );
		iDelta = normalisePWM( iDelta );
				
		// Inversion si necessaire
		if( bInvertX )
			iDelta = -iDelta;
		if( bInvertY )
			iVitesse = -iVitesse;

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



int normalisePWM(int pwm){
	if( pwm > MAX_PWM )
		pwm = MAX_PWM;
	else if( pwm < -MAX_PWM )
		pwm = -MAX_PWM;

	return pwm;
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
	if( Debug ){
		Serial.print( "Vitesse : " );
		Serial.print( iVitesse );
		Serial.print( " Delta : " );
		Serial.println( iDelta );
	}
	int iVDroite = 0;		// Indicateur pour la borne de gauche
	int iVGauche = 0;		// Indicateur pour la borne de droite
	
	// Etalonnage des Bornes.
	iVGauche = iVitesse + iDelta;
	iVDroite = iVitesse - iDelta;
	iVGauche = normalisePWM( iVGauche );
	iVDroite = normalisePWM( iVDroite );
	
	// Affectation aux MOTEURS
	actionMD( iVDroite );
	actionMG( iVGauche );	
	
	if( Debug ){
		Serial.print( "Vitesse Gauche : " );
		Serial.print( iVGauche );
		Serial.print( " Vitesse Droite : " );
		Serial.println( iVDroite );
	}
}

void actionMG( int iVitesse ){
	if( iVitesse > 0 ){
		digitalWrite( MG_S1_PIN, MAX_PWM );
		digitalWrite( MG_S2_PIN, 0 );
		analogWrite( MG_V_PIN, iVitesse );
	}else{
		digitalWrite( MG_S1_PIN, 0 );
		digitalWrite( MG_S2_PIN, MAX_PWM );
		analogWrite( MG_V_PIN, -iVitesse );

	}
}
void actionMD( int iVitesse ){
	if( iVitesse > 0 ){
		digitalWrite( MD_S1_PIN, MAX_PWM );
		digitalWrite( MD_S2_PIN, 0 );
		analogWrite( MD_V_PIN, iVitesse );
	}else{
		digitalWrite( MD_S1_PIN, 0 );
		digitalWrite( MD_S2_PIN, MAX_PWM );
		analogWrite( MD_V_PIN, -iVitesse );
	}
}




void actionServos( int iPosH, int iPosV ){
	Serial.print( "Hori : " );
	Serial.print( iPosH );
	Serial.print( " Vert : " );
	Serial.println( iPosV );
	
	tourH.write( iPosH );
	tourV.write( iPosV );
}

void actionLights( int iLight, int iValue ){
	Serial.print( "Light : " );
	Serial.print( iLight );
	Serial.print( " iValue : " );
	Serial.println( iValue );
	
	switch( iLight ){
		case LIGHT_SPOT :
			digitalWrite( LIGHT_SPOT_PIN, (iValue == 0)?LOW:HIGH );
			break;
		case LIGHT_STROB :
			digitalWrite( LIGHT_STROB_PIN, (iValue == 0)?LOW:HIGH );
			break;
		case LIGHT_LAZER :
			digitalWrite( LIGHT_LAZER_PIN, (iValue == 0)?LOW:HIGH );
			break;
		default :
		;
	}
	Lights[iLight] = iValue;
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
	Serial.print( "[data][light]" );
	if( iLight == LIGHT_LAZER ){
		Serial.print( "[lazer]" );
	}else if( iLight == LIGHT_SPOT ){
		Serial.print( "[spot]" );
	}else if( iLight == LIGHT_STROB ){
		Serial.print( "[strob]" );
	}
	Serial.println( Lights[iLight] );

}













