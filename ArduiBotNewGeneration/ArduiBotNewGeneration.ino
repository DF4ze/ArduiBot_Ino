#include <Servo.h>


//////////////////////
// PINS
// Moteurs Gauche
#define MG_S1_PIN   	24//22	// Pin Moteurs Gauche Sens 1
#define MG_S2_PIN  		25//23 	// Pin Moteurs Gauche Sens 2
#define MG_V_PIN   		4//3 	// Pin Moteurs Gauche Vitesse
// Moteurs Droite
#define MD_S1_PIN   	22//24  // Pin Moteurs Droite Sens 1
#define MD_S2_PIN   	23//25 	// Pin Moteurs Droite Sens 2
#define MD_V_PIN   		3//4 	// Pin Moteurs Droite Vitesse

// Servos Tourelle
#define S_HORI_PIN		10 	// Pin
#define S_VERT_PIN		11 	// Pin

// Servo balai
#define S_BALAI_PIN		8 	// Pin

// Lumi�re(s)
#define LIGHT_SPOT_PIN	52 	// pin de la lumi�re pour la cam�ra
#define LIGHT_STROB_PIN	53 	// pin de la lumi�re pour le Stromboscope
#define LIGHT_LAZER_PIN	7 	// pin de la lumi�re pour le Lazer

// Ultrason
#define US_TRIG_PIN		12 	
#define US_ECHO_PIN		13 	
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
// Valeurs de butt�s pour ne pas que les servos forcent
#define POS_MIN_HORI 		33
#define POS_MAX_HORI 		148
#define POS_MIN_VERT 		50
#define POS_MAX_VERT		147
// Valeur d'angles pour le servo balai
#define BALAI_POS_MAX		120
#define BALAI_POS_MIN		60
#define BALAI_NBSAUT		3 // doit-etre > 1 ... et impair

///////////////////



	
	
	
/////////////////
// MOTEURS
boolean bInvertX = false;
boolean bInvertY = false;
////////////////

///////////////////////
// Port S�rie
String inputString = "";  		// D�tient la chaine de caract�re qui est r�ceptionn�e
boolean stringComplete = false;	// Indicateur de fin de commande.
int maxParams = 5;				// nombre de parametres maximum qui peuvent etre receptionn� dans une commande
String delim = ".";				// d�limiteur de parametres


///////////////////////
// Servo Tourelle
Servo tourH;
Servo tourV;

////////////////
// Lights
int Lights [4] = {0,0,0,0};

/////////////
// UltraSon
unsigned long stampTimeUS = millis();
unsigned long checkingTimeUS = 1000;
// Servo balai
Servo balai;
int posBalai = 90;
bool sensBalai = true; // true sens horaire, false : anti-horaire
int pasBalai = (BALAI_POS_MAX - BALAI_POS_MIN) / (BALAI_NBSAUT-1);




boolean Debug = false;


void setup() {
	// Port Serie
	Serial.begin( 57600/*115200  /*9600 */);
	Serial.println( "Ouverture du port serie" );

	tourH.attach( S_HORI_PIN );  	// attache le servo horizontal au pin  
	tourV.attach( S_VERT_PIN );  	// attache le servo vertical au pin  
	balai.attach( S_BALAI_PIN );  	// attache le servo balai au pin  
	Serial.println( "Declaration des Servos" );

	// Initialisation des pin pour les moteurs
	pinMode( MG_S1_PIN, OUTPUT);
	pinMode( MG_S2_PIN, OUTPUT);
	pinMode( MG_V_PIN, OUTPUT);
	pinMode( MD_S1_PIN, OUTPUT);
	pinMode( MD_S2_PIN, OUTPUT);
	pinMode( MD_V_PIN, OUTPUT);
	
	// Initialisation des pin pour les lumi�res
	pinMode( LIGHT_SPOT_PIN, OUTPUT);
	pinMode( LIGHT_LAZER_PIN, OUTPUT);
	pinMode( LIGHT_STROB_PIN, OUTPUT);
	
	  // Initialisation des pin pour le capteur UltraSon
	pinMode(US_TRIG_PIN, OUTPUT);
	pinMode(US_ECHO_PIN, INPUT);
  
	Serial.println( "Declaration des PINS" );
	
	Serial.println( "Ready !" );
	Serial.println( "" );
}


/**
 *
 * Boucle Initiale
 *
 */
void loop() {
	unsigned long runningTime; // temps depuis le dernier checkingTimeUS
	

 	if( stringComplete ){
		// Pour test : Permet de savoir vraiment ce qui est retenu
		if( Debug ){
			Serial.println();
			Serial.print("-* ");
			Serial.print(inputString);
			Serial.println(" *-");
 		}
		int iaParams[maxParams];
		int nbParams;
		if( parseParams( inputString, maxParams, iaParams, nbParams )){
			// Serial.print("IAPARAMS[0] : ");
			// Serial.println(iaParams[0]);
			// Serial.print("nbParams : ");
			// Serial.println(nbParams);
			if( !doAction(iaParams, nbParams) )
				if( Debug )
					Serial.println( "Erreur de Params" );
		}
		// On r�initialise la chaine:
		inputString = "";
		stringComplete = false;
	} 
	
	// demande de r�cup�ration de la distance si on a d�pass� le checkingTimeUS
	runningTime = millis() - stampTimeUS;
	if( runningTime > checkingTimeUS ){
		stampTimeUS = millis();
		// Serial.print("Distance : ");
		// Serial.print( getDistanceUS() );
		// Serial.println("cm");
		
		// On affiche la distance
		actionStateUltraSonic();
		// On demande au servo de bouger
		actionServosBalai();
	}
	
}


/**
 *
 * Boucle reception Serial
 *
 */
void serialEvent() {
	// Tant que nous avons des donn�es � r�ceptionner
	while( Serial.available() && stringComplete == false ){
		// R�cup�re un nouvel Octet:
		char inChar = Serial.read();
		
		// Si le caract�re r�ceptionn� est *
		// La boucle principale va pouvoir s'occuper du r�sultat.
		if( inChar == '*' ){
			stringComplete = true;
		}else
			// Sinon, on concat�ne l'octet � "inputString" ... si on n'est pas en mode stringComplete ... 
			// ca �vite l'empilage de parametres qd ils arrivent trop rapidement..
			if( !stringComplete )
				if(inChar >= 32) // Ne concat�ne que des caract�re imprimable.
					inputString += inChar;
	}

}



/**
 *
 * Gestion des parametres
 *
 */
boolean parseParams( String params, int maxParams, int iaParams[], int& nbParams ){
    // Indicateur de validit� des Params
    boolean ok = true;

	String saParams[ maxParams ];
	
	// On cherche les d�limiteurs
	nbParams = 0;
	int oldpos = 0;
	int newpos = params.indexOf( delim );
	do{
		saParams[nbParams] = params.substring( oldpos, newpos );
		oldpos = newpos + delim.length();
		newpos = params.indexOf( delim, oldpos );
		nbParams++;
	}while( newpos > -1 && nbParams < maxParams );
	// ne pas oublier le dernier param qui n'a pas de d�limiteur ;)
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
		
		// v�rification des limites
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
			
		actionServosTourelle( iPosH, iPosV );
		
	}else if( iaParams[0] == MODE_EXTRA && nbParams == 4){ 
		if( iaParams[1] == EXTRA_LIGHT ){
			actionLights( iaParams[2], iaParams[3] );
		}else
			ok = false;
		
	}else if( iaParams[0] == MODE_STATE && nbParams == 3){ 
		if( iaParams[1] == 0 && iaParams[2] == 0 )
			actionStateFull();
			
		else if( iaParams[1] == EXTRA_LIGHT ){
			actionStateLight( iaParams[2] );
		}else
			ok = false;
		
	}else 
		ok = false;
		
	return ok;
}


/**
 *
 * Fonction de v�rification des donn�es entr�es
 *
 */
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




/**
 *
 * Actions sur les moteurs
 *
 */
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



/**
 *
 * Actions sur les servo
 *
 */
void actionServosTourelle( int iPosH, int iPosV ){
	if( Debug ){
		Serial.print( "Hori : " );
		Serial.print( iPosH );
		Serial.print( " Vert : " );
		Serial.println( iPosV );
	}
	iPosH = 180 - iPosH;
	iPosV = 180 - iPosV;
	
	tourH.write( iPosH );
	tourV.write( iPosV );
}

void actionServosBalai( ){
	int tempPas = ((sensBalai)?1:-1) * pasBalai;
	
	// on v�rifie si le pas de plus dans le meme sens ne nous fait pas sortir des bornes
	if( posBalai + tempPas < BALAI_POS_MIN || posBalai + tempPas > BALAI_POS_MAX ){
		// si c le cas, on inverse
		sensBalai = !sensBalai;
		tempPas = ((sensBalai)?1:-1) * pasBalai;
		
		// si on d�borde qd meme : pas = 0
		tempPas = ( posBalai + tempPas < BALAI_POS_MIN || posBalai + tempPas > BALAI_POS_MAX )?0:tempPas;
	}
	
	// on demande au servo de se mettre en position
	posBalai += tempPas;
	balai.write( 180 - posBalai );
	
	// si on est sur la valeur exacte, on change de sens
}

/**
 *
 * Actions sur les lumi�res
 *
 */
void actionLights( int iLight, int iValue ){
	if( Debug ){
		Serial.print( "Light : " );
		Serial.print( iLight );
		Serial.print( " iValue : " );
		Serial.println( iValue );
	}
	switch( iLight ){
		case LIGHT_SPOT :
			digitalWrite( LIGHT_SPOT_PIN, (iValue == 0)?LOW:HIGH );
			break;
		case LIGHT_STROB :
			digitalWrite( LIGHT_STROB_PIN, (iValue == 0)?LOW:HIGH );
			break;
		case LIGHT_LAZER :
			digitalWrite( LIGHT_LAZER_PIN, (iValue == 0)?0:127 );
			break;
		default :
		;
	}
	Lights[iLight] = iValue;
}


/**
 *
 * Capteur Ultrason
 *
 */
int getDistanceUS(){
	int duration, distance;
	
	// Initialisation du capteur
	digitalWrite(US_TRIG_PIN, HIGH);
	delayMicroseconds(1000);
	digitalWrite(US_TRIG_PIN, LOW);
	// Mesure du temps : all�/retour de l'ultrason
	duration = pulseIn(US_ECHO_PIN, HIGH, 12000); // l'ultrason ne peut mesur� pr�cis�ment que jusqu'a 4-5m, je mets un timeout pour 10m
	// conversion en distance.
	distance = (duration/2) * 0.03495;
	
	// s'il y a timeout, duration va etre = 0 donc on met la distance a 10m
	if( duration == 0 )
		distance = 1000;
	
	return distance;
}







/**
 *
 * Retour d'�tats
 *
 */
	/**
	 * serial doit etre de la forme :
	 * [data]	[sensor]	[distance]		int(pos).int(distance)
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
void actionStateFull(){
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

void actionStateUltraSonic( ){
	Serial.print( "[data][sensor][distance]" );
	Serial.print( posBalai );
	Serial.print( "." );
	Serial.println( getDistanceUS() );
}













