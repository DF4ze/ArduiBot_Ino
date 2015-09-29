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

// Lumière(s)
#define LIGHT_SPOT_PIN	52 	// pin de la lumière pour la caméra
#define LIGHT_STROB_PIN	53 	// pin de la lumière pour le Stromboscope
#define LIGHT_LAZER_PIN	7 	// pin de la lumière pour le Lazer

// Ultrason
#define US_TRIG_PIN		12 	
#define US_ECHO_PIN		13 	

// Capteur Voltage
#define VOL_SENOR A0

//////////////////////

//////////////////
// MODE
// 1ere cat
#define MODE_MOTOR 		1
#define MODE_SERVO 		2
#define MODE_EXTRA 		3
#define MODE_STATE 		4

// 2nd cat
#define EXTRA_LIGHT 	1
#define EXTRA_BALAI 	2

#define LIGHT_SPOT  	1
#define LIGHT_LAZER 	2
#define LIGHT_STROB 	3

#define BALAI_MODE_AVANCE 	1
#define BALAI_MODE_ARRET 	2
#define BALAI_MODE_GCH 		3
#define BALAI_MODE_DRT 		4

//////////////////////////

//////////////////////
// CONTANTES 
// PWM pour les moteurs ... normalement 6v mais on les fait monter jusqu'a 9v maxi ;)
#define MAX_PWM		200		
#define MIN_PWM		107
// Valeurs de buttés pour ne pas que les servos forcent
#define POS_MIN_HORI 		33
#define POS_MAX_HORI 		148
#define POS_MIN_VERT 		50
#define POS_MAX_VERT		147
// Capteur Voltage
#define GAIN 50/9.0	//VOL = SIG*GAIN
#define ADC_REF 5  	//reference voltage of ADC is 5v
					//Warning:
					//     If the reference voltage of ADC is 5V, the VOL should be less than 25V;
					//     If the reference voltage of ADC is 3.3V, the VOL should be less than 18V;
					//     If the reference voltage of ADC is 2.56V, the VOL should be less than 14V;
					//     If the reference voltage of ADC is 1.1V, the VOL should be less than 6V;

///////////////////



	
	
	
/////////////////
// MOTEURS
boolean bInvertX = true;   // inverse la direction
boolean bInvertY = false;  // inverse la vitesse
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

/////////////
// UltraSon
unsigned long stampTimeUS = millis();
unsigned long checkingTimeUS = 200; 	// temps entre 2 relevés US // min 40ms
unsigned long stampTimeScan = millis();
unsigned long checkingTimeScan = 1000; // temps entre 2 scans complet // sera modifier par le mode
// Valeur d'angles pour le servo balai
int posMaxBalai = 120;
int posMinBalai = 60;
int nbSautBalai = 3; 					// doit-etre > 1 ... et impair
// Servo balai
Servo balai;
int posBalai = 90;
bool sensBalai = true; 					// true sens horaire, false : anti-horaire
int pasBalai = 0;
bool isBalaiMoving = false; 			// des/active le balayage 
bool isBalaiScanComplet = false; 		// indique si un scan complet du cone est fini.
int balaiRunningMode = BALAI_MODE_ARRET;

/////////////
// Capteur Voltage
int voltageSamples = 10000;  			// Nombre de samples
int voltageSamplingNb = 0; 				// incremential number
unsigned long voltageValuesSum = 0;		// Somme des valeurs
float voltage;							// Voltage moyen





boolean isDebug = true;


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
	
	// Initialisation des pin pour les lumières
	pinMode( LIGHT_SPOT_PIN, OUTPUT);
	pinMode( LIGHT_LAZER_PIN, OUTPUT);
	pinMode( LIGHT_STROB_PIN, OUTPUT);
	
	  // Initialisation des pin pour le capteur UltraSon
	pinMode(US_TRIG_PIN, OUTPUT);
	pinMode(US_ECHO_PIN, INPUT);
	modeServoBalai( BALAI_MODE_ARRET );
  
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
	

 	if( stringComplete ){
		// Pour test : Permet de savoir vraiment ce qui est retenu
		if( isDebug ){
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
				if( isDebug )
					Serial.println( "Erreur de Params" );
		}
		// On réinitialise la chaine:
		inputString = "";
		stringComplete = false;
	} 
	
	// demande de récupération de la distance si on a dépassé le checkingTimeUS
	actionUS();

	// traitement du voltage de la batterie
	actionVoltage();
}


/**
 *
 * Boucle reception Serial
 *
 */
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



/**
 *
 * Gestion des parametres
 *
 */
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

		// gestion du balayage en fonction de la direction
		if( iDelta == 0 ){
			if( iVitesse == 0 )
				modeServoBalai( BALAI_MODE_ARRET );
			else
				modeServoBalai( BALAI_MODE_AVANCE );
		}else if( iDelta > 0 ){
			modeServoBalai( BALAI_MODE_DRT );
		}else 
			modeServoBalai( BALAI_MODE_GCH );
			
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
		}if( iaParams[1] == EXTRA_BALAI ){
			;
			//actionBalai( iaParams[2], iaParams[3] );
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
 * Fonction de vérification des données entrées
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
	if( isDebug ){
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
	
	if( isDebug ){
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
	if( isDebug ){
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
	
	// on vérifie si le pas de plus dans le meme sens ne nous fait pas sortir des bornes
	if( posBalai + tempPas < posMinBalai || posBalai + tempPas > posMaxBalai ){
		// si c le cas, on inverse
		sensBalai = !sensBalai;
		tempPas = ((sensBalai)?1:-1) * pasBalai;
		
		// si on déborde qd meme : pas = 0
		//tempPas = ( posBalai + tempPas < posMinBalai || posBalai + tempPas > posMaxBalai )?0:tempPas;
		
		// si on est qd meme en dehors, peut-etre que l'on vient de changer de mode et que le servo est de l'autre coté...
		// si on est en dessous de la position Mini on se remet sur la posMinBalai		
		if( posBalai + tempPas < posMinBalai  )
			tempPas = posMinBalai - posBalai;
		// et inversement pour posMaxBalai
		else if( posBalai + tempPas > posMaxBalai )
			tempPas = posMaxBalai - posBalai;
		
		// sinon si on fait demi-tour c'est que l'on a fini de scaner le cone.
		else
			isBalaiScanComplet = true;
	}
	
	// on demande au servo de se mettre en position
	posBalai += tempPas;
	balai.write( 180 - posBalai );
}

void prepareServoBalai( int posMin, int posMax, int pas, unsigned long freq ){
	int delta;
	unsigned long delayPos;
	
	posMaxBalai = posMax;
	posMinBalai = posMin;
	//nbSautBalai = nbPas;
	checkingTimeScan = freq;
	
	pasBalai = pas;//(posMaxBalai - posMinBalai) / (nbSautBalai-1);
	
	// on positionne le balai en début de cone
	if( abs(posBalai - posMinBalai) > abs( posBalai - posMaxBalai ) ){
		delta = abs( posBalai - posMinBalai ); // nb de degré pour arriver en position
		delayPos = (delta * 20)/60; 	// le servo met 20'' pour faire 60°, combien pour le delta?
		
		posBalai = posMinBalai;
		sensBalai = true;
	}else{
		delta = abs( posBalai - posMinBalai ); // nb de degré pour arriver en position
		delayPos = (delta * 20)/60; 	// le servo met 20'' pour faire 60°, combien pour le delta?
		
		posBalai = posMaxBalai;
		sensBalai = false;
	}
	
	// pour forcer le scan au changement de mode
	stampTimeScan = 4294967295;
	if( isDebug ){
		Serial.print("Delay servo balai : ");
		Serial.print(delayPos);
		Serial.print(" delta : ");
		Serial.println(delta);
	}
		
	balai.write( posBalai );
	delay( delayPos );
}

void modeServoBalai( int mode ){
	if( mode != balaiRunningMode ){
		switch( mode ){
			case BALAI_MODE_ARRET :
				balaiRunningMode = BALAI_MODE_ARRET;
				prepareServoBalai( 0, 180, 30, 10000 ); // a l'arret toute les 10sec on scan le périmètre complet
				if( isDebug )
					Serial.println( "BALAI_MODE_ARRET" );
				break;
			case BALAI_MODE_AVANCE :
				balaiRunningMode = BALAI_MODE_AVANCE;
				prepareServoBalai( 60, 120, 30, 500 ); // en avancant, tt les 500ms on scan un petit périmètre
				if( isDebug )
					Serial.println( "BALAI_MODE_AVANCE" );
				break;
			case BALAI_MODE_DRT :
				balaiRunningMode = BALAI_MODE_DRT;
				prepareServoBalai( 120, 180, 30, 500 );
				if( isDebug )
					Serial.println( "BALAI_MODE_DRT" );
				break;
			case BALAI_MODE_GCH :
				balaiRunningMode = BALAI_MODE_GCH;
				prepareServoBalai( 0, 60, 30, 500 );
				if( isDebug )
					Serial.println( "BALAI_MODE_GCH" );
				break;
			default:
				prepareServoBalai( 90, 90, 30, 100000 );
				if( isDebug )
					Serial.println( "BALAI_MODE_..... pas de mode" );
				break;
		}
	}
}
//actionBalai( iaParams[2], iaParams[3] )

/**
 *
 * Actions sur les lumières
 *
 */
void actionLights( int iLight, int iValue ){
	if( isDebug ){
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
 
void actionUS(){
	unsigned long runningTimeUS; // temps depuis le dernier checkingTimeUS
	unsigned long runningTimeScan; // temps depuis le dernier checkingTimeScan
	
	runningTimeScan = millis() - stampTimeScan;
	if( runningTimeScan > checkingTimeScan ){
		isBalaiScanComplet = false;
		stampTimeScan = millis();
	}	


	if( !isBalaiScanComplet ){
		runningTimeUS = millis() - stampTimeUS;
		if( runningTimeUS > checkingTimeUS ){
			stampTimeUS = millis();
			
			// On affiche la distance
			actionStateUltraSonic();
			// On demande au servo de bouger 
			actionServosBalai();
		}
	}
}

int getDistanceUS(){
	int duration, distance;
	
	// Initialisation du capteur
	digitalWrite(US_TRIG_PIN, HIGH);
	delayMicroseconds(1000);
	digitalWrite(US_TRIG_PIN, LOW);
	// Mesure du temps : allé/retour de l'ultrason
	duration = pulseIn(US_ECHO_PIN, HIGH, 12000); // l'ultrason ne peut mesuré précisément que jusqu'a 4-5m, je mets un timeout pour 10m
	// conversion en distance.
	distance = (duration/2) * 0.03495;
	
	// s'il y a timeout, duration va etre = 0 donc on met la distance a 10m
	if( duration == 0 )
		distance = 1000;
	
	return distance;
}


/**
 *
 * Capteur Voltage
 *
 */
void actionVoltage(){
	// on récupère la valeur du capteur
	int sensorValue;  
	sensorValue=analogRead(VOL_SENOR);
	// on incrémente
	voltageSamplingNb ++;
	voltageValuesSum += sensorValue;

	// si nous avons passé le nombre d'échantillonage
	if( voltageSamplingNb > voltageSamples ){
		// on calcule la moyenne
		sensorValue = voltageValuesSum / voltageSamples; 
		voltage = GAIN * sensorValue * ADC_REF / 1023.00; // tiré de l'exemple officiel
		// reset
		voltageSamplingNb = 0;
		voltageValuesSum = 0;
		// affichage
		actionStateVoltage();
	}

}



/**
 *
 * Retour d'états
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

void actionStateVoltage( ){
	Serial.print( "[data][sensor][voltage]" );
	Serial.println( voltage );
}













