/***********************************************************************************************************************\
	* Intitulé * 
	Asservissement d'un chassis avec 4 moteurs (2 gauche/2 droite).
	Le but est de lancer les bases de l'automatisation des deplacements.

	* Matériel *
	- 2 CI L293D (double pont en H) pour la gestion des moteurs : cablage redondant de chaque cote du L293D
		Ainsi avec 3 fils je pilote les 2 moteurs d'un cote (3 de plus pour l'autre cote) et chaque moteur est monte sur un seul pont en H. 
		(repartition de la charge electrique) plutot que 2 moteurs cables ensemble sur un seul pont en H.
	- 1 Servo-Moteur pour le 'balai' -> servo qui oscille pour la detection de distance.
	- Le tout pilote par un multiplexer TLC5940
	
	Il est donc necessaire d'inclure les bibliotheques Tlc5940.H et tlc_servos.H. Ce dernier a pour but de baisser la Frequence de PWM a 50Hz
	
	
	* Pin *
	- CF cablage TLC5940
	

	* Remarques *
	*Code qui n'a pas pour but d'étre fonctionnel mais de lancer les fonctions primaires de gestion de la direction.
	Ceci comporte :
	- Reglage de la vitesse et rotation du chassis.
	- Gestion du servo balai et detection ultra-son
	
	*La gestion du PWM est inversee : 0 -> PWM actif | 4095 -> PWM inactif.
	(a confirmer...)

	
	11/10/2013
	Par ORTIZ Clement.

\***********************************************************************************************************************/

//////////////////////////////////////
// 		Appels de bibliotheques		//
//////////////////////////////////////
// Reference the TLC5940 multiplexer
// #include "Tlc5940.h"
// #include "tlc_servos.h"
// Reference the I2C Library
#include <Wire.h>
// Reference the HMC5883L Compass Library
#include <HMC5883L.h>

// pour test en direct sur les servos
#include <Servo.h> 



//////////////////////////////////////////
// 		Declaration des constantes		//
//////////////////////////////////////////

/////////////////////////
// Globales
#define MAX_PWM		200		
#define MIN_PWM		107


/////////////////////////
// Moteurs

// Mode lors du pilotage a distance
#define MODE_MOTOR	1
// Moteurs Gauche
#define MG_S1   	22//2	//0	// Pin Moteurs Gauche Sens 1
#define MG_S2   	23//4	//1	// Pin Moteurs Gauche Sens 2
#define MG_V   		3	//2	// Pin Moteurs Gauche Vitesse
// Moteurs Droite
#define MD_S1   	24//7	//3	// Pin Moteurs Droite Sens 1
#define MD_S2   	25//8	//4	// Pin Moteurs Droite Sens 2
#define MD_V   		4//6	//5	// Pin Moteurs Droite Vitesse


/////////////////////////
// Servo-Moteur
// Mode lors du pilotage a distance
#define MODE_SERVO	2

// Servo BALAI
#define S_BALAI				9	//16	// Pin Servo Balai
#define S_BALAI_DELAY 		200	// Delai en ms, entre 2 mouvements du balai ... 0.20 sec/60é à 5v pour le S3010(0.23/60 pour le S3003) donc 0.1s/30é(0.12/30) donc ...100 ms (120ms)(à caler avec le S_BALAI_PAS)
#define S_BALAI_PAS 		30	// Nb de Degre entre 2 balayage.
#define S_BALAI_NBCHECK		3	// Nb de positions a checker.
#define S_BALAI_ANGLE_MIN	0	// Positions minimale
#define S_BALAI_ANGLE_MAX	180	// Positions maximale

// Servos Tourelle
#define S_HORI				10 	// Pin
#define S_HORI_NAME			1 	// N° du servo
#define S_VERT				11 	// Pin
#define S_VERT_NAME			2 	// N° du servo
// Valeurs de buttés pour ne pas que les servos forcent
#define POS_MIN_HORI 		33
#define POS_MAX_HORI 		148
#define POS_MIN_VERT 		50
#define POS_MAX_VERT		147

/////////////////////////
// Lumière(s)
#define MODE_LIGHT			3	
#define LIGHT_CAM			52 	// pin de la lumière pour la caméra
#define LIGHT_CAM_NAME		1 	// N° de la light
#define LIGHT_STROMB		53 	// pin de la lumière pour le Stromboscope
#define LIGHT_STROMB_NAME	2 	// N° de la light
#define LIGHT_LAZER			7 	// pin de la lumière pour le Lazer
#define LIGHT_LAZER_NAME	3 	// N° de la light


/////////////////////////
// Capteurs

// Boussole / Compass
#define C_ERROR_MARGE		10 	// Nb de degrés d'erreur
#define C_MODER				3	// Nbvaleur relevé pour la moyenne

// Ultrason
#define US_TRIG		12		// Pin Triguer
#define US_ECHO		13		// Pin Echo
#define US_ALIM		36		//11		// Pin Alim ... pour la phase de test ... sera sur une vrai alim par la suite.
#define US_TIMEOUT	25000	// TIMEOUT en Microsec. (Important car le script se bloque durant ce temps quand les obstacles sont trop loins)
#define US_MODER	5		// Combien de relevés pour faire une moyenne?

/********************************************************************************************************************************************\
pi : le cone de detection de l'UltraSon semble etre entre 20-30°
Malheureusement un effet de bord apparait qd l'objet est sur ces limites...

du coup, le drone faisant 14cm de large, pour un cone de 20° si aucune detection en face à moins de 40cm, le drone passe
pour un cone de 30°, seulement 26 cm sont necessaire pour que le dronne passe. (26+40 / 2 = Moyenne de 33)
											|
______										|
      |------------------------------------------>
______|------------------------------------------>
											|
											|


On sait que le capteur ne capte pas plus loin que 2 bon mètres.
on sait également calculer la distance par rapport au temps d'attente... en inversant la formule... on va savoir combien de temps il faut pour calculer 2m ...
et ainsi pouvoir caler le TIMEOUT sur ... 2,5m par exemple, sachant qu'on ne pourra pas detecter plus loin ... et ceci évitera d'avoir un blocage de la boucle le temps du TIMEOUT
    	fDistance = ( fDuration/2 ) * 0.03495
donc fDuration = ( 2*fDistance ) / 0.03495
Ce qui nous donne pour 2,5m : 14306,151645207439198855507868383 que l'on va arrondir à 15 000.
Ce qui nous donne pour 3m : 25751,07296137339055793991416309 que l'on va arrondir à 25 000.
--> Pour info, il faut 25 000µs soit 25 millis MAX pour que le capteur fasse un relevé + 100 millis pour que le servo se mette en place 
soit : 125 millis pour faire un relevé sur une trajectoire.
Comme l'on fait une moyenne de 3 relevés : 25*3 + 100= 175
* et que nous avons un parametrage de 3 NBCHECK : 175*3 =  525ms Maximum pour un relevé complet avec une moyenne de 3 mesures sur 3 axes.
* Si qu'un seul relevé et que nous avons un parametrage de 3 NBCHECK : 125*3 =  375ms Maximum pour un relevé complet sans moyenne.
... il serait intéréssant de connaitre la vitesse du module pour savoir combien de cm sont parcouru le temps d'un relevé ;)
a 1Km/h : 0,277m/s soit 28cm/s donc environ 15cm pr 1/2s ... il sera intéréssant de baisser la vitesse si on a besoin de plus de précision!

relevé : 
Angle 0 Timing 579 : 0 | 30 | 60
Angle 30 Timing 579 : 0 | 30 | 60
Angle 60 Timing 578 : 30 | 60 | 90
Angle 90 Timing 579 : 60 | 90 | 120
Angle 120 Timing 578 : 90 | 120 | 150
Angle 150 Timing 579 : 120 | 150 | 180
Angle 180 Timing 579 : 120 | 150 | 180 

Le copier/colle ici m'a fait pensé que si le servo est completement a l'autre bout ... il faut mettre un delay bien plus grand pour attendre qu'il arrive...
le delay est a calculer en fonction de la position initiale du servo du coup : 
Angle 60 Timing 588 : 30 | 60 | 90
Angle 90 Timing 579 : 60 | 90 | 120
Angle 120 Timing 580 : 90 | 120 | 150

... la il y a passage de la position 150é à la position 30é soit 4*30é donc ... 4*100 de delay...+ les environ 525 de traitement .... ca commence à faire long! 
Angle 60 Timing 889 : 30 | 60 | 90
Angle 90 Timing 579 : 60 | 90 | 120
Angle 120 Timing 580 : 90 | 120 | 150
--> il va falloir optimiser le déplacement du servo de facon a ce qu'il commence le tableau d'angle par la valeur la plus proche de sa position initiale.

Apres Optimisation : 
90 | 60 | 30 : Angle 60 Timing 679
60 | 90 | 120 : Angle 90 Timing 579
90 | 120 | 150 : Angle 120 Timing 579
--> On voit bien que le tableau est parcouru à l'envers pour la 1ere ligne du coup le timing est de 889-679 = 210ms plus rapide :)

En SpeedCheck
30 | 60 | 90 : Angle 60 Timing 393
60 | 90 | 120 : Angle 90 Timing 393
90 | 120 | 150 : Angle 120 Timing 393

90 | 60 | 30 : Angle 60 Timing 494
60 | 90 | 120 : Angle 90 Timing 393
90 | 120 | 150 : Angle 120 Timing 394
--> au lieu de 3 check par direction, il n'y en a plus qu'un. 579-393 = 186ms plus rapide.

\****************************************************************************************************************************************/

// Amperemetre
#define AMP			0	//Pin analog pour l'amperemetre




//////////////////////////////////////////
// 		Declaration des variables		//
//////////////////////////////////////////
boolean bDebug = true;
int iTimeStamp_Demo = 0;

/////////////////////////
// Moteurs
int iCountMargeMG =	0; 		// compte le nombre de fois qu'on est en dessous du MIN_PWM pour le Moteur Gauche
int iCountMargeMD =	0; 		// compte le nombre de fois qu'on est en dessous du MIN_PWM pour le Moteur Droit
int COUNT_MARGE; 			// nombre de fois ou il sera toléré un < MIN_PWM ... initialisé dans le SETUP()

	
// Moteurs Gauche
int iMG_S1 = 		0;		// valeur du MG_S1 (MIN_PWM ou MAX_PWM)
int iMG_S2 = 		0;		// valeur du MG_S2 (MIN_PWM ou MAX_PWM)
int iMG_V = 		0;		// valeur du MG_V (de MIN_PWM a MAX_PWM)


// Moteurs Droite
int iMD_S1 = 		0;		// valeur du MD_S1 (MIN_PWM ou MAX_PWM)
int iMD_S2 = 		0;		// valeur du MD_S2 (MIN_PWM ou MAX_PWM)
int iMD_V = 		0;		// valeur du MD_V (de MIN_PWM a MAX_PWM)

// Pilotage a distance
int iVitesse = 0; 			// Vitesse du drone
int iDelta = 0;				// Delta entre les 2 moteurs
boolean bInvertX = false;	// Inverser gauche/droite
boolean bInvertY = false;	// Inverser Avancer/Reculer



/////////////////////////
// Servo-Moteur

// Servo BALAI
Servo servo_balai;
int iBalai_pos =			0; 	// Position du servo Balai
int iBalai_TimeStamp =		0; 	// Reference de temps qui sera compare a S_BALAI_DELAY
// Pour Test
int iInitPos = 0; 				// position de depart du servo balai
int iFinPos = 180; 				// position de fin du servo balai

// Servos Tourelle
Servo servo_hori;
Servo servo_vert;
int iServo = 0; 				// Servo sur lequel on travaille en mode pilotage a distance
int iServoPos = 0; 				// Position du Servo sur lequel on travaille en mode pilotage a distance


/////////////////////////
// Lumière(s)
int iLightNum = 0;
int iLightState = 0;

/////////////////////////
// Capteurs

// Boussole / Compass.
HMC5883L compass;



////////////////////////
// Pilotage

int iDirection = 100; 			// Direction, en ° (0 et +360 par rapport au Nord) que doit prendre le drone.



///////////////////////
// Port Série
String inputString = "";  		// Détient la chaine de caractère qui est réceptionnée
boolean stringComplete = false;	// Indicateur de fin de commande.

int iMode = 0;  // sur quel materiel va-t-on travailler




//////////////////////////////////////
// 		Fonctions Principales		//
//////////////////////////////////////

void setup()
{
	if( bDebug )
		COUNT_MARGE = 5;	// nombre de fois ou il sera toléré un < MIN_PWM
	else	
		COUNT_MARGE = 100;

	// Port Serie
	Serial.begin(/* 115200 */ 9600 );
	Serial.println( "Ouverture du port serie" );

	// Init du multiplexer TLC5940
	// Tlc.init();
	// tlc_initServos();  // Note: this will drop the PWM freqency down to 50Hz.
	
	// UltraSon
	pinMode(US_TRIG, OUTPUT);
	pinMode(US_ECHO, INPUT);
	pinMode(US_ALIM, OUTPUT);
	Serial.println( "Declaration de l'UltraSon" );
	// Pour Test : On allume le capteur ultra-son
	digitalWrite( US_ALIM, HIGH );

	// Servos
	servo_balai.attach( S_BALAI );  	// attache le servo balai au pin  
	Serial.println( "Declaration Servo Balai" );
	servo_hori.attach( S_HORI );  	// attache le servo horizontal au pin  
	servo_vert.attach( S_VERT );  	// attache le servo vertical au pin  
	Serial.println( "Declaration Servo Tourelle" );

	// Boussole / Compass
	Serial.println( "Connexion a la boussole" );
	init_compass();
	// if( init_compass() != 0 )
		// Serial.println(compass.GetErrorText(error));
	
	// Lights
	pinMode(LIGHT_CAM, 	  OUTPUT);
	pinMode(LIGHT_STROMB, OUTPUT);
	pinMode(LIGHT_LAZER, OUTPUT);
	
	
	// servo_balai.write( iInitPos );
	
	
	Serial.print( "Mode Debug : " );
	Serial.println( (bDebug?"ON":"OFF") );
	Serial.println( "Ready !" );
	Serial.println( "" );

}



void loop()
{
	///////////////////////////////////
	// Reception via port serie
	
	// On check si on a fini la reception des données. (caractère d'échappement '*')
	if( stringComplete ){
		// Pour test : Permet de savoir vraiment ce qui est retenu
 		Serial.println();
		Serial.print("-* ");
		Serial.print(inputString);
		Serial.println(" *-");
 		
		// Transforme la commande en ordre
		if( command2order() ){
/* 			// Vérifions dans quel mode nous sommes
			if( iMode == MODE_SERVO ){
				// Affiche des résultats
 				Serial.print( "Servo : " );
				Serial.println( iServo );
				Serial.print( "Position : " );
				Serial.println( iServoPos );
 			}else if( iMode == MODE_MOTOR ){
				// Affiche des résultats
				Serial.print( "Moteur : " );
				// Serial.println( moteurX );
				Serial.print( "Sens : " );
				// Serial.println( sensX );
				Serial.print( "Vitesse : " );
				// Serial.println( vitesseX );
			} */
		}else
			Serial.println( "Params non OK" );
		
		// On réinitialise la chaine:
		inputString = "";
		stringComplete = false;
	}



	///////////////////////////////////
	// Test de matériels
	
	/* test correlation moteurs */
	// 3sec du coté Gauche en Avant
/* 	if( millis() - iTimeStamp_Demo < 3000 ){
		affect_MG( MAX_PWM/2 );
		affect_MD( 0 );
	}else 
	// 3sec coté Droit en Avant
	if( millis() - iTimeStamp_Demo < 6000 ){
		affect_MG( 0 );
		affect_MD( MAX_PWM/2 );
	}else
	// 3sec du coté Gauche en Arrière
	if( millis() - iTimeStamp_Demo < 9000 ){
		affect_MG( -MAX_PWM/2 );
		affect_MD( 0 );
	}else 
	// 3sec coté Droit en Arrière
	if( millis() - iTimeStamp_Demo < 12000 ){
		affect_MG( 0 );
		affect_MD( -MAX_PWM/2 );
	}else
		iTimeStamp_Demo = millis();
		
	ordre_moteur(); */
	
	
	/* Test Compass */
	// Serial.println( get_Orientation() );

	
	/* Test UltraSon */

	// float iMesure = 0;
	// float iMesureMoy = 0;
	
	// digitalWrite( US_ALIM, HIGH );
	// iMesure = get_distance();
	// iMesureMoy = get_moderated_distance();

	// Serial.print( "Mesure Simple : " );
	// Serial.println( iMesure );
	
	// Serial.print( "Mesure Moyenne : " );
	// Serial.println( iMesureMoy );
	


	
	
	///////////////////////////////////
	// Algos de pilotage
	/* test de modification de trajectoire */
	// traj_modifier( iDirection );

	
	
	/* test Compass + Moteurs */
	// command_moteurs_compass( 0, iDirection );
	
	

	/* Test UltraSon + Balai 
	int aAngles[ S_BALAI_NBCHECK ];
	int aMesures[ S_BALAI_NBCHECK ];
	//for( int i=60; i <= 120; i+=30 ){
		// Serial.print("check_angles : ");
		// Serial.println(i);
		check_angles( 77, aAngles, aMesures, false);
		for( int j=0; j < 3; j++ ){
			Serial.print("Angle : ");
			Serial.print(aAngles[j]);
			Serial.print(" Distance : ");
			Serial.println(aMesures[j]);
			
		}
		
	//}
	*/
	

/* 	digitalWrite( US_ALIM, HIGH );

 	Serial.print( "S_BALAI_NBCHECK : " );
	Serial.println( S_BALAI_NBCHECK );
	
	Serial.print( "iInitPos : " );
	Serial.println( iInitPos );
	
	Serial.print( "S_BALAI_ANGLE_MAX : " );
	Serial.println( S_BALAI_ANGLE_MAX );
	
	for( int iAngle =iInitPos; iAngle <= iFinPos; iAngle += S_BALAI_PAS ){
	// for( int iAngle =S_BALAI_ANGLE_MAX; iAngle >= iInitPos; iAngle -= S_BALAI_PAS ){
	
		
		int aAngles[ S_BALAI_NBCHECK ];
		int aMesures[ S_BALAI_NBCHECK ];

		long lTimeStart = millis();

		// gen_angles( iAngle, aAngles );
		check_angles( iAngle, aAngles, aMesures, false );
		
		long lTimeTotal = millis() - lTimeStart;
		
		Serial.print( "Angle " );
		Serial.print( iAngle );
		Serial.print( " Timing " );
		Serial.print( lTimeTotal );
		Serial.println( "" );
		
 		for( int j=0; j < S_BALAI_NBCHECK ; j++){
			Serial.print( aAngles[j] );
			Serial.print( " : " );
			Serial.print( aMesures[j] );
			if( j != S_BALAI_NBCHECK -1 )
				Serial.print( " | " );
		}
		Serial.println( "" ); 
		delay( 2000 );
	} 
	Serial.println( "" );
 */	

 
	//////////////////////
	// Test Amperemetre
	// Serial.print( "Amperemetre : " );
	// Serial.println( get_amperage() );
 
 
/*  	if( bDebug ){
		Serial.println("");
		delay( 2000 );
	}  */
}



//////////////////////////////////////
// 		Gestion du port série		//
//////////////////////////////////////

/* Fonction de réception des données sur le port série
 * determine quand la reception d'une commande est terminée
 * met la commande dans "inputString". 
 */
void serialEvent() {
	// Tant que nous avons des données à réceptionner
	while( Serial.available() ){
		// Récupère un nouvel Octet:
		char inChar = Serial.read();
		
		// Si le caractère réceptionné est *
		// La boucle principale va pouvoir s'occuper du résultat.
		if( inChar == '*' ){
			stringComplete = true;
		}else
			// Sinon, on concatène l'octet à "inputString" ... si on n'est pas en mode stringComplete ... ca évite l'empilage de parametres qd ils arrivent trop rapidement..
			if( !stringComplete )
				if(inChar >= 32) // Ne concatène que des caractère imprimable.
					inputString += inChar;
	}
	 
	// Ce qu'il faudrait faire : 
	// Lire le buffer sans le vider
	// Voir s'il y a une '*' ... si oui, on récup la commande
	// et on vide le buffer jusqu'a '*'
	// Ainsi on récupèrerait commande par commande.
	 
	// Avec la technique actuelle ... s'il y a 2 commandes en buffer... on ne lit que la 1ere...et encore...
	// Se pencher sur : readStringUntil()
}

/* Fonction qui va parser la chaine de caractère "params" (la commande)
 * et mettre les résultats dans les différentes variables temporaires globales
 */
boolean get_params( String params ){
    // Indicateur de validité des Params
    boolean ok = true;
    
    // On met tt dans un tableau
    int myStringLength = params.length();
    char myChar[ myStringLength+1 ];
    params.toCharArray( myChar, myStringLength+1 );

    //// On check 1er parametre 
    // Le mode (Gestion Servo ou Moteur) : numérique compris entre 0 et 3
    if( (int)myChar[0] < '0' || (int)myChar[0] > '4' ){
		ok = false;
    }else
		iMode = myChar[0] - '0';


    //// En fonction du mode on ne va pas récupérer les memes type de parametres.
    // Mode SERVO
    if( iMode == MODE_SERVO ){
		// On check 2eme parametre 
		// N° Servo : numérique compris entre 0 et 2
		if( (int)myChar[1] < '0' || (int)myChar[1] > '2' )
			ok = false;
		else
			iServo = myChar[1] - '0';
    
		// Si pas d'erreur, 
		if( ok ){
			// On check le 3eme
			// Position : Numérique compris entre pos_min et pos_max.
			char vitChar[ myStringLength ]; 
			// il faut ajouter le 0 de fin de chaine sinon ATOI décone :)
			vitChar[myStringLength - 1] = 0; 
			// On récup les char apres le Xeme, que l'on concatène pour avoir le chiffre contenant le ° de rotation du servo
			int x = 2 ; // position de départ (en fonction du nombre de parametres avant celui-ci)
			for( int i=x; i < myStringLength; i++){
				vitChar[ i-x ] = myChar[i];
			}
	   
			// On transforme la chaine nouvellement créée en Integer
			int pos_temp = atoi( vitChar );
			// On vérifie que nous sommes bien dans les bornes
			if( iServo == S_HORI_NAME ){
				if( pos_temp < POS_MIN_HORI )
					iServoPos = POS_MIN_HORI;
				else if ( pos_temp > POS_MAX_HORI )
					iServoPos = POS_MAX_HORI;
				else
					iServoPos = pos_temp;
			}else if( iServo == S_VERT_NAME ){
				if( pos_temp < POS_MIN_VERT )
					iServoPos = POS_MIN_VERT;
				else if( pos_temp > POS_MAX_VERT )
					iServoPos = POS_MAX_VERT;
				else
					iServoPos = pos_temp;
			}
		}
    
    }else 
	// Mode MOTOR
	if( iMode == MODE_MOTOR ){
		///////////////////////
		// On check 2eme parametre 
		// Vitesse : compris entre -MAX_PWM et +MAX_PWM
		const int iPosStart = 1 ; // position de départ (en fonction du nombre de parametres avant celui-ci)
		int iSeparator = 0;			// offset du Separateur
		
		// On cherche le symbole de séparation '.'
		for(int i=0; i < myStringLength; i++){
			if( (int)myChar[i] == '.' ){
				iSeparator = i;
				i = myStringLength;
			}
		}
		// Si le séparateur ne tombe pas au debut ou a la fin alors on récupère la vitesse
		if( iSeparator != 0 && iSeparator != 1 && iSeparator != myStringLength-1 ){
			int iTaille = iSeparator - iPosStart + 1; // Différence entre l'emplacement du separateur et la position de départ...+1 pour le zero :)
			char sVitChar[ iTaille ]; 
			// il faut ajouter le 0 de fin de chaine sinon ATOI décone :)
			sVitChar[iTaille - 1] = 0;
			
			// On concatène dans un tableau
			for( int i=iPosStart; i < iSeparator; i++){
				sVitChar[ i-iPosStart ] = myChar[i];
			}
			
			// On transforme en int
			int iVit = atoi( sVitChar );
			if( iVit < -MAX_PWM )
				iVitesse = -MAX_PWM;
			else if( iVit > MAX_PWM )
				iVitesse = MAX_PWM;
			else
				iVitesse = iVit;
				
			if( bInvertY )
				iVitesse = -iVitesse;
			
			// if( bDebug ){
				// Serial.print( "Vitesse : " );
				// Serial.println( iVitesse );
			// }
		}else
			ok = false;
		
		
		///////////////////////
		// On check 3eme parametre 
		// Delta : compris entre -MAX_PWM*2 et +MAX_PWM*2
		if( ok ){
			int iTaille = myStringLength - iSeparator ; // Différence entre l'emplacement du separateur et la fin de la chaine...+1 pour le zero :)
			char sDeltaChar[ iTaille ]; 
			
				// if( bDebug ){
					// Serial.print( "iTaille : " );
					// Serial.println( iTaille );
				// }
			
			// On concatène dans un tableau
			for( int i=iSeparator+1; i < myStringLength; i++){
				int offset = i-iSeparator-1;
				sDeltaChar[ offset ] = myChar[i];
				// if( bDebug ){
					// Serial.print( "offset : " );
					// Serial.print( offset );
					// Serial.print( " : " );
					// Serial.print( i );
				// }
			}
			// il faut ajouter le 0 de fin de chaine sinon ATOI décone :)
			sDeltaChar[iTaille - 1] = 0;
			
			// if( bDebug ){
				// Serial.print( "sDeltaChar : " );
				// Serial.println( sDeltaChar );
			// }
			
			// On transforme en int
			int iDel = atoi( sDeltaChar );
			if( iDel < -MAX_PWM*2 )
				iDelta = -MAX_PWM*2;
			else if( iDel > MAX_PWM*2 )
				iDelta = MAX_PWM*2;
			else
				iDelta = iDel;
				
			if( bInvertX )
				iDelta = -iDelta;
			// if( bDebug ){
				// Serial.print( "Delta : " );
				// Serial.println( iDelta );
			// }

		}
		
		
		
		
/* 		if( (int)myChar[1] < '0' || (int)myChar[1] > '2' )
			ok = false;
		else
			moteurX = myChar[1] - '0';

		// On check 3eme parametre 
		// Sens : numérique compris entre 0 et 2
		if( (int)myChar[2] < '0' || (int)myChar[2] > '2' )
			ok = false;
		else
			sensX = myChar[2] - '0';
     
		// On check le 4eme
		// Vitesse : Numérique compris entre 0 et 255.
		char vitChar[ myStringLength ]; 
        // il faut ajouter le 0 de fin de chaine sinon ATOI décone :)
        vitChar[myStringLength - 1] = 0; 
        // On récup les char apres le Xeme, que l'on concatène pour avoir le chiffre contenant la vitesse du moteur. (0 à 255)
        int x = 3 ; // position de départ (en fonction du nombre de parametres avant celui-ci)
        for( int i=x; i < myStringLength; i++){
			vitChar[ i-x ] = myChar[i];
        }

		if( ok ){
			int vit = atoi( vitChar );
			if( vit < 0 || vit > 255 )
				ok = false;
			else
				vitesseX = vit;
		} */

    }else 
	if( iMode == MODE_LIGHT ){
		// si la chaine est assez longue
		if( myStringLength >= 3 && myStringLength <= 5){
			//////////////////////
			// Numéro de la Light
			iLightNum = myChar[1] - '0';
			
			if( iLightNum != LIGHT_LAZER_NAME ){
				iLightState = myChar[2] - '0';			
			}else{
				//////////////////
				// PWM de la light : iLightState
				int iPosStart = 2; // on a deja puisé 2 caractère dans la chaine
				int iTaille = myStringLength - iPosStart +1; // Taille de la chaine -2 caractère...+1 pour le zero :)
				char sVitChar[ iTaille ]; 
				// il faut ajouter le 0 de fin de chaine sinon ATOI décone :)
				sVitChar[iTaille - 1] = 0;
				
				// On concatène dans un tableau
				for( int i=iPosStart; i < myStringLength; i++){
					sVitChar[ i-iPosStart ] = myChar[i];
				}
				
				// On transforme en int
				iLightState = atoi( sVitChar );
				// Verif
				if( iLightState < 0 )
					iLightState = 0;
				else if( iLightState > 255 )
					iLightState = 255;
				}
		}else
			ok = false;
	}else
/* 	if( iMode == MODE_BALAI ){
		// on regarde s'il allumer ou eteindre
		etat_balai = myChar[1] - '0';
		Serial.print( "Etat Balai : " );
		Serial.println( etat_balai );
		
		if( etat_balai == 0 )
		  servo_balai.write( 90 );
    // Si on est pas en mode MOTOR, SERVO ou BALAI alors il y a une erreur.
	}else */
      ok = false;
  
	return ok;
} 

/* Fonction qui va donner les ordre suite à la commande receptionnée.
 * retourne la validité de la commande.
 * /!\ utilise les variables globales : inputString, mode, + tt les var des moteurs/servos
 */
boolean command2order(){
	boolean ok = true;
    // On parse pour savoir si tt est OK. et on récup les parametres au passage
    if( get_params( inputString ) ){
		// Vérifions dans quel mode nous sommes
		if( iMode == MODE_SERVO ){
			// Action sur le servo
			ordre_servo( iServoPos, iServo );
		}else if( iMode == MODE_MOTOR ){
			// Action sur le moteur
			//ordre_moteur( moteurX, sensX, vitesseX );
			command_moteurs( iVitesse, iDelta );
			
			if( bDebug ){
				Serial.print( "Cmd2Order Vitesse : " );
				Serial.print( iVitesse );
				Serial.print( " Delta : " );
				Serial.println( iDelta );
			}
		}else if( iMode == MODE_LIGHT ){
			if( bDebug ){
				Serial.print("Light Num : ");
				Serial.print(iLightNum);
				Serial.print(" Light Etat : ");
				Serial.print(iLightState);
			}
			
			if( iLightNum == LIGHT_CAM_NAME ){
				if( iLightState )
					digitalWrite( LIGHT_CAM, HIGH );
				else
					digitalWrite( LIGHT_CAM, LOW );
					
			}else if( iLightNum == LIGHT_STROMB_NAME ){
				if( iLightState )
					digitalWrite( LIGHT_STROMB, HIGH );
				else
					digitalWrite( LIGHT_STROMB, LOW );
					
			}else if( iLightNum == LIGHT_LAZER_NAME ){
				analogWrite( LIGHT_LAZER, iLightState );
			}
		}
    }else{
		ok = false;
		// a la reception de données érronées : On stope les machines!
		//ordre_moteur( 0, 0, 0 );
	}
     


	return ok;
}




//////////////////////////////////////
//		Gestion des moteurs			//
//////////////////////////////////////

/* Depuis les variables globales, va donner l'action aux moteurs
 *
 */
void ordre_moteur(){
/*  	Tlc.clear();// clear ou pas clear....? car le clear va supprimer tt les valeurs ... qui peuvent piloter d'autre choses...
    // Moteur Gauche 
    Tlc.set(MG_S1, iMG_S1);
    Tlc.set(MG_S2, iMG_S2);
    Tlc.set(MG_V, iMG_V);
    // Moteur Droit 
    Tlc.set(MD_S1, iMD_S1);
    Tlc.set(MD_S2, iMD_S2);
    Tlc.set(MD_V, iMD_V);
    Tlc.update(); 
 */
	

    // Moteur Gauche 
    digitalWrite(MG_S1, iMG_S1);
    digitalWrite(MG_S2, iMG_S2);
    analogWrite(MG_V, iMG_V);
    // Moteur Droit 
    digitalWrite(MD_S1, iMD_S1);
    digitalWrite(MD_S2, iMD_S2);
    analogWrite(MD_V, iMD_V);
	
 	if( bDebug ){
		Serial.print("MGV (");
		Serial.print(MG_V);
		Serial.print(") : ");
		Serial.print(iMG_V);
		Serial.print(" iMG_S1 : ");
		Serial.print(iMG_S1);
		Serial.print(" MGS2 : ");
		Serial.println(iMG_S2);
		
		Serial.print("MDV : ");
		Serial.print(iMD_V);
		Serial.print(" MD_S1 : ");
		Serial.print(iMD_S1);
		Serial.print(" MDS2 : ");
		Serial.println(iMD_S2);	
	}	
}

/* Va attribuer les valeurs aux variables globales concernant le moteur de gauche (MG) ou de droite (MD)
 * (sens 1 avance, 2 recule ... si erreur... il faut le modifier partout, avec cette fonction il n'y aura qu'a le modifier ici)
 *
 * On va en profiter pour tenter de corriger un probleme physique...
 * Il y a un seuil ou le PWM(iVitesse) n'est pas suffisant pour faire bouger le drone a cause de son poid.
 * On va donc garder iVitesse au dessus de ce seuil tant qu'on n'est pas ds la partie inférieure...
 *
 * 0	seuil/2	       seuil ou le PWM n'est suffisant
   |		  |		     |
   V		  V          V
 * |----------|----------|############.... -> PWM suffisant pour déplacer le drone
 * |   V=0	  |  V=seuil |
 */
int regul_vitesse( int iVitesse, boolean bMoteur ){
	// Sinon on ne peut pas s'arreter :)
	if( iVitesse != 0 ){
		// Si on est entre le 1/2 seuil : On met le moteur a zero car il n'arrivera pas a faire son effet.
		// par contre on compte... car sinon on va rester bloquer dans cette direction qui n'est apparemment pas la bonne.
		if( iVitesse < MIN_PWM/2 && iVitesse > -MIN_PWM/2 ){
			if( bMoteur )
				iCountMargeMD ++;
			else
				iCountMargeMG ++;
				
			if( iCountMargeMG > COUNT_MARGE || iCountMargeMD > COUNT_MARGE ){
				if( iVitesse < MIN_PWM/2 && iVitesse > 0 )
					iVitesse = MIN_PWM;
				else
					iVitesse = -MIN_PWM;
					
				if( bMoteur )
					iCountMargeMD = 0;
				else
					iCountMargeMG = 0;
			}else
				iVitesse = 0;
		}
		else if( iVitesse > MIN_PWM/2 && iVitesse < MIN_PWM )
			iVitesse = MIN_PWM;
		else if( iVitesse < -MIN_PWM/2 && iVitesse > -MIN_PWM )
			iVitesse = -MIN_PWM;
		else{
			// Si tout va bien, on remets les compteurs à 0.
			if( bMoteur )
				iCountMargeMD = 0;
			else
				iCountMargeMG = 0;
		}	
	}
	return iVitesse;		
}
void affect_MG( int iVitesse ){
	//iVitesse = regul_vitesse( iVitesse , false );
		
	if( iVitesse > 0 ){
		iMG_S1 = MAX_PWM;
		iMG_S2 = 0;
		iMG_V = iVitesse;
	}else{
		iMG_S1 = 0;
		iMG_S2 = MAX_PWM;
		iMG_V = -iVitesse;
	}
}
void affect_MD( int iVitesse ){
	//iVitesse = regul_vitesse( iVitesse, true );

	if( iVitesse > 0 ){
		iMD_S1 = MAX_PWM;
		iMD_S2 = 0;
		iMD_V = iVitesse;

	}else{
		iMD_S1 = 0;
		iMD_S2 = MAX_PWM;
		iMD_V = -iVitesse;	
	}
}
 
/* va donner modifier les variables globales moteurs en fonction de la vitesse et le delta entre les 2 cotes.
 * delta positif : tourne a droite
 * delta negatif : tourne a gauche
 *
 * vitesse 	: -MAX_PWM a +MAX_PWM
 * delta	: -MAX_PWM*2 a +MAX_PWM*2
 *
 *									 MG	            MD
 *-MAX_PWM					  0		  v 	 V      v   MAX_PWM
 * <-|------------------------|-------|######|######|----|->
 *									V-delta/2	V+delta/2
 *
 * Si delta est negatif, on inverse MG et MD.
 */
void command_moteurs( int iVitesse, int iDelta ){
	boolean bDelta_plus = true; // Detecter le sens du delta
	int iBorneDroite = 0;		// Indicateur pour la borne de gauche
	int iBorneGauche = 0;		// Indicateur pour la borne de droite
	int iDeltaDemi = 0;			// Detient le demi delta (positif)
	
	if( bDebug ){
		Serial.print("iVitesse : ");
		Serial.println( iVitesse );
	}
	//////////////////////////
	//// Calibrage des donnees
	// La vitesse
	if( iVitesse > MAX_PWM ){
		iVitesse = MAX_PWM;
		// Serial.print(" est sup a MAX_PWM ");
		
	}else if( iVitesse < -MAX_PWM ){
		iVitesse = -MAX_PWM;
		// Serial.print(" est inf a -MAX_PWM ");
	}
	// si delta negatif, on le marque et on le remet positif.
	if( iDelta < 0 ){
		bDelta_plus = false;
		iDelta = -iDelta;
	}
	// Verif qu'il ne depasse pas la valeur max.
	if( iDelta > MAX_PWM )
		iDelta = MAX_PWM;
	// Calcul du 1/2 delta.
	// iDeltaDemi = iDelta/2;
		
		
	///////////////////////////
	//// Etalonnage des Bornes.
	// On verifie que l'on ne depasse pas de la borne maximale (soit le PWM Maximal).
	if( iVitesse + iDelta >  MAX_PWM ){
		iBorneDroite = MAX_PWM;
		iBorneGauche = iBorneDroite - iDelta;
		// Serial.print(" et iVitesse + iDeltaDemi est sup a MAX_PWM ");
	
	// On verifie qu'on ne depasse pas la borne minimale.
	}else if( iVitesse - iDelta <  -MAX_PWM ){
		iBorneGauche = -MAX_PWM;
		iBorneDroite = iBorneGauche + iDelta;
		// Serial.print(" et iVitesse - iDeltaDemi est inf a -MAX_PWM ");
	
	// si rien ne depasse
	}else{
		// iBorneGauche = iVitesse - iDeltaDemi;
		// iBorneDroite = iVitesse + iDeltaDemi;
		iBorneGauche = iVitesse + iDelta;
		iBorneDroite = iVitesse - iDelta;
	}
	
	
	///////////////////////////////////////////////
	//// Affectation aux variables globales MOTEURS
	// ... si iDelta positif : Borne Gauche = MG et Borne Droite = MD
	// ... sinon c'est l'inverse.
	// Serial.print(" soit iBorneGauche : ");
	// Serial.print(iBorneGauche);
	// Serial.print(" soit iBorneDroite : ");
	// Serial.print(iBorneDroite);
	// Serial.println(" ");
	
	if( bDelta_plus ){
		affect_MG( iBorneDroite );
		affect_MD( iBorneGauche );
	//Serial.println(" Delta Positif ");
	}else{
		affect_MD( iBorneDroite );
		affect_MG( iBorneGauche );	
	//Serial.println(" Delta Negatif ");
	}
	
	ordre_moteur();
}







//////////////////////////////////////
//		Gestion des Servos			//
//////////////////////////////////////

/////////////////////
//// Balai
/* void move_balai( int iAngle ){
	tlc_setServo( S_BALAI, iAngle);
	Tlc.update();
	delay( S_BALAI_DELAY );
} */
// Pour test : Servo branche sur le PIN9
void move_balai( int iAngle ){
	int iDeltaAngle;
	float iNbPas;
	// On demande au servo de bouger
	servo_balai.write( iAngle );
	
	// On attend qu'il soit en place.
	// ... Dépend de l'emplacement initial.
	iDeltaAngle = iBalai_pos - iAngle;
	if( iDeltaAngle < 0 )
		iDeltaAngle = -iDeltaAngle;
	// comme le S_BALAI_DELAY est calculé par rapport au S_BALAI_PAS
	iNbPas = iDeltaAngle / S_BALAI_PAS;
	
	// Pour Debug : 
/* 	Serial.print( " (iNbPas : " );
	Serial.print( iNbPas );
	Serial.print( ") " ); */
	
	// On attend autant de temps que de pas a faire ... iNbPas étant un float, ceci devrai faire le rapport meme si iAngle n'est pas un multiple de S_BALAI_DELAY
	delay( S_BALAI_DELAY * iNbPas );
		
	// et on mémorise la derniere position
	iBalai_pos = iAngle;
}

/* va générer un tableau avec les differents angles a checker. Par exemple 60é, 90é et 120é par rapport a un angle de 90é.
 * .... la question reste sur Quel referentiel prendre pour les à ???
 * - Ref Voiture, Ref Servo, Ref Geographique ... avec la boussole...
 * Le plus simple va etre la Ref Servo.
 *	
 */
void gen_angles( int iAngle, int aAngles[ S_BALAI_NBCHECK ] ){
	// int aAngles[ S_BALAI_NBCHECK ]; // detient les NBCHECK angles a checker
	int bIsImpair;
	
	////////////////////////////
	//// Calibrage des donnees
	
	//// Il faut savoir si la derniere borne ne serait pas hors 0-180
	
	// S_BALAI_NBCHECK : Pair ou impair ?
	// Car si on est impair ... le chiffre du milieu va tomber pile sur le iAngle, sinon le iAngle va se retrouver tout juste entre 2 positions...!
	bIsImpair = S_BALAI_NBCHECK % 2;
	// Combien de check a gauche... et donc combien a droite?
	int iNbPart = S_BALAI_NBCHECK / 2; 	// Comme on cast en INT : on n'aura qu'un entier. 
										// 3/2=1 : Il n'y a bien qu'un item sur la gauche ou la droite lorsqu'on demande 3 Checks
										// 4/2=2 : Il y a bien 2 items sur la gauche ou la droite.
	
	// Serial.print( "bIsImpair ? ");
	// Serial.print( bIsImpair);
	// Serial.print( " ... Demi NBCHECK ? ");
	// Serial.println( iNbPart);
	
	if( bIsImpair == 0 ){ // oui ca aurait été mieux d'avoir un == TRUE ... mais je me suis raté au début!
		if( iAngle - (S_BALAI_PAS /2) - ((iNbPart-1)*S_BALAI_PAS) < S_BALAI_ANGLE_MIN ){ // (iNbPart-1) car le S_BALAI_PAS /2 compte deja pour une borne ;)
			// Comme on a depasse le minimum : on part de 0 et on fait X* un pas de Yé soit S_BALAI_NBCHECK * un pas de S_BALAI_PASé
			int pos = S_BALAI_ANGLE_MIN;
			for( int i=0; i < S_BALAI_NBCHECK; i++ ){
				aAngles[i] = pos;
				pos += S_BALAI_PAS;
			}
		}else if( iAngle + (S_BALAI_PAS /2) + ((iNbPart-1)*S_BALAI_PAS) > S_BALAI_ANGLE_MAX ){
			// Comme on a depasse le maximum : on part du max et on fait X* un pas de -Yé soit S_BALAI_NBCHECK * un pas de -S_BALAI_PASé
			int pos = S_BALAI_ANGLE_MAX;
			for( int i = S_BALAI_NBCHECK -1; i >= 0 ; i-- ){
				aAngles[i] = pos;
				pos -= S_BALAI_PAS;
			}
			
		}else{
			int pos = iAngle - (S_BALAI_PAS /2) - ((iNbPart-1)*S_BALAI_PAS); 
			for( int i=0; i < S_BALAI_NBCHECK; i++ ){
				aAngles[i] = pos;
				pos += S_BALAI_PAS;
			}			
		}
	}else{
		if( iAngle - (iNbPart*S_BALAI_PAS) < S_BALAI_ANGLE_MIN ){
			// Comme on a depasse le minimum : on part de 0 et on fait S_BALAI_NBCHECK * un pas de S_BALAI_PASé
			int pos = S_BALAI_ANGLE_MIN;
			for( int i=0; i < S_BALAI_NBCHECK; i++ ){
				aAngles[i] = pos;
				pos += S_BALAI_PAS;
			}			
		}else if( iAngle + (iNbPart*S_BALAI_PAS) > S_BALAI_ANGLE_MAX ){
			// Comme on a depasse le maximum : on part du max et on fait S_BALAI_NBCHECK * un pas de -S_BALAI_PASé
			int pos = S_BALAI_ANGLE_MAX;
			for( int i = S_BALAI_NBCHECK -1; i >= 0 ; i-- ){
				aAngles[i] = pos;
				pos -= S_BALAI_PAS;
			}			
		}else{
			int pos = iAngle - (iNbPart*S_BALAI_PAS);
			for( int i=0; i < S_BALAI_NBCHECK; i++ ){
				aAngles[i] = pos;
				pos += S_BALAI_PAS;
			}			
		}
	}
} 

/* va lancer une mesure sur les angles récupéré par gen_angles() et retourner un tableau de mesure.
 *
 */
void check_angles( int iAngle, int aAngles[ S_BALAI_NBCHECK ], int aMesures[ S_BALAI_NBCHECK ], boolean bSpeedCheck){
	boolean bOrderAsc = true; // par defaut, on lit le tableau d'angles du 0 vers le S_BALAI_NBCHECK : Order Ascendant
	int iDiffDebut = 0;
	int iDiffFin = 0;
	
	// Récupération des angles à checker
	gen_angles( iAngle, aAngles );
	
	// Optimisation du déplacement (que le servo aille à la position la plus proche ... et non à la 1ere position du tableau.)
	// En fait, on va regarder par quel coté on attaque le tableau. car il n'est pas intéréssant de partir d'un offset en plein milieu pour revenir a la fin/début du tableau.
	iDiffDebut = iBalai_pos - aAngles[0];
	if( iDiffDebut < 0 )
		iDiffDebut = -iDiffDebut;
	iDiffFin = iBalai_pos - aAngles[ S_BALAI_NBCHECK-1 ];
	if( iDiffFin < 0 )
		iDiffFin = -iDiffFin;
	
	if( iDiffDebut > iDiffFin )
		bOrderAsc = false;
	
	if( bOrderAsc ){
		// Pour chacun des angles
		for( int i=0; i < S_BALAI_NBCHECK; i++ ){
			// On déplace le balai
			move_balai( aAngles[i] );
			// on effectue un relevé
			if( bSpeedCheck )
				aMesures[i] = get_distance();
			else
				aMesures[i] = get_moderated_distance();
				
			// Pour Test
			// Serial.print( aAngles[i] );
			// Serial.print( ":" );
			// Serial.print( aMesures[i] );
			// if( i != S_BALAI_NBCHECK -1 )
				// Serial.print( " | " );
		}
	}else{
		// Pour chacun des angles
		for( int i=S_BALAI_NBCHECK-1; i >= 0; i-- ){
			// On déplace le balai
			move_balai( aAngles[i] );
			// on effectue un relevé
			if( bSpeedCheck )
				aMesures[i] = get_distance();
			else
				aMesures[i] = get_moderated_distance();
			
			// Pour Test
			// Serial.print( aAngles[i] );
			// Serial.print( ":" );
			// Serial.print( aMesures[i] );
			// if( i != 0 )
				// Serial.print( " | " );
		}
	}
	// Serial.print( " : " );
}

int check_direction( int iDegresCible, int aAngles[ S_BALAI_NBCHECK ], int aMesures[ S_BALAI_NBCHECK ], boolean bSpeedCheck ){
	// On récupère la différence en notre cap et le cap à suivre.
	int iDegresDiff = compass_delta( iDegresCible );
	
	// comme le capteur est inversé :
	 iDegresDiff = -iDegresDiff;
	
	if( bDebug ){
		Serial.print( "Direction demandee : " );
		Serial.print( iDegresCible );
		Serial.print( " Delta avec la boussole: " );
		Serial.print( iDegresDiff );
		Serial.print( " Check Angles : " );
		Serial.print( dtos(iDegresDiff) );
	}

	if( iDegresDiff <= 90 && iDegresDiff >= -90){
		check_angles( dtos( iDegresDiff ), aAngles, aMesures, bSpeedCheck);
	}else if( iDegresDiff > 90 ){
		check_angles( dtos( 90 ), aAngles, aMesures, bSpeedCheck);
	}else if( iDegresDiff < -90 ){
		check_angles( dtos( -90 ), aAngles, aMesures, bSpeedCheck);
	}

	return -iDegresDiff;
}



/////////////////////
//// Tourelle
/* Fonction qui va donner les ordres au servo
 *
 */
void ordre_servo( int pos, int servo ){
	switch( servo ){
		case S_HORI_NAME : 
			servo_hori.write( pos );
			break;
		case S_VERT_NAME :
			servo_vert.write( pos );
			break;
        case 3 :
            servo_balai.write( pos );
            break;
         
		default :
            //raz();
            break;
  }
}





//////////////////////////////////////
//		Gestion des Capteurs		//
//////////////////////////////////////

/////////////////////////////////
//// Capteur UltraSon

/* va faire un releve de distance et le retourne (en cm)
 */
float get_distance(){
    float fDuration, fDistance;
	
	// Activation du capteur
    digitalWrite( US_TRIG, HIGH );
    delayMicroseconds( 1000 );
    digitalWrite( US_TRIG, LOW );

	
	// Lecture du temps de retour
    fDuration = pulseIn( US_ECHO, HIGH, US_TIMEOUT );
    // fDuration = pulseIn( US_ECHO, HIGH );
	
	// Convertion en distance
    fDistance = ( fDuration/2 ) * 0.03495;

    return fDistance;
}
/* va faire un releve de 5(US_MODER) distances et en retourne la moyenne(en cm)
 * On va retirer le min et le max et faire la moyenne du reste
 */
float get_moderated_distance(){
	float aDist[ US_MODER ];
	float fAverage = 0;
	float fCurrDist = 0;
	float iMin = 99999;
	float iMax = 0;
	int iOffsetMin = 0;
	int iOffsetMax = 0;
	// On prélève les X mesures et on relève les indices des valeurs min et max
	for( int i=0; i < US_MODER; i++ ){
		aDist[i] = get_distance();
		if( aDist[i] > iMax ){
			iMax = aDist[i];
			iOffsetMax = i;
		}if( aDist[i] < iMin ){
			iMin = aDist[i];
			iOffsetMin = i;
		}
	}

	// On fait la sommes des valeurs hors min et max
	for( int i=0; i < US_MODER; i++ ){
		if( i != iOffsetMin && i != iOffsetMax )
			fAverage += aDist[i];	
	}

	// On divise
	if( US_MODER - 2 > 0 )
		fAverage /= US_MODER;
	return fAverage;
}





//////////////////////////////////
//// Boussole / Compass

/* Initialisation des composants pour la boussole
 *
 */
int init_compass(){
  Wire.begin(); // Start the I2C interface.

  compass = HMC5883L(); // Construct a new HMC5883 compass.
    
  int error = compass.SetScale(1.3); // Set the scale of the compass....Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  
  return error;
}

/* va retourner l'indicateur de position horizontale : X
 *
 */
float get_Orientation(){
	// Retrive the raw values from the compass (not scaled).
	MagnetometerScaled scaled = compass.ReadScaledAxis();

	// Calculate heading when the magnetometer is level, then correct for signs of axis.
	float heading = atan2(scaled.YAxis, scaled.XAxis);
	
	// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
	// Find yours here: http://www.magnetic-declination.com/
	// Mine is: 2? 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
	// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
	//a toulouse : 59é8' soit 1.0320713643 radians
	float declinationAngle = 1.0321;
	// float declinationAngle = 0.0457;
	heading += declinationAngle;
	
	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2*PI;
    
	// Check for wrap due to addition of declination.
	if(heading > 2*PI)
		heading -= 2*PI;
   
	// Convert radians to degrees for readability.
	float headingDegrees = heading * 180/M_PI; 

	return headingDegrees;
}

/* va retourner l'indicateur de position horizontale : X en faisant une moyenne sur C_MODER mesures
 *
 */
float get_OrientationMoy(){
	float fMesure = 0;
	
	for( int i=0; i < C_MODER; i++ )
		fMesure += get_Orientation();
	
	fMesure /= C_MODER;
	
	return fMesure;
}

/* va retourner la différence en ° (-180 à +180) entre la position actuelle et la position cible
 *	Retournera 0 si le delta est inférieur C_ERROR_MARGE
 */
int compass_delta( int iDegresCible ){
	boolean  bSens;
	
	// Récup de la position actuelle donnée par la boussole
	float iPosActuelle = get_OrientationMoy();
	// Différence entre la position actuelle et la position demandée.
	int iDiffPos =  iDegresCible - (int)iPosActuelle;
	// Detection du sens //plus facile a traiter par la suite avec un iDiff forcément positif.
	if( iDiffPos < 0 ){
		bSens = false;
		iDiffPos = -iDiffPos;
	}else
		bSens = true;
	// Changement de reférentiel au lieu de 0/+360é -> -180/+180
	if( iDiffPos >= 180 ){
		iDiffPos -= 180;
		iDiffPos = 180 - iDiffPos;
	}
	// Pas de modif de trajectoire si le delta est trop petit
	if( iDiffPos <= C_ERROR_MARGE ){
		iDiffPos = 0;
	}
	// On redonne le sens : 
	if( !bSens ){
		iDiffPos = -iDiffPos;
	}
	
	return iDiffPos;
}



////////////////////////////////////////
// Amperemetre
int get_amperage(){
	return analogRead( AMP );
}


//////////////////////////////////////
//			Pilotage				//
//////////////////////////////////////

/////////////////////////////////
//// Trajectoires
 
/* va faire tourner les moteurs à iVitesse et faire pivoter le mobil de -180°(demi-tour gauche) à +180° (demi-tour droite)
 * 
 * On va prendre la position actuelle, qu'on va comparer avec notre position en cours
 * On retrouvera donc la différence entre 2 degrés...
 * Mais si la cible est 350é et que je suis à 30° ... je vais avoir une diff de 320° donc + qu'un demi tour!
 * Il me faut obliratoirement transcrire en degrés relatif au mobil pour pouvoir retrouver le iDelta
 *  
 * iDiffPos est compris entre 0 et 359.99
 * Logiquement, on ne donnea jamais un ordre suppérieur à 180° puisque ca impliquerait qu'il faudrait faire plus que demi-tour
 * car dans ce cas la, partir dans l'autre sens serait plus rapide.
 */
void command_moteurs_compass( int iVitesse, int iDegresCible ){
	int iDeltaMoteur;
	long long lIntermediaire;
	
	// Récupération de la différence entre le cap actuel et le cap a prendre.
	int iDiffPos = compass_delta( iDegresCible );
	
	// mise a l'echelle par rapport au moteurs : actuellement 0/+180, moteurs : 0/+MAX_PWM*2
	// iDeltaMoteur = iDiffPos * 8190 / 180; //!\\ OVERFLOOOOOOOW ... Arduino prend le type le plus grand... donc ici un INT. hors iDiff allant jusqu'a 180 : 180*8190=1 474 200 et un INT OVERFLOW a 32 767... et pour info ... des que iDiff > 4 : Overflow! :)
	lIntermediaire = (long long)iDiffPos * (MAX_PWM*2); 	////!\\ le fait de caster en unsigned long permet de faire un calcul avec ce type et donc ne pas etre overflow.
														// Peut-etre iDeltaMoteur = (unsigned long)iDiffPos * MAX_PWM*2 / 180; fonctionnerait...?
	iDeltaMoteur = lIntermediaire / 180;
	
	
 	if( bDebug ){
		Serial.print("PosActuelle : ");
		Serial.print(get_Orientation());
		Serial.print(" Cible : ");
		Serial.print(iDegresCible);
		Serial.print(" Vitesse : ");
		Serial.print(iVitesse);
		Serial.print(" DeltaDegres : ");
		Serial.print(iDiffPos);
		Serial.print(" DeltaMoteur : ");
		Serial.println(iDeltaMoteur);
	}
	command_moteurs( iVitesse, iDeltaMoteur );
}

/* va comparer les aMesures/aAngles au cone d'action et donc retourner l'action la plus appropriée aux éléments dernierement detecté.
 *
 */
void cone_action( int* aAngles, int* aMesures, int iDegresDelta ){

/* 	int mesure_ref = 30; //
	
	//// Check des angles, puis check des mesures.
	// Check 0°
	if( aAngles[i] - iDegresDelta < 15 ){
		if( aMesures[i] < mesure_ref / 4 ){
			// Vitesse = MIN_PWM  (et Tourne--)
			;
		}else if( aMesures[i] < mesure_ref / 2 ){
			// Vitesse = MIN_PWM + MIN_PWM/2 (et tourne-)
			;
		}
	}
	// Check 30°
	else if( aAngles[i] - iDegresDelta < 45 ){
		if( aMesures[i] < mesure_ref / 4 ){
			// Vitesse = -MIN_PWM et Tourne gauche+ en marche arriere 
			;
		}else if( aMesures[i] < mesure_ref / 2 ){
			// Vitesse = MIN_PWM + MIN_PWM/2 et tourne droite
			;
		}	
	}
	// Check 60°
	else if( aAngles[i] - iDegresDelta < 75 ){
		if( aMesures[i] < mesure_ref / 4 ){
			// Vitesse = -MIN_PWM et Tourne gauche++ en marche arriere 
			;
		}else if( aMesures[i] < mesure_ref / 2 ){
			// Vitesse = MIN_PWM et tourne droite+
			;
		}else if( aMesures[i] < mesure_ref ){
			// Vitesse = MIN_PWM + MIN_PWM/2 et tourne droite
			;
		}		
	}
	// Check 90°
	else if( aAngles[i] - iDegresDelta < 105 ){
		if( aMesures[i] < mesure_ref / 4 ){
			// Vitesse = -MIN_PWM et Tourne gauche++ en marche arriere 
			;
		}else if( aMesures[i] < mesure_ref / 2 ){
			// Vitesse = 0 et tourne++
			;
		}else if( aMesures[i] < mesure_ref ){
			// Vitesse = MIN_PWM + MIN_PWM/2 et tourne++
			;
		}else if( aMesures[i] < mesure_ref * 2 ){
			// Vitesse = MAX_PWM et tourne+
			;
		}else{
			// Vitesse = MAX_PWM et tourne
			;
		}		
	}
	// Check 120°
	else if( aAngles[i] - iDegresDelta < 135 ){
		if( aMesures[i] < mesure_ref / 4 ){
			// Vitesse = -MIN_PWM et Tourne droite++ en marche arriere 
			;
		}else if( aMesures[i] < mesure_ref / 2 ){
			// Vitesse = MIN_PWM et tourne gauche+
			;
		}else if( aMesures[i] < mesure_ref ){
			// Vitesse = MIN_PWM + MIN_PWM/2 et tourne gauche
			;
		}		
	}
	// Check 150°
	else if( aAngles[i] - iDegresDelta < 165 ){
		if( aMesures[i] < mesure_ref / 4 ){
			// Vitesse = -MIN_PWM et Tourne droite+ en marche arriere 
			;
		}else if( aMesures[i] < mesure_ref / 2 ){
			// Vitesse = MIN_PWM + MIN_PWM/2 et tourne gauche
			;
		}		
	}
	// Check 180°
	else if( aAngles[i] - iDegresDelta >= 165 ){
		if( aMesures[i] < mesure_ref / 4 ){
			// Vitesse = MIN_PWM  (et Tourne--)
			;
		}else if( aMesures[i] < mesure_ref / 2 ){
			// Vitesse = MIN_PWM + MIN_PWM/2 (et tourne-)
			;
		}	
	}
 */
 }

/* Par rapport à un cap donné, va modifier ce cap par rapport aux relevés UltraSon
 * va retourner le nouveau cap.
 * //!\\ 	le iDegresCible == au CAP (donc de 0 à 359 par rapport au NORD)
 * 			Le check_angles == Direction du Servo => 0° Servo = gauche du drone, 180° = droite du drone
 * 			... Il va falloir trouver un référentiel commun!
 *		D'ou l'utilité des 2 fonctions : stod et dtos qui va transcrire les "°servo to °drone" et inversement
 */
int traj_modifier( int iDegresCible ){
	// En Degrés, la correction apportée à iDegresCible
	int iCorrection = 0;
	
	// On réalise un relevé
	int aAngles[ S_BALAI_NBCHECK ];
	int aMesures[ S_BALAI_NBCHECK ];
	boolean bSpeedCheck = false;
	
	int iDegresDelta = check_direction( iDegresCible, aAngles, aMesures, bSpeedCheck ); 
	
	// On check le relevé // On admet que le S_BALAI_NBCHECK == 3
	// y a-t-il des valeurs "blocante"? sur le cap du milieu?
	if( aMesures[1] < 15 ){
		// Oui alors on check de quel coté on va tourner
		// y a-t-il des valeurs "blocante"? sur les caps gauche ou droite?
		if( aMesures[0] > 15 && aMesures[2] > 15){
			// Non, Ou y a-t-il le plus de place?
			if( aMesures[0] > aMesures[2] ){
				// Tourne à gauche
				iCorrection -= S_BALAI_PAS;
			}else{
				// Tourne à droite
				iCorrection +=  S_BALAI_PAS;
			}
		}
		// a gauche? que ca bloque?
		else if( aMesures[0] <= 15 ){
			// Tourne à droite
			iCorrection += S_BALAI_PAS;
		}
		// a droite? que ca bloque?
		else if( aMesures[2] <= 15 ){
			// Tourne à gauche
			iCorrection -= S_BALAI_PAS;
		}
	}
	// Pas de valeur blocante en face, mais un obstacle tout de meme.
	else if( aMesures[1] < 30 ){
		// Alors on check de quel coté on va tourner
		// y a-t-il des valeurs "blocante"? sur les caps gauche ou droite?
		if( aMesures[0] > 15 && aMesures[2] > 15){
			// Non, Ou y a-t-il le plus de place?
			if( aMesures[0] > aMesures[2] ){
				// Tourne à gauche
				iCorrection -= S_BALAI_PAS/2;
			}else{
				// Tourne à droite
				iCorrection +=  S_BALAI_PAS/2;
			}
		}
		// a gauche? que ca bloque?
		else if( aMesures[0] <= 15 ){
			// Tourne à droite
			iCorrection += S_BALAI_PAS;
		}
		// a droite? que ca bloque?
		else if( aMesures[2] <= 15 ){
			// Tourne à gauche
			iCorrection -= S_BALAI_PAS;
		}
	}
	
	
	iDegresCible += iCorrection;
	if( bDebug ){
		Serial.print( " Correction : " );
		Serial.print( iCorrection );
		Serial.print( " Nvl Cible : " );
		Serial.println( iDegresCible );
	}
	return iDegresCible;
}







//////////////////////////////////////
//			Fct Usuelles			//
//////////////////////////////////////

/* Transcript les ° servo en ° drone
 * les ° servo allant de 0 à +180
 * les ° drones de -90 à +90
 * ... pour le meme angle : 0° servo = -90° drone
 */
 int stod(int iDegresServo ){
	iDegresServo -= 90;
	return iDegresServo;
 }
 
 /* Transcript les ° drone en ° servo
 * les ° servo allant de 0 à +180
 * les ° drones de -90 à +90
 * ... pour un meme angle : 0° drone = 90° servo
 */
 int dtos(int iDegresDrone ){
	iDegresDrone += 90;
	return iDegresDrone;
 }