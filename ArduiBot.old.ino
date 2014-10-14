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
#define MAX_PWM		200		//4095
#define MIN_PWM		107


/////////////////////////
// Moteurs

// Moteurs Gauche
#define MG_S1   	2	//0	// Pin Moteurs Gauche Sens 1
#define MG_S2   	4	//1	// Pin Moteurs Gauche Sens 2
#define MG_V   		3	//2	// Pin Moteurs Gauche Vitesse
// Moteurs Droite
#define MD_S1   	7	//3	// Pin Moteurs Droite Sens 1
#define MD_S2   	8	//4	// Pin Moteurs Droite Sens 2
#define MD_V   		6	//5	// Pin Moteurs Droite Vitesse


/////////////////////////
// Servo-Moteur

// Servo BALAI
#define S_BALAI				9	//16	// Pin Servo Balai
#define S_BALAI_DELAY 		150	// Delai en ms, entre 2 mouvements du balai ... 0.20 sec/60é à 5v pour le S3010(0.23/60 pour le S3003) donc 0.1s/30é(0.12/30) donc ...100 ms (120ms)(à caler avec le S_BALAI_PAS)
#define S_BALAI_PAS 		30	// Nb de Degre entre 2 balayage.
#define S_BALAI_NBCHECK		3	// Nb de positions a checker.
#define S_BALAI_ANGLE_MIN	0	// Positions minimale
#define S_BALAI_ANGLE_MAX	180	// Positions maximale


/////////////////////////
// Capteurs

// Boussole / Compass
#define C_ERROR_MARGE		10 	// Nb de degrés d'erreur
#define C_MODER				3	// Nbvaleur relevé pour la moyenne

// Ultrason
#define US_TRIG		12		// Pin Triguer
#define US_ECHO		13		// Pin Echo
#define US_ALIM		11		// Pin Alim ... pour la phase de test ... sera sur une vrai alim par la suite.
#define US_TIMEOUT	25000	// TIMEOUT en Microsec. (Important car le script se bloque durant ce temps quand les obstacles sont trop loins)
#define US_MODER	3		// Combien de relevés pour faire une moyenne?
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



//////////////////////////////////////////
// 		Declaration des variables		//
//////////////////////////////////////////
boolean bDebug = true;


/////////////////////////
// Moteurs
int iCountMargeMG =	0; 	// compte le nombre de fois qu'on est en dessous du MIN_PWM pour le Moteur Gauche
int iCountMargeMD =	0; 	// compte le nombre de fois qu'on est en dessous du MIN_PWM pour le Moteur Droit
int COUNT_MARGE; 		// nombre de fois ou il sera toléré un < MIN_PWM ... initialisé dans le SETUP()

	
// Moteurs Gauche
int iMG_S1 = 		0;	// valeur du MG_S1 (MIN_PWM ou MAX_PWM)
int iMG_S2 = 		0;	// valeur du MG_S2 (MIN_PWM ou MAX_PWM)
int iMG_V = 		0;	// valeur du MG_V (de MIN_PWM a MAX_PWM)


// Moteurs Droite
int iMD_S1 = 		0;	// valeur du MD_S1 (MIN_PWM ou MAX_PWM)
int iMD_S2 = 		0;	// valeur du MD_S2 (MIN_PWM ou MAX_PWM)
int iMD_V = 		0;	// valeur du MD_V (de MIN_PWM a MAX_PWM)




/////////////////////////
// Servo-Moteur

// Servo BALAI
int iBalai_pos =			0; 	// Position du servo Balai
int iBalai_TimeStamp =		0; 	// Reference de temps qui sera compare a S_BALAI_DELAY
Servo servo_balai;
// Pour Test
int iInitPos = 60; // position de depart du servo balai
int iFinPos = 120; // position de fin du servo balai


/////////////////////////
// Capteurs

// Boussole / Compass.
HMC5883L compass;



////////////////////////
// Pilotage

int iDirection = 100; 	// Direction, en ° (0 et +360 par rapport au Nord) que doit prendre le drone.




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
	Serial.begin(9600);
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

	// pour test
	servo_balai.attach( S_BALAI );  	// attache le servo balai au pin  
	Serial.println( "Declaration Servo" );

	// Boussole / Compass
	Serial.println( "Connexion a la boussole" );
	init_compass();
	// if( init_compass() != 0 )
		// Serial.println(compass.GetErrorText(error));
	
	
	
	// servo_balai.write( iInitPos );
	
	
	Serial.println( "Ready !" );
	Serial.println( "" );

}



void loop()
{

	/* test de modification de trajectoire */
	traj_modifier( iDirection );

	/* Test Compass */
//	Serial.println( get_Orientation() );
	
	
	/* test Compass + Moteurs */
	// command_moteurs_compass( 0, iDirection );
	
	
	
/* 	Test UltraSon

	float iMesure = 0;
	float iMesureMoy = 0;
	
	digitalWrite( US_ALIM, HIGH );
	iMesure = get_distance();
	iMesureMoy = get_moderated_distance();

	Serial.print( "Mesure Simple : " );
	Serial.println( iMesure );
	
	Serial.print( "Mesure Moyenne : " );
	Serial.println( iMesureMoy );
	
 */
/* 	Test UltraSon + Balai

	digitalWrite( US_ALIM, HIGH );

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
		
 		// for( int j=0; j < S_BALAI_NBCHECK ; j++){
			// Serial.print( aAngles[j] );
			// if( j != S_BALAI_NBCHECK -1 )
				// Serial.print( " | " );
		// }
		Serial.println( "" ); 
		delay( 2000 );
	} 
	Serial.println( "" );
 */	

	if( bDebug ){
		Serial.println("");
		delay( 2000 );
	}
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
	iVitesse = regul_vitesse( iVitesse , false );
		
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
	iVitesse = regul_vitesse( iVitesse, true );

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
 * vitesse 	: -4095 a 4095
 * delta	: -8190 a 8190
 *
 *									 MG	            MD
 *-4095						  0		  v 	 V      v   4095
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
	
	
	Serial.print("iVitesse : ");
	Serial.print( iVitesse );
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
	if( iDelta > MAX_PWM*2 )
		iDelta = MAX_PWM*2;
	// Calcul du 1/2 delta.
	iDeltaDemi = iDelta/2;
		
		
	///////////////////////////
	//// Etalonnage des Bornes.
	// On verifie que l'on ne depasse pas de la borne maximale (soit le PWM Maximal).
	if( iVitesse + iDeltaDemi >  MAX_PWM ){
		iBorneDroite = MAX_PWM;
		iBorneGauche = iBorneDroite - iDelta;
		// Serial.print(" et iVitesse + iDeltaDemi est sup a MAX_PWM ");
	
	// On verifie qu'on ne depasse pas la borne minimale.
	}else if( iVitesse - iDeltaDemi <  -MAX_PWM ){
		iBorneGauche = -MAX_PWM;
		iBorneDroite = iBorneGauche + iDelta;
		// Serial.print(" et iVitesse - iDeltaDemi est inf a -MAX_PWM ");
	
	// si rien ne depasse
	}else{
		iBorneGauche = iVitesse - iDeltaDemi;
		iBorneDroite = iVitesse + iDeltaDemi;
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
/* va faire un releve de 3(US_MODER) distances et en retourne la moyenne(en cm)
 */
float get_moderated_distance(){
	float iAverage = 0;
	// On additionne
	for( int i=0; i < US_MODER; i++ ){
		iAverage += get_distance();
	}
	// On divise
	iAverage /= US_MODER;
	return iAverage;
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

/* va retourner la différence en à (-180 à +180) entre la position actuelle et la position cible
 *	Rertournera 0 si le delta est inférieur C_ERROR_MARGE
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




//////////////////////////////////////
//			Pilotage				//
//////////////////////////////////////

/////////////////////////////////
//// Trajectoires

/* va comparer les aMesures/aAngles au cone d'action et donc retourner l'action la plus appropriée aux éléments dernierement detecté.
 *
 */
void cone_action( int* aAngles, int* aMesures, int iDegresDelta ){

	int mesure_ref = 30; //
	
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