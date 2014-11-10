#include "ConnectoRF.h"

////// Trame d'echange de connection
// max 4 294 967 296... valeur d'un unsigned long... mais apparemment c'est bien plus bas...
//Plage reservee aux commandes internes
unsigned long MAXRANGECMD =	100; // jusqu'au maxi d'un unsigned long

unsigned long HELLOIAMCTRL = 	5; //10
unsigned long HELLOCTRL = 		6;//15

unsigned long AREUOK = 			7; //20
unsigned long IMOK  =			8; //25

unsigned long STILLHERE =		9; //30
unsigned long IAMHERE =			10; //35

ConnectoRF::ConnectoRF(){
	this->init( );
}

void ConnectoRF::init(){
	
	this->iStatus = SEARCHING;
	this->iUnsuccessTry = 0;
	
	this->ulTimePool = 0;
	this->ulTimeMaxSpeed = 0;
		
	this->aiMasterCmd[0] = HELLOIAMCTRL;
	this->aiMasterCmd[1] = AREUOK;
	this->aiMasterCmd[2] = STILLHERE;
	
	this->aiSlaveCmd[0] = HELLOCTRL;
	this->aiSlaveCmd[1] = IMOK;
	this->aiSlaveCmd[2] = IAMHERE;
	
	this->bMaster = true;
	
	this->bDebug = false;
}

void ConnectoRF::setPins( int pinRec, int pinTra ){
	this->mySwitchRec = myRCSwitch();
	this->mySwitchTra = myRCSwitch();

	this->mySwitchTra.enableTransmit(pinTra);
	this->mySwitchRec.enableReceive(pinRec);
}

bool ConnectoRF::isSlaveCmd( unsigned long ulMessage ){
	bool bFinded = false;
	
	for( int i=0; i< sizeof( aiSlaveCmd ) ; i++ ){
		if( aiSlaveCmd[i] == ulMessage ){
			bFinded = true;
		}
	}	
	return bFinded;
}
bool ConnectoRF::isMasterCmd( unsigned long ulMessage ){
	bool bFinded = false;
	
	for( int i=0; i< sizeof( aiMasterCmd ); i++ ){
		if( aiMasterCmd[i] == ulMessage ){
			bFinded = true;
		}
	}		
	return bFinded;
}

unsigned long ConnectoRF::getAvailableReceivedControl(){
	unsigned long ulReceived = 0;
	// s'il y a des datas
	if (this->mySwitchRec.available()) {
		// on les récupère
		ulReceived = this->mySwitchRec.getReceivedValue();
		
		// s'il s'agit d'un des elements de controle
		if( this->bMaster ){
			if( this->isSlaveCmd( ulReceived ) ){
				if( this->bDebug ){ Serial.print( "Master Receved Slave cmd : " ); Serial.println( ulReceived ); }
				// on reset les tentatives
				this->iUnsuccessTry = 0;
				// on vide le buffer
				this->mySwitchRec.resetAvailable();
			}/* else{
 				if( this->isMasterCmd( ulReceived ) )
					// on vide le buffer
					this->mySwitchRec.resetAvailable(); 
				// sinon on retourne 0
				ulReceived = 0;
			} */
		}else{
			if( this->isMasterCmd( ulReceived ) ){
				if( this->bDebug ){ Serial.print( "Slave Receved Master cmd : " ); Serial.println( ulReceived ); }
				// on reset les tentatives
				this->iUnsuccessTry = 0;
 				// on vide le buffer
				this->mySwitchRec.resetAvailable(); 
			}/* else{
				if( this->isSlaveCmd( ulReceived ) )
					// on vide le buffer
					this->mySwitchRec.resetAvailable(); 
				// sinon on retourne 0
				ulReceived = 0;
			} */
		}
	}
	
	return ulReceived;
}

int ConnectoRF::connectionMgr(){
	int iStatut;
	if( this->bMaster )
		iStatut = this->masterConnection();
	else
		iStatut = this->slaveConnection();
		
	return iStatus;
}

int ConnectoRF::masterConnection(){
	unsigned long ulReceived = 0;
	unsigned long ulTransmit = 0;
	
	// on récupère les datas si elles sont de type de control
	ulReceived = this->getAvailableReceivedControl();

	// si on a recu un message du slave ou si on a passe le temps de pooling
	if( isSlaveCmd( ulReceived ) || (millis() - this->ulTimePool) > POOLTIME  ){
		// on reset notre timestamp
		this->ulTimePool = millis();
		
		// si on n'est pas connecte
		if( this->iStatus != CONNECTED && this->iStatus != MAYLOSTED ){
			// si reponse au HELLOIAMCTRL
			if( ulReceived == HELLOCTRL ){

				// on change le statut
				this->iStatus = FINDED;

				// On prepare la var de transmit
				ulTransmit = AREUOK;
				if( this->bDebug ){ Serial.print( "Are you ok? : "  ); Serial.print( ulTransmit  ); }
				
				// on l'envoie
				this->mySwitchTra.send(ulTransmit);
				
				// On comptabilise les essais
				this->iUnsuccessTry ++;
				
				if( this->bDebug ){ Serial.print( " Try #"  ); Serial.println( this->iUnsuccessTry  ); }

			// si reponse au AREUOK
			}else if( ulReceived == IMOK ){
				this->iStatus = CONNECTED;
				//if( !this->bDebug ){ Serial.println( "[Connected]" ); }

			// sinon on cherche...
			}else{
				// On prépare la var de transmit
				ulTransmit= HELLOIAMCTRL;
				if( this->bDebug ){ Serial.print( "Hello i'm controler : "  ); Serial.print( ulTransmit  ); }
					
				// on l'envoie
				this->mySwitchTra.send(ulTransmit);
				
				// On comptabilise les essais
				this->iUnsuccessTry ++;
				
				if( this->bDebug ){ Serial.print( " Try #"  ); Serial.println( iUnsuccessTry  ); }
				
			}
			
		// sinon on teste la connexion
		}else{
			if( ulReceived != IAMHERE ){
				if( this->iUnsuccessTry < MAXUNSUCESSTRY ){
					// On prépare la var de transmit
					ulTransmit = STILLHERE;
					if( this->bDebug ){ Serial.print( "Are you still here? : "  ); Serial.print( ulTransmit  ); }
					
					// on l'envoie
					this->mySwitchTra.send(ulTransmit);
					
					// On comptabilise les essais
					this->iUnsuccessTry ++;
					if( this->bDebug ){ Serial.print( " Try #"  ); Serial.println( this->iUnsuccessTry  ); }
					
					// s'il s'agit de la 1er tentative, nous sommes connecté
					if( this->iUnsuccessTry == 1){
						this->iStatus = CONNECTED;
					
					// s'il s'agit de notre 2eme tentative ...on a peut-etre perdu la connexion
					}else if( this->iUnsuccessTry == 2 ){
						this->iStatus = MAYLOSTED;
					}
					
				
				// Si on a passé le quota d'essais de maintient de connexion : on vient de la perdre
				}else{
					if( this->bDebug ){ Serial.println( "Connection lost, Trying again! "  ); }
					
					this->iStatus = LOSTED;
				}
				
				
			// si on a recu la réponse au STILLHERE
			}else{
				if( bDebug ){ Serial.println( "OK It's alive!"  ); }
				this->iUnsuccessTry = 0;
				this->iStatus = CONNECTED;
			}		
		}
	}
	return this->iStatus;
}

int ConnectoRF::slaveConnection(){
	unsigned long ulReceived = 0;
	unsigned long ulTransmit = 0;
	
	// on récupère les datas si elles sont de type de control
	ulReceived = this->getAvailableReceivedControl();

	if( ulReceived == HELLOIAMCTRL ){
		// On prépare la var de transmit
		ulTransmit = HELLOCTRL;
		if( this->bDebug ){ Serial.print( "*** Hello Mr Controler : "  ); Serial.println( ulTransmit  ); }
		// on l'envoie
		this->mySwitchTra.send(ulTransmit);
		// on est donc en début de connexion
		this->iStatus = FINDED;
		
		this->ulTimePool = millis();
	
	}else if( ulReceived == AREUOK ){
		// On prépare la var de transmit
		ulTransmit = IMOK;
		if( this->bDebug ){ Serial.print( "*** I'm OK Thanks =) : "  ); Serial.println( ulTransmit  ); }
		// on l'envoie
		this->mySwitchTra.send(ulTransmit);
		// on est donc connecté
		this->iStatus = CONNECTED;
		
		this->ulTimePool = millis();
	
	}else if( ulReceived == STILLHERE ){
		// On prépare la var de transmit
		ulTransmit = IAMHERE;
		if( this->bDebug ){ Serial.print( "*** Yes i'm here : "  ); Serial.println( ulTransmit  ); }
		// on l'envoie
		this->mySwitchTra.send(ulTransmit);
		// on est donc connecté
		this->iStatus = CONNECTED;
		
		this->ulTimePool = millis();
	// s'il n'y a rien
	}else{
		// si on a attendu plus de MAXUNSUCESSTRY fois le temps de pooling, on estime que la connexion est coupée.
		if( (millis() - this->ulTimePool) > POOLTIME*MAXUNSUCESSTRY ){
			this->iStatus = LOSTED;
			this->ulTimePool = millis();
			
		// si on a attendu plus du temps de pooling, on estime que la connexion est peut-etre coupée.
		}else if( (millis() - this->ulTimePool) > POOLTIME + POOLTIME/2 ){
			this->iStatus = MAYLOSTED;
		}
	}
	return this->iStatus;
}

unsigned long ConnectoRF::getReceivedValue(){
	return this->mySwitchRec.getReceivedValue();
}

void ConnectoRF::send(unsigned long Code){
	this->mySwitchTra.send( Code );
}

void ConnectoRF::setAsMaster(){
	this->bMaster = true;
}

void ConnectoRF::setAsSlave(){
	this->bMaster = false;
}




