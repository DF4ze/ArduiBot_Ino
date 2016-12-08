#include "ReadSerial.h"


ReadSerial::ReadSerial(){

	this->cDelimCmd = '*';
	this->cDelimParams = ',';
	this->inputString = "";
	this->stringComplete = false;
	this->iWriteIndex = 0;
	this->buffSize = 10;
	this->maxParams = 10;
	
	for( int i = 0; i< this->buffSize; i++ ){
		this->buffStrings[i] = "";
	}	
	this->bDebug = false;
}

void ReadSerial::setDelimCmd( char aChar ){
	this->cDelimCmd = aChar;
}

void ReadSerial::setDelimParams( char aChar ){
	this->cDelimParams = aChar;
}

bool ReadSerial::putAvailable( char aChar ){
	bool bOK = true;
	// nous reste-t-il de la place dans le pool?
	if( this->iWriteIndex < this->buffSize ){
		// on recupere le nouveau char
		char inChar = aChar;   
		// si nous arrivons sur le delimiteur on change le flag
		if (inChar == this->cDelimCmd){
		  // on la mémo dans le pool
		  this->buffStrings[ this->iWriteIndex ] = this->inputString;
		  // on incrémente l'indicateur
		  this->iWriteIndex++;
		  // on reset la inputString
		  this->inputString = "";
			// on indique qu'une String est dispo
		  this->stringComplete = true;

		}else{
		   // sinon on ajoute a inputString:
		   this->inputString += inChar;
		}
	}else{
		bOK = false;
	}
	
	return bOK;
}

bool ReadSerial::isStringComplete( ){
	return this->stringComplete;
}

String ReadSerial::getInputString(){
	String sTemp = "";
	
	if( this->stringComplete ){
		// double vérif pour éviter un indicateur à -1
		if( this->iWriteIndex != 0 ){
			sTemp = this->pullString();
		}
	}
	return sTemp;
}

String ReadSerial::pullString(){
	// on mémorise la 1ere case
	String sTemp = this->buffStrings[0];
	// on décalle d'une case
	for( int i=1; i < this->buffSize; i++ ){
		this->buffStrings[i-1] = this->buffStrings[i];
	}
	// on éfface la derniere
	this->buffStrings[this->buffSize-1] = "";
	// on recule notre indicateur d'écriture
	this->iWriteIndex --;
	
	// si l'indicateur d'écriture est a zero, alors le pool est vide.
	if( this->iWriteIndex == 0 )
		this->stringComplete = false;
	
	return sTemp;
}

bool ReadSerial::parseStringParams( String params, String saParams[], int& nbParams ){
	const int nbMaxParam = this->maxParams;
	nbParams = 0;
	int oldpos = 0;
	bool ok = true;
	
	
	// On cherche le délimiteur
	int newpos = params.indexOf( this->cDelimParams );
	// Tant qu'on trouve un délimiteur
	while( newpos > -1 && nbParams < nbMaxParam ){
		// On le mémorise
		saParams[nbParams] = params.substring( oldpos, newpos );
		// On recale l'ancien index
		oldpos = newpos + 1;
		// On cherche de nouveau le délimiteur
		newpos = params.indexOf( this->cDelimParams, oldpos );
		// On comptabilise le nombre de params trouvés
		nbParams++;
	}
	
	// on a dépassé le nombre de parametres max
	if( newpos != -1 && nbParams >= nbMaxParam )
		ok = false;
	
	// ne pas oublier le dernier (ou le 1er s'il n'y en a qu'un) param qui n'a pas de délimiteur ;)
	saParams[nbParams] = params.substring( oldpos, params.length() );	
	nbParams++;
	
	return ok;
}

bool ReadSerial::parseIntParams( String params, int iaParams[], int& nbParams ){
	String saParams[ this->maxParams ];
	bool ok = true;
	
	ok = this->parseStringParams( params, saParams, nbParams );
	
	// transfert dans un tab de Int
	int i=0;
	int j=0;
	while(  i < nbParams ){
		iaParams[j] = saParams[i].toInt();
		
		// la convertion s'est mal passée
		if( saParams[i].compareTo( "0" ) != 0 && iaParams[j] == 0 ){
			ok = false;
		}else
			j++;
			
		i++;
	}
	nbParams = j;

	return ok;	
}
bool ReadSerial::parseFloatParams( String params, float faParams[], int& nbParams ){

	String saParams[ this->maxParams ];
	bool ok = true;
	
	ok = this->parseStringParams( params, saParams, nbParams );
	
	// transfert dans un tab de Float
	int i=0;
	int j=0;
	while(  i < nbParams ){
		faParams[j] = saParams[i].toFloat();
		
		// la convertion s'est mal passée
		if( saParams[i].compareTo( "0" ) != 0 && faParams[j] == 0 ){
			ok = false;
		}else
			j++;
			
		i++;
	}
	nbParams = j;

	return ok;
}


int ReadSerial::getMaxParams(){
	return this->maxParams;
}

