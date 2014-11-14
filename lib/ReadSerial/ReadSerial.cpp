#include "ReadSerial.h"


ReadSerial::ReadSerial(){

	this->cDelim = '*';
	this->inputString = "";
	this->stringComplete = false;
	this->iWriteIndex = 0;

	this->buffSize = 10;
	//this->buffStrings[10] = {"", "", "", "", "", "", "", "", "", ""};
	for( int i = 0; i< this->buffSize; i++ ){
		this->buffStrings[i] = "";
	}	
	this->bDebug = false;
}

void ReadSerial::setDelim( char aChar ){
	this->cDelim = aChar;
}
/* void ReadSerial::setBuffSize( int iSize ){
	this->buffSize = iSize;
	
	this->buffStrings = String [iSize];
	
	for( int i = 0; i< iSize; i++ ){
		this->buffStrings[i] = "";
	}
} */

bool ReadSerial::putAvailable( char aChar ){
	bool bOK = true;
	// nous reste-t-il de la place dans le pool?
	if( this->iWriteIndex < this->buffSize ){
		// on recupere le nouveau char
		char inChar = aChar;   
		// si nous arrivons sur le delimiteur on change le flag
		if (inChar == this->cDelim){
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

