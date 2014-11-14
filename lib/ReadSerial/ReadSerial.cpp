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
		  // on la m�mo dans le pool
		  this->buffStrings[ this->iWriteIndex ] = this->inputString;
		  // on incr�mente l'indicateur
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
		// double v�rif pour �viter un indicateur � -1
		if( this->iWriteIndex != 0 ){
			sTemp = this->pullString();
		}
	}
	return sTemp;
}

String ReadSerial::pullString(){
	// on m�morise la 1ere case
	String sTemp = this->buffStrings[0];
	// on d�calle d'une case
	for( int i=1; i < this->buffSize; i++ ){
		this->buffStrings[i-1] = this->buffStrings[i];
	}
	// on �fface la derniere
	this->buffStrings[this->buffSize-1] = "";
	// on recule notre indicateur d'�criture
	this->iWriteIndex --;
	
	// si l'indicateur d'�criture est a zero, alors le pool est vide.
	if( this->iWriteIndex == 0 )
		this->stringComplete = false;
	
	return sTemp;
}

