#include "ReadSerial.h"


ReadSerial::ReadSerial(){

	this->cDelim = '*';
	this->inputString = "";
	this->stringComplete = false;
	
	this->bDebug = false;
}

void ReadSerial::setDelim( char aChar ){
	this->cDelim = aChar;
}

void ReadSerial::putAvailable( char aChar ){
	 // on recupere le nouveau char
    //char inChar = Serial.read();
    char inChar = aChar;   
    // si nous arrivons sur le delimiteur on change le flag
    if (inChar == this->cDelim){
      this->stringComplete = true;
    }else
       // sinon on ajoute a inputString:
       this->inputString += inChar;
}

bool ReadSerial::isStringComplete( ){
	return this->stringComplete;
}

String ReadSerial::getInputString(){
	String sTemp = "";
	if( this->stringComplete ){
		sTemp = this->inputString;
		this->inputString = "";
		this->stringComplete = false;
	}
	return sTemp;
}


