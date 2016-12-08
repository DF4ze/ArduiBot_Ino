#include "UltraSonic.h"


UltraSonic::UltraSonic( int pinTrig, int pinEcho ):
	_pinTrig( pinTrig ),
	_pinEcho( pinEcho ),
	_nbSamples( 5 ),
	_timeOut( 28000 )
{
	this->init();
}

UltraSonic::UltraSonic( int pinTrig, int pinEcho, int nbSamples ):
	_pinTrig( pinTrig ),
	_pinEcho( pinEcho ),
	_nbSamples( nbSamples ),
	_timeOut( 28000 )
	
{
	if( this->_nbSamples == 0 )
		this->_nbSamples = 5;

	this->init();
}

    
void UltraSonic::init(){
	pinMode( this->_pinTrig, OUTPUT );
	pinMode( this->_pinEcho, INPUT );
}


int UltraSonic::getDistance(){
	int duration, distance;
	
	// Initialisation du capteur
	digitalWrite( this->_pinTrig, HIGH );
	delayMicroseconds( 1000 );
	digitalWrite( this->_pinTrig, LOW );
	// Mesure du temps : allé/retour de l'ultrason
	duration = pulseIn( this->_pinEcho, HIGH, this->_timeOut ); // l'ultrason ne peut mesuré précisément que jusqu'a 4-5m, je mets un timeout pour 10m
	// conversion en distance.
	distance = ( duration / 2 ) * 0.03495;
	
	// s'il y a timeout, duration va etre = 0 donc on met la distance a 10m
	if( duration == 0 )
		distance = -1;
	
	return distance;
}

int UltraSonic::getAVGDistance(){
	return this->getAVGDistance( this->_nbSamples );
}

int UltraSonic::getAVGDistance( int nbSamples ){
	int sum = 0;
	int retry = 0;
	for( int i=0; i < nbSamples; i++ ){
		int interm = this->getDistance();
		if( interm != -1 ){
			sum += this->getDistance();
		}else if( retry < nbSamples ){
			retry++;
			i--;
		}
	}
	
	return sum / this->_nbSamples;
}

void UltraSonic::setTimeOut( int timeOut ){
	this->_timeOut = timeOut;
}

void UltraSonic::setTimeOutByDistance( int distance ){
	this->_timeOut = 2*distance/0.03495;
}




