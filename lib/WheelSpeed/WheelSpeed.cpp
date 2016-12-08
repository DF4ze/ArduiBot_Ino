#include "WheelSpeed.h"


WheelSpeed::WheelSpeed( int pin, int nbHoles ):
	_pin( pin ),
	_nbHoles( nbHoles ),
	_diam( 0 ),
	_count( 0 ),
	_countDist( 0 ),
	_isLow( false ),
	_period( 1000 ),
	_TbyS(0)
{
	this->init();
}

WheelSpeed::WheelSpeed( int pin, int nbHoles, long period ):
	_pin( pin ),
	_nbHoles( nbHoles ),
	_diam( 0 ),
	_count( 0 ),
	_countDist( 0 ),
	_isLow( false ),
	_period( period ),
	_TbyS(0)
{
	this->init();	
}

WheelSpeed::WheelSpeed( int pin, int nbHoles, long period, int diam ):
	_pin( pin ),
	_nbHoles( nbHoles ),
	_diam( diam ),
	_count( 0 ),
	_countDist( 0 ),
	_isLow( false ),
	_period( period ),
	_TbyS(0)
{
	this->init();
}
    
WheelSpeed::WheelSpeed( int pin, int nbHoles, int diam ):
	_pin( pin ),
	_nbHoles( nbHoles ),
	_diam( diam ),
	_count( 0 ),
	_countDist( 0 ),
	_isLow( false ),
	_period( 1000 ),
	_TbyS(0)
{
	this->init();
}
    
void WheelSpeed::init(){
	this->_timeStamp = millis();
	this->_lastTimeStamp = millis();
	
	this->_peri = (double)PI * this->_diam;
	
	pinMode( _pin, INPUT );
}

double WheelSpeed::calcTbyS(){
	long delay = this->_timeStamp - this->_lastTimeStamp;
	double ratio = (double)delay / 1000;
	double tbys = ((double)this->_count / this->_nbHoles) * ratio;
	return tbys;
}

void WheelSpeed::check(){
  if( !digitalRead( this->_pin ) )
    this->_isLow = true;
  
  if( digitalRead( this->_pin ) && this->_isLow ){
    this->_count++;
    this->_countDist++;
    this->_isLow = false;
  }	
  
  this->_timeStamp = millis();
  if( this->_timeStamp - this->_lastTimeStamp >= this->_period ){
    this->_TbyS = calcTbyS();
    
	this->_lastTimeStamp = this->_timeStamp;
    this->_count = 0;
  }
}



double WheelSpeed::getTbyS(){
	return this->_TbyS;
}

double WheelSpeed::getDistanceCm(){
	double distance = 0;
	if( this->_diam != 0 ){
		distance = this->_peri * ( (double)this->_countDist / this->_nbHoles )/10;
		this->_countDist = 0;
	}
	return distance;
}

double WheelSpeed::getSpeedKMH(){
	double speed = 0;
	if( this->_diam != 0 ){
		speed = (this->_peri * this->getTbyS()) / 1000000 * 60 * 60;
	}
	return speed;
}

