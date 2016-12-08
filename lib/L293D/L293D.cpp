#include "L293D.h"



L293D::L293D( int mg_s1_pin, int mg_s2_pin, int mg_v_pin, int md_s1_pin, int md_s2_pin, int md_v_pin  ):
	_mg_s1_pin(mg_s1_pin),
	_mg_s2_pin(mg_s2_pin),
	_mg_v_pin(mg_v_pin),
	_md_s1_pin(md_s1_pin),
	_md_s2_pin(md_s2_pin),
	_md_v_pin(md_v_pin),
	_revX( false ),
	_revY( false )
{
	this->init();
}


L293D::L293D( int mg_s1_pin, int mg_s2_pin, int mg_v_pin, int md_s1_pin, int md_s2_pin, int md_v_pin, bool revX, bool revY  ):
	_mg_s1_pin(mg_s1_pin),
	_mg_s2_pin(mg_s2_pin),
	_mg_v_pin(mg_v_pin),
	_md_s1_pin(md_s1_pin),
	_md_s2_pin(md_s2_pin),
	_md_v_pin(md_v_pin),
	_revX( revX ),
	_revY( revY )
{
	this->init();
}


void L293D::init(){
	pinMode( this->_mg_s1_pin, 	OUTPUT);
	pinMode( this->_mg_s2_pin, 	OUTPUT);
	pinMode( this->_mg_v_pin, 	OUTPUT);
	pinMode( this->_md_s1_pin, 	OUTPUT);
	pinMode( this->_md_s2_pin, 	OUTPUT);
	pinMode( this->_md_v_pin, 	OUTPUT);
}

void L293D::setActionMotors( int vitesse, int delta ){
	if( this->_revX )
		vitesse = -vitesse;
	if( this->_revY )
		delta = -delta;
	
	int iVDroite = vitesse - delta/2;		
	int iVGauche = vitesse + delta/2;		
	
	iVGauche = this->normalizePWM( iVGauche );
	iVDroite = this->normalizePWM( iVDroite );
	
	Serial.print( "VDroite : " );
	Serial.println( iVDroite );
	Serial.print( "VGauche : " );
	Serial.println( iVGauche );

	
	this->actionMD( iVDroite );
	this->actionMG( iVGauche );	
}

void L293D::actionMG( int iVitesse ){
	if( iVitesse > 0 ){
		digitalWrite( this->_mg_s1_pin, 1 );
		digitalWrite( this->_mg_s2_pin, 0 );
		analogWrite( this->_mg_v_pin, iVitesse );
	}else if( iVitesse < 0 ){
		digitalWrite( this->_mg_s1_pin, 0 );
		digitalWrite( this->_mg_s2_pin, 1 );
		analogWrite( this->_mg_v_pin, -iVitesse );
	}else{
		digitalWrite( this->_mg_s1_pin, 1 );
		digitalWrite( this->_mg_s2_pin, 1 );
		analogWrite( this->_mg_v_pin, 0 );
		
	}
}
void L293D::actionMD( int iVitesse ){
	if( iVitesse > 0 ){
		digitalWrite( this->_md_s1_pin, 1 );
		digitalWrite( this->_md_s2_pin, 0 );
		analogWrite( this->_md_v_pin, iVitesse );
	}else if( iVitesse < 0 ){
		digitalWrite( this->_md_s1_pin, 0 );
		digitalWrite( this->_md_s2_pin, 1 );
		analogWrite( this->_md_v_pin, -iVitesse );
	}else{
		digitalWrite( this->_md_s1_pin, 1 );
		digitalWrite( this->_md_s2_pin, 1 );
		analogWrite( this->_md_v_pin, 0 );
		
	}
}

void L293D::stopMotors(){
	
}

void L293D::reverseX( bool revX ){
	this->_revX = revX;
}
void L293D::reverseY( bool revY ){
	this->_revY = revY;
}

int L293D::normalizePWM( int pwm ){
	if( pwm > MAX_PWM )
		pwm = MAX_PWM;
	else if( pwm < -MAX_PWM )
		pwm = -MAX_PWM;

	return pwm;
}

	
	
	
	
	
	
	
	