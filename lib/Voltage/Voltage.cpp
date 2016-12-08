#include "Voltage.h"

#define GAIN 50/9.0	//VOL = SIG*GAIN
#define ADC_REF 5  	//reference voltage of ADC is 5v
					//Warning:
					//     If the reference voltage of ADC is 5V, the VOL should be less than 25V;
					//     If the reference voltage of ADC is 3.3V, the VOL should be less than 18V;
					//     If the reference voltage of ADC is 2.56V, the VOL should be less than 14V;
					//     If the reference voltage of ADC is 1.1V, the VOL should be less than 6V;

Voltage::Voltage( int pin ):
	_pin( pin ),
	voltageSamples( 10000 ),  	// Nombre de samples
	voltageSamplingNb( 0 ), 	// incremential number
	voltageValuesSum( 0 ),		// values sum
	voltage(0)
{
	this->init();
}

bool Voltage::check(){
	bool ok = false;
	// on récupère la valeur du capteur
	int sensorValue;  
	sensorValue=analogRead(this->_pin);
	// on incrémente
	this->voltageSamplingNb ++;
	this->voltageValuesSum += sensorValue;

	// si nous avons passé le nombre d'échantillonage
	if( this->voltageSamplingNb > this->voltageSamples ){
		// on calcule la moyenne
		sensorValue = this->voltageValuesSum / this->voltageSamples; 
		this->voltage = GAIN * sensorValue * ADC_REF / 1023.00; // tiré de l'exemple officiel
		// reset
		this->voltageSamplingNb = 0;
		this->voltageValuesSum = 0;
		
		ok = true;
	}	
	return ok;
}

float Voltage::getVoltage(){
	return this->voltage;
}
    
void Voltage::init(){
}

