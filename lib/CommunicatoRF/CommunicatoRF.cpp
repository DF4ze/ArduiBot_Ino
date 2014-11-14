#include "CommunicatoRF.h"











CommunicatoRF::CommunicatoRF(){
	this->cRF = ConnectoRF();
	
	this->bDebug = true;
}
	
void CommunicatoRF::setAsMaster(){
	this->cRF.setAsMaster();
}

void CommunicatoRF::setAsSlave(){
	this->cRF.setAsSlave();
}

void CommunicatoRF::setPins(int a, int b){
	this->cRF.setPins( a, b );
}

int CommunicatoRF::connectionMgr(){
	return this->cRF.connectionMgr();
}


// temp
void CommunicatoRF::send( char* caCode ){
	this->cRF.send( caCode );
}
bool CommunicatoRF::isAvailable(){
	return this->cRF.isAvailable();
}
unsigned long CommunicatoRF::getReceivedValue(){
	return this->cRF.getReceivedValue();
}
void CommunicatoRF::resetAvailable(){
	this->cRF.resetAvailable();
}

