#include "myRCSwitch.h"

#if not defined( RCSwitchDisableReceiving )
unsigned long myRCSwitch::nReceivedValue = NULL;
unsigned int myRCSwitch::nReceivedBitlength = 0;
unsigned int myRCSwitch::nReceivedDelay = 0;
unsigned int myRCSwitch::nReceivedProtocol = 0;
int myRCSwitch::nReceiveTolerance = 60;
#endif
unsigned int myRCSwitch::timings[RCSWITCH_MAX_CHANGES];

myRCSwitch::myRCSwitch() {
  this->nTransmitterPin = -1;
  this->setPulseLength(350);
  this->setRepeatTransmit(10);
  this->setProtocol(1);
  #if not defined( RCSwitchDisableReceiving )
  this->nReceiverInterrupt = -1;
  this->setReceiveTolerance(60);
  myRCSwitch::nReceivedValue = NULL;
  #endif
}

/**
  * Sets the protocol to send.
  */
void myRCSwitch::setProtocol(int nProtocol) {
  this->nProtocol = nProtocol;
  if (nProtocol == 1){
    this->setPulseLength(350);
  }
  else if (nProtocol == 2) {
    this->setPulseLength(650);
  }
  else if (nProtocol == 3) {
    this->setPulseLength(100);
  }
}



/**
  * Sets pulse length in microseconds
  */
void myRCSwitch::setPulseLength(int nPulseLength) {
  this->nPulseLength = nPulseLength;
}

/**
 * Sets Repeat Transmits
 */
void myRCSwitch::setRepeatTransmit(int nRepeatTransmit) {
  this->nRepeatTransmit = nRepeatTransmit;
}

/**
 * Set Receiving Tolerance
 */
#if not defined( RCSwitchDisableReceiving )
void myRCSwitch::setReceiveTolerance(int nPercent) {
  myRCSwitch::nReceiveTolerance = nPercent;
}
#endif
  

/**
 * Enable transmissions
 *
 * @param nTransmitterPin    Arduino Pin to which the sender is connected to
 */
void myRCSwitch::enableTransmit(int nTransmitterPin) {
  this->nTransmitterPin = nTransmitterPin;
  pinMode(this->nTransmitterPin, OUTPUT);
}

/**
  * Disable transmissions
  */
void myRCSwitch::disableTransmit() {
  this->nTransmitterPin = -1;
}



void myRCSwitch::send(unsigned long Code) {
  String sCode = String(Code,BIN);
  
  char caCode[ sCode.length()+1 ];
  sCode.toCharArray(caCode, sCode.length()+1) ;
  
  this->send( caCode );
}

void myRCSwitch::send(unsigned long Code, unsigned int length) {
  this->send( this->dec2binWzerofill(Code, length) );
}

void myRCSwitch::send(char* sCodeWord) {
  for (int nRepeat=0; nRepeat<nRepeatTransmit; nRepeat++) {
    int i = 0;
    while (sCodeWord[i] != '\0') {
      switch(sCodeWord[i]) {
        case '0':
          this->send0();
        break;
        case '1':
          this->send1();
        break;
      }
      i++;
    }
    this->sendSync();
  }
}

void myRCSwitch::transmit(int nHighPulses, int nLowPulses) {
    #if not defined ( RCSwitchDisableReceiving )
    boolean disabled_Receive = false;
    int nReceiverInterrupt_backup = nReceiverInterrupt;
    #endif
    if (this->nTransmitterPin != -1) {
        #if not defined( RCSwitchDisableReceiving )
        if (this->nReceiverInterrupt != -1) {
            this->disableReceive();
            disabled_Receive = true;
        }
        #endif
        digitalWrite(this->nTransmitterPin, HIGH);
        delayMicroseconds( this->nPulseLength * nHighPulses);
        digitalWrite(this->nTransmitterPin, LOW);
        delayMicroseconds( this->nPulseLength * nLowPulses);
        
        #if not defined( RCSwitchDisableReceiving )
        if(disabled_Receive){
            this->enableReceive(nReceiverInterrupt_backup);
        }
        #endif
    }
}
/**
 * Sends a "0" Bit
 *                       _    
 * Waveform Protocol 1: | |___
 *                       _  
 * Waveform Protocol 2: | |__
 */
void myRCSwitch::send0() {
    if (this->nProtocol == 1){
        this->transmit(1,3);
    }
    else if (this->nProtocol == 2) {
        this->transmit(1,2);
    }
    else if (this->nProtocol == 3) {
        this->transmit(4,11);
    }
}

/**
 * Sends a "1" Bit
 *                       ___  
 * Waveform Protocol 1: |   |_
 *                       __  
 * Waveform Protocol 2: |  |_
 */
void myRCSwitch::send1() {
      if (this->nProtocol == 1){
        this->transmit(3,1);
    }
    else if (this->nProtocol == 2) {
        this->transmit(2,1);
    }
    else if (this->nProtocol == 3) {
        this->transmit(9,6);
    }
}



/**
 * Sends a "Sync" Bit
 *                       _
 * Waveform Protocol 1: | |_______________________________
 *                       _
 * Waveform Protocol 2: | |__________
 */
void myRCSwitch::sendSync() {

    if (this->nProtocol == 1){
        this->transmit(1,31);
    }
    else if (this->nProtocol == 2) {
        this->transmit(1,10);
    }
    else if (this->nProtocol == 3) {
        this->transmit(1,71);
    }
}

#if not defined( RCSwitchDisableReceiving )
/**
 * Enable receiving data
 */
void myRCSwitch::enableReceive(int interrupt) {
  this->nReceiverInterrupt = interrupt;
  this->enableReceive();
}

void myRCSwitch::enableReceive() {
  if (this->nReceiverInterrupt != -1) {
    myRCSwitch::nReceivedValue = NULL;
    myRCSwitch::nReceivedBitlength = NULL;
    attachInterrupt(this->nReceiverInterrupt, handleInterrupt, CHANGE);
  }
}

/**
 * Disable receiving data
 */
void myRCSwitch::disableReceive() {
  detachInterrupt(this->nReceiverInterrupt);
  this->nReceiverInterrupt = -1;
}

bool myRCSwitch::available() {
  return myRCSwitch::nReceivedValue != NULL;
}

void myRCSwitch::resetAvailable() {
  myRCSwitch::nReceivedValue = NULL;
}

unsigned long myRCSwitch::getReceivedValue() {
    return myRCSwitch::nReceivedValue;
}

unsigned int myRCSwitch::getReceivedBitlength() {
  return myRCSwitch::nReceivedBitlength;
}

unsigned int myRCSwitch::getReceivedDelay() {
  return myRCSwitch::nReceivedDelay;
}

unsigned int myRCSwitch::getReceivedProtocol() {
  return myRCSwitch::nReceivedProtocol;
}

unsigned int* myRCSwitch::getReceivedRawdata() {
    return myRCSwitch::timings;
}

/**
 *
 */
bool myRCSwitch::receiveProtocol1(unsigned int changeCount){
    
      unsigned long code = 0;
      unsigned long delay = myRCSwitch::timings[0] / 31;
      unsigned long delayTolerance = delay * myRCSwitch::nReceiveTolerance * 0.01;    

      for (int i = 1; i<changeCount ; i=i+2) {
      
          if (myRCSwitch::timings[i] > delay-delayTolerance && myRCSwitch::timings[i] < delay+delayTolerance && myRCSwitch::timings[i+1] > delay*3-delayTolerance && myRCSwitch::timings[i+1] < delay*3+delayTolerance) {
            code = code << 1;
          } else if (myRCSwitch::timings[i] > delay*3-delayTolerance && myRCSwitch::timings[i] < delay*3+delayTolerance && myRCSwitch::timings[i+1] > delay-delayTolerance && myRCSwitch::timings[i+1] < delay+delayTolerance) {
            code+=1;
            code = code << 1;
          } else {
            // Failed
            i = changeCount;
            code = 0;
          }
      }      
      code = code >> 1;
    if (changeCount > 6) {    // ignore < 4bit values as there are no devices sending 4bit values => noise
      myRCSwitch::nReceivedValue = code;
      myRCSwitch::nReceivedBitlength = changeCount / 2;
      myRCSwitch::nReceivedDelay = delay;
      myRCSwitch::nReceivedProtocol = 1;
    }

    if (code == 0){
        return false;
    }else if (code != 0){
        return true;
    }
    

}

bool myRCSwitch::receiveProtocol2(unsigned int changeCount){
    
      unsigned long code = 0;
      unsigned long delay = myRCSwitch::timings[0] / 10;
      unsigned long delayTolerance = delay * myRCSwitch::nReceiveTolerance * 0.01;    

      for (int i = 1; i<changeCount ; i=i+2) {
      
          if (myRCSwitch::timings[i] > delay-delayTolerance && myRCSwitch::timings[i] < delay+delayTolerance && myRCSwitch::timings[i+1] > delay*2-delayTolerance && myRCSwitch::timings[i+1] < delay*2+delayTolerance) {
            code = code << 1;
          } else if (myRCSwitch::timings[i] > delay*2-delayTolerance && myRCSwitch::timings[i] < delay*2+delayTolerance && myRCSwitch::timings[i+1] > delay-delayTolerance && myRCSwitch::timings[i+1] < delay+delayTolerance) {
            code+=1;
            code = code << 1;
          } else {
            // Failed
            i = changeCount;
            code = 0;
          }
      }      
      code = code >> 1;
    if (changeCount > 6) {    // ignore < 4bit values as there are no devices sending 4bit values => noise
      myRCSwitch::nReceivedValue = code;
      myRCSwitch::nReceivedBitlength = changeCount / 2;
      myRCSwitch::nReceivedDelay = delay;
      myRCSwitch::nReceivedProtocol = 2;
    }

    if (code == 0){
        return false;
    }else if (code != 0){
        return true;
    }

}

/** Protocol 3 is used by BL35P02.
 *
 */
bool myRCSwitch::receiveProtocol3(unsigned int changeCount){
    
      unsigned long code = 0;
      unsigned long delay = myRCSwitch::timings[0] / PROTOCOL3_SYNC_FACTOR;
      unsigned long delayTolerance = delay * myRCSwitch::nReceiveTolerance * 0.01;    

      for (int i = 1; i<changeCount ; i=i+2) {
      
          if  (myRCSwitch::timings[i]   > delay*PROTOCOL3_0_HIGH_CYCLES - delayTolerance
            && myRCSwitch::timings[i]   < delay*PROTOCOL3_0_HIGH_CYCLES + delayTolerance
            && myRCSwitch::timings[i+1] > delay*PROTOCOL3_0_LOW_CYCLES  - delayTolerance
            && myRCSwitch::timings[i+1] < delay*PROTOCOL3_0_LOW_CYCLES  + delayTolerance) {
            code = code << 1;
          } else if (myRCSwitch::timings[i]   > delay*PROTOCOL3_1_HIGH_CYCLES - delayTolerance
                  && myRCSwitch::timings[i]   < delay*PROTOCOL3_1_HIGH_CYCLES + delayTolerance
                  && myRCSwitch::timings[i+1] > delay*PROTOCOL3_1_LOW_CYCLES  - delayTolerance
                  && myRCSwitch::timings[i+1] < delay*PROTOCOL3_1_LOW_CYCLES  + delayTolerance) {
            code+=1;
            code = code << 1;
          } else {
            // Failed
            i = changeCount;
            code = 0;
          }
      }      
      code = code >> 1;
      if (changeCount > 6) {    // ignore < 4bit values as there are no devices sending 4bit values => noise
        myRCSwitch::nReceivedValue = code;
        myRCSwitch::nReceivedBitlength = changeCount / 2;
        myRCSwitch::nReceivedDelay = delay;
        myRCSwitch::nReceivedProtocol = 3;
      }

      if (code == 0){
        return false;
      }else if (code != 0){
        return true;
      }
}

void myRCSwitch::handleInterrupt() {

  static unsigned int duration;
  static unsigned int changeCount;
  static unsigned long lastTime;
  static unsigned int repeatCount;
  

  long time = micros();
  duration = time - lastTime;
 
  if (duration > 5000 && duration > myRCSwitch::timings[0] - 200 && duration < myRCSwitch::timings[0] + 200) {
    repeatCount++;
    changeCount--;
    if (repeatCount == 2) {
      if (receiveProtocol1(changeCount) == false){
        if (receiveProtocol2(changeCount) == false){
          if (receiveProtocol3(changeCount) == false){
            //failed
          }
        }
      }
      repeatCount = 0;
    }
    changeCount = 0;
  } else if (duration > 5000) {
    changeCount = 0;
  }
 
  if (changeCount >= RCSWITCH_MAX_CHANGES) {
    changeCount = 0;
    repeatCount = 0;
  }
  myRCSwitch::timings[changeCount++] = duration;
  lastTime = time;  
}

/**
  * Turns a decimal value to its binary representation
  */
char* myRCSwitch::dec2binWzerofill(unsigned long Dec, unsigned int bitLength){
    return dec2binWcharfill(Dec, bitLength, '0');
}

char* myRCSwitch::dec2binWcharfill(unsigned long Dec, unsigned int bitLength, char fill){
  static char bin[64];
  unsigned int i=0;

  while (Dec > 0) {
    bin[32+i++] = ((Dec & 1) > 0) ? '1' : fill;
    Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j< bitLength; j++) {
    if (j >= bitLength - i) {
      bin[j] = bin[ 31 + i - (j - (bitLength - i)) ];
    }else {
      bin[j] = fill;
    }
  }
  bin[bitLength] = '\0';
  
  return bin;
}

#endif

