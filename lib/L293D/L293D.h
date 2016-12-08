
#ifndef L293D_h
#define L293D_h


#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"	
#else
    #include "WProgram.h"
#endif





class L293D{

  public:
    L293D( int mg_s1_pin, int mg_s2_pin, int mg_v_pin, int md_s1_pin, int md_s2_pin, int md_v_pin  );
    L293D( int mg_s1_pin, int mg_s2_pin, int mg_v_pin, int md_s1_pin, int md_s2_pin, int md_v_pin, bool revX, bool revY  );
	
	void setActionMotors( int vitesse, int delta );
	void stopMotors();
	void reverseX( bool revX );
	void reverseY( bool revY );


  private:	
	const int MAX_PWM = 200;
  
	int _mg_s1_pin;
	int _mg_s2_pin;
	int _mg_v_pin;
	int _md_s1_pin;
	int _md_s2_pin;
	int _md_v_pin;
	bool _revX;
	bool _revY;
	
	void actionMG( int iVitesse );
	void actionMD( int iVitesse );
	int normalizePWM( int pwm );
	
	void init();
};
#endif