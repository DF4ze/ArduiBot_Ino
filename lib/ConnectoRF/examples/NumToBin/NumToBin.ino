void setup()
{
  Serial.begin(9600);
}
 
void loop()
{  
unsigned long myNum = 5;
String myStr;

myStr = String(myNum,BIN); 
send( myNum )  ;
Serial.println(myStr);
delay(1000);
}

void send( unsigned long Code) {
   // code recu
  Serial.print( "Code: " );
  Serial.println( Code );
  
  // convertion en string
  String sCode = String(Code,BIN);
  Serial.print( "sCode: " );
  Serial.println( sCode );
 
 // taille de la String
  unsigned int iLenght = sCode.length();
  Serial.print( "lenght : " );
  Serial.println( iLenght );
  
  // convertion en char*
  char caCode[ iLenght+1 ];
  sCode.toCharArray(caCode, iLenght+1) ;
  //caCode[iLenght+1] = '\0';
  
  Serial.print( "caCode: " );
  Serial.println( caCode );
  
 // this->send( caCode );
}
