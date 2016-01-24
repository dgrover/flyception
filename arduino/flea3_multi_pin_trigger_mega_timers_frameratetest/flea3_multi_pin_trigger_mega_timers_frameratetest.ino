int incomingByte = 0;
bool flash_fired_once = false;
int flash_count = 0;

void setup()
{
 Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
  
 pinMode(4, OUTPUT);
 digitalWrite(4, HIGH); 
 
 pinMode(9, OUTPUT);  //1kHz on timer 2 
 pinMode(12, OUTPUT); // 100 Hz on timer 1
 pinMode(2, OUTPUT);  //100 Hz on timer 3
 
 GTCCR = _BV(TSM)| _BV(PSRASY)| _BV(PSRSYNC); // halt all timers
 
 TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // fast PWM, clear OC2A on compare
 //TCCR2B = _BV(WGM22) | _BV(CS22); //prescale 64
 //TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS20); //prescale 128
 TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21); //prescale 256
 
 OCR2A = 207;  
 //OCR2B = 25;  //100us duty cycle at prescale 256
 //OCR2B = 13;  //100us duty cycle at prescale 128
 OCR2B = 6;  //100us duty cycle at prescale 256
 
 TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
 TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);
 
 OCR1A = 624;  //100Hz 
 //OCR1A = 1249;  //50Hz  
 //OCR1B = 312;  //5ms duty cycle
 OCR1B = 500;  //8ms duty cycle
 
 TCCR3A = _BV(COM3A0) | _BV(COM3B1) | _BV(WGM31) | _BV(WGM30);
 TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS32);
 
 OCR3A = 624;  //100Hz
 //OCR3A = 1249;  //50Hz 
 OCR3B = 6;  //100us duty cycle

 TCNT0 = 0; // set timer0 to 0
 
 TCNT1 = 0x223;  //offset to compensate for delay in triggering of fluoview camera
 
 TCNT2 = 0; // set timer2 to 0
 
 TCNT3H = 0; // set timer3 high byte to 0
 TCNT3L = 0; // set timer3 low byte to 0

 GTCCR = 0; // release all timers
}

void loop()
{
  if (Serial.available() > 0)
    incomingByte = Serial.read();  //if incoming flash trigger from keyboard
  
  if (incomingByte > 0)  //if there was a trigger for flash
  {
      digitalWrite(4, LOW);
      delay(1000);
      digitalWrite(4, HIGH);
      
      incomingByte = 0;
      flash_fired_once = true;
      flash_count = 0;
  }  
  
  if (flash_fired_once)
  {
    delay(10);
    flash_count++;
  }
  
  if (flash_count == 6000)
  {
      digitalWrite(4, LOW);
      delay(1000);
      digitalWrite(4, HIGH);
      
      flash_fired_once = false;
      flash_count = 0;
  }
}


//prescale = 64
//1000Hz
//OCR2A = 249
//OCR2B = 25

// prescale = 128
//900Hz
//OCR2A = 138;  
//OCR2B = 13;

//800Hz
//OCR2A = 155;  
//OCR2B = 13;

//700Hz
//OCR2A = 177;  
//OCR2B = 13;

//600Hz
//OCR2A = 207;  
//OCR2B = 13;

//500Hz
//OCR2A = 249;
//OCR2B = 13;

// prescale = 256 
//400Hz
//OCR2A = 155
//OCR2B = 6;

//300Hz
//OCR2A = 207
//OCR2B = 6;
