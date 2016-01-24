void setup()
{
 pinMode(3, OUTPUT);  //1kHz on timer 2 
 pinMode(10, OUTPUT); // 100 Hz on timer 1
 
 GTCCR = _BV(TSM)| _BV(PSRASY)| _BV(PSRSYNC); // halt all timers
 
 TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // fast PWM, clear OC2A on compare
 TCCR2B = _BV(WGM22) | _BV(CS22); // fast PWM, prescaler of 64
 
 OCR2A = 249;  //1kHz
 OCR2B = 25;  //100us duty cycle
 
 TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
 TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);
 
 OCR1A = 624;  //100Hz  
 //OCR1A = 1249;  //50Hz
 //OCR1B = 6;  //100us duty cycle    
 OCR1B = 312;  //5ms duty cycle
 
 TCNT0 = 0; // set timer0 to 0
 TCNT1H = 0; // set timer1 high byte to 0
 TCNT1L = 0; // set timer1 low byte to 0
 TCNT2 = 0; // set timer2 to 0

 GTCCR = 0; // release all timers
}

void loop()
{}
