void setup()
{
 pinMode(3, OUTPUT);
 
 TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // fast PWM, clear OC2A on compare
 TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21); // fast PWM, prescaler of 64
 
 OCR2A = 207;  
 OCR2B = 6;
 }

void loop()
{}

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
