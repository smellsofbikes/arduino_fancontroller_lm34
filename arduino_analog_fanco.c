#include <PID_v1.h>

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,16,10,3, REVERSE);

long adc_result;
int debug = 1;

void setup(void) {
  if(debug)
    Serial.begin(9600);
  pinMode(9, OUTPUT);  // enable output on 16 bit capable pwm pin
  myPID.SetOutputLimits(0, 1023); // required coz default is for an 8 bit pwm and I'm setting this up as a 10 bit pwm
  // without the above, it'll have a max output of 256 on a pin configured for 10-bit, so max pwm achieved will be 25% dutycycle
  
  TCCR1A = (1<<COM1A1) |  (1<<WGM10) | (1<<WGM11);
  // this should set 10 bit fast pwm mode, set clock full speed, set atmega pin 15 as pwm output
  TCCR1B = (1 << WGM12) | (1 << CS10) | (0<<CS12); // set for /1024 divider
  OCR1A = 0; // initialize 
  OCR1B = 0; // initialize
  ICR1 = 0b0000001111111111; //set to 10 bit pwm

// below section definitely works
  ADMUX =  0b01000100;  // measuring on ADC4, use AVCC tied to VCC as ref, right-justify result
  ADCSRA = 0b10101111;  // AD-converter on, interrupt enabled, prescaler = 128
  ADCSRB = 0x40;        // AD channels MUX on, free running mode
  ADCSRA |= (1<<ADSC);  // Start the conversion by setting bit 6 (=ADSC) in ADCSRA
  sei();                // set interrupt flag
    
  Setpoint = 200;       // this is degrees F, presuming use of LM34 fahrenheit temp sensor
  myPID.SetMode(AUTOMATIC);
}

ISR(ADC_vect)
{
  cli();
  adc_result = ADCL;
  adc_result += (ADCH << 8);
  sei();
}
 
void loop(void) {
  float temp;
   // 1023 = 5V = 500 degrees
  temp = adc_result; // this should be calibrated at some point
  if(debug)
  {
    Serial.print("  Raw value = ");
    Serial.print(temp);
    Serial.print(" F, ");
  }
  temp = float(temp)/2.046; // this should be calibrated at some point
  if(debug)
  {
    Serial.print("  Temperature = ");
    Serial.print(temp);
    Serial.print(" F, ");
  }
  
  Input = temp;
  cli();
  myPID.Compute();
  sei();
  if (Output > 1023)
    Output = 1023;
  if (Output < 0)
    Output = 0;
  OCR1A = Output;  // compiler automatically knows to break up the 16 bit word into two bytes.
  OCR1B = Output;
  if(debug)
  {
    Serial.print(" PID = ");
    Serial.println(Output);
  }
}
