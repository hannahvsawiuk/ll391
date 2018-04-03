

  // pinMode(12, OUTPUT);
  // pinMode(11, OUTPUT);

  // TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(WGM41) | _BV(WGM40);
  // TCCR4B = _BV(CS40);
  
  // OCR2A = 50;
  // OCR2B = 120;
  // OCR2C = 200;

//Pinouts for Each Timer
//timer 0 (controls pin 13, 4) (timer 0 is special)
//timer 1 (controls pin 12, 11)
//timer 2 (controls pin 10, 9)
//timer 3 (controls pin 5, 3, 2)
//timer 4 (controls pin 8, 7, 6)




//TCCRnA --> n is your timer number
//Autonomous Motor Control

#define enA 11
#define in1 8
#define in2 9

//#define enB 10
//#define in3 13
//#define in4 12



int rotDirection = 0;
//int delayCounterClockwise = 50;

void setup() {
//  pinMode(enB, OUTPUT);
//  pinMode(in3, OUTPUT);
//  pinMode(in4, OUTPUT);
  // Set initial rotation direction


//  digitalWrite(in3, LOW);
//  digitalWrite(in4, HIGH);

  Serial.begin(9600);
  
  //Timer Adjustment to increase pwm frequency
//  pinMode(enA, OUTPUT);
//  pinMode(in1, OUTPUT);
//  pinMode(in2, OUTPUT);
//  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
//  TCCR1B = _BV(CS12);
//  OCR1A = 180;
//  OCR2B = 50;


  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);  

//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);

  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);

  
  TCCR4B = TCCR4B & ~7;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  // int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   
  TCCR4B = TCCR4B | 5;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011
  OCR4A = 150;
  OCR4B = 200;
  OCR4C = 250;
  // TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  // TCCR2B = _BV(CS20);
  
  // OCR2A = 200;
  // OCR2B = 120;

//Pinouts for Each Timer
//timer 0 (controls pin 13, 4) (timer 0 is special)
//timer 1 (controls pin 12, 11)
//timer 2 (controls pin 10, 9)
//timer 3 (controls pin 5, 3, 2)
//timer 4 (controls pin 8, 7, 6)

//TCCRnA --> n is your timer number

   
}

void loop() {
//  int potValue = analogRead(A0); // Read potentiometer value
//  int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255

//  int pwmOutput = 254;
//  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin

  
//  analogWrite(enB, pwmOutput); // Send PWM signal to L298N Enable pin

  //Clockwise Direction
//  Serial.println ("Clockwise direction");

//  delay(delayClockwise);
//
//  //Changing direction 
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, LOW);
//
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW);
//
//  //Counter-Clockwise Direction  
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);
//
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, HIGH);
//  
//  delay(delayCounterClockwise);

 
}
