/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#define enA 8
#define in1 9
#define in2 10


#define enB 13
#define in1_B 11
#define in2_B 12



int rotDirection = 0;
int pressed = false;
#define encoder0PinA  2
#define encoder0PinB  3
volatile signed int encoder0Pos = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // pinMode(enB, OUTPUT);
    // pinMode(in1_B, OUTPUT);
    // pinMode(in2_B, OUTPUT);

    // pinMode(encoder0PinA, INPUT);
    // pinMode(encoder0PinB, INPUT);

    // encoder pin on interrupt 0 (pin 2)
    // attachInterrupt(0, doEncoderA, CHANGE);

    // encoder pin on interrupt 1 (pin 3)
    // attachInterrupt(1, doEncoderB, CHANGE);
    // Set initial rotation direction
    Serial.println("Setting initial direction");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW); 

    TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(WGM41) | _BV(WGM40);
    TCCR4B = _BV(CS40);
    
    OCR4A = 200;
    OCR4B = 200;
    OCR4C = 200;
    // OCR4B = 140;

//Pinouts for Each Timer
//timer 0 (controls pin 13, 4) (timer 0 is special)
//timer 1 (controls pin 12, 11)
//timer 2 (controls pin 10, 9)
//timer 3 (controls pin 5, 3, 2)
//timer 4 (controls pin 8, 7, 6)

//TCCRnA --> n is your timer number
   pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);

  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  
  OCR2A = 200;
  OCR2B = 120;

//Pinouts for Each Timer
//timer 0 (controls pin 13, 4) (timer 0 is special)
//timer 1 (controls pin 12, 11)
//timer 2 (controls pin 10, 9)
//timer 3 (controls pin 5, 3, 2)
//timer 4 (controls pin 8, 7, 6)

//TCCRnA --> n is your timer number




}

void loop() {
    int pwmOutput;
   // Serial.println("Setting PWM signal");
    // int pwmIn = 100; // speed of 3234 RPM


    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW); 
    OCR4A = 200;
    Serial.println("Turning");

    // analogWrite(enB, 255); // Send PWM signal to L298N Enable pin
    // digitalWrite(in1_B, HIGH);
    // digitalWrite(in2_B, LOW); 
  //  Serial.println("Sent PWM Signal");
  //  Serial.println(encoder0Pos);
   
  //  // Serial.println("Now changing to backward rotation");
  //   digitalWrite(in1, LOW);
  //   digitalWrite(in2, HIGH);

  //   delay(3000);
  //   digitalWrite(in1, LOW);
  //   digitalWrite(in2, LOW);
  //   delay(200);
  //   digitalWrite(in1, HIGH);
  //   digitalWrite(in2, LOW);
  //   delay(3000);
  //   digitalWrite(in1, LOW);
  //   digitalWrite(in2, LOW);
  //   delay(200);
    // // if ( rotDirection == 0) {
    // //     digitalWrite(in1, HIGH);
    // //     digitalWrite(in2, LOW);
    // //     delay(5000);
    // // }
    // // else
    // // {
    // //     digitalWrite(in1, LOW);
    // //     digitalWrite(in2, HIGH);
    // //     delay(5000);
    // // }
    // Serial.println("Now changing to backward rotation");
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);
    // delay(2000);
    // Serial.println("Rotating backwards");
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, LOW);
    // Serial.println("Set both low");
    // // delay(500);
    // Serial.println("Now changing to forward rotation");
    // digitalWrite(in1, HIGH);
    // digitalWrite(in2, LOW);
    // Serial.println("Rotating forwards");
    // delay(2000);
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, LOW);
    // Serial.println("Set both low");
    // // delay(500);

    // Serial.println(encoder0Pos);
    // delay(200);

}


void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println (encoder0Pos, DEC);
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}


