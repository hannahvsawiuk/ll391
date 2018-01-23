/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#define enA 9
#define in1 6
#define in2 7

int rotDirection = 0;
int pressed = false;
#define encoder0PinA  2
#define encoder0PinB  3
volatile unsigned int encoder0Pos = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);

    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doEncoderA, CHANGE);

    // encoder pin on interrupt 1 (pin 3)
    attachInterrupt(1, doEncoderB, CHANGE);
    // Set initial rotation direction
    Serial.println("Setting initial direction");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    
}
void loop() {
    int potValue = analogRead(A0); // Read potentiometer value
    int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
    Serial.println("Setting PWM signal");
    int pwmOutput = 10;
    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
    Serial.println("Sent PWM Signal");

    Serial.println(encoder0Pos);
   
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

    if endcoder0Pos

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
  Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
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
