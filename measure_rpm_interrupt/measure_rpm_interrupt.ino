 /*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#include "TimerOne.h"
#define enA 9
#define in1 12
#define in2 11
#define FREQ 200
// #include <SimpleTimer.h>
// SimpleTimer timer;
int rotDirection = 0;
int pressed = false;
#define encoder0PinA  2
#define encoder0PinB  3
// volatile long signed encoder0Pos = 0;
volatile unsigned rpm=0;
volatile unsigned period_milli;
volatile unsigned long accum_time;
bool change = false;
// volatile long signed accum0Pos = 0;

volatile int encoder0Pos = 0;
volatile int accum0Pos = 0;
volatile int speed = 0;



void setup() {
    Serial.begin(115200);
    // Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(encoder0PinA, INPUT);
    // pinMode(encoder0PinB, INPUT);

    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doEncoderA, CHANGE);

    // encoder pin on interrupt 1 (pin 3)
    // attachInterrupt(1, doEncoderB, CHANGE);
    // Set initial rotation direction
    // Serial.println("Setting initial direction");
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);

    Timer1.initialize(5000);         // initialize timer1, and set a 1/2 second period
    // Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
    Timer1.attachInterrupt(timer_ISR);  // attaches callback() as a timer overflow interrupt
}
void loop() {
    // int pwmOutput;
   // Serial.println("Setting PWM signal");
  //   int pwmIn = 100; // speed of 3234 RPM 
    

  //   pwmOutput = map(pwmIn, 0, 100, 0, 255);
  //   analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  // //  Serial.println("Sent PWM Signal");
  // //  Serial.println(encoder0Pos);
   
  //  // Serial.println("Now changing to backward rotation");
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);
    
    // if (micros()%500==0){
    //   encoder0Pos=0;
    // }

    // if (micros()%500==499){
    //   rpm=encoder0Pos/360*2*1000*60;
    //   Serial.println("HI");
    // }

    // if (millis==0) {//print every 10 milliseconds
    //   accum_time=millis();
    //   Serial.print(accum_time);
    //   Serial.print("\t");
    //   Serial.print(rpm);
    //   Serial.print("\n");
    // }

    // accum_time=micros();
    // Serial.print('%lu\t%d\n',accum_time,encoder0Pos);

      // accum_time=micros();
      // Serial.print(accum_time);
      // Serial.print("\t");
      // Serial.print(encoder0Pos);
      // Serial.print("\n");
    
      // if (millis() % 8 == 0)
      // {
      //   accum_time = millis();
      //   Serial.print(accum_time);
      //   Serial.print("\t");
      //   Serial.print(encoder0Pos*0.9);
      //   Serial.print("\n");
      // }

      if (change)
      {
        Serial.println(speed);
        change = false;
      }

}

void timer_ISR()
{
  accum0Pos += encoder0Pos;
  speed = encoder0Pos*60;
  encoder0Pos = 0;
  change = true;
}

// void doEncoderA() {
//   // look for a low-to-high on channel A
//   if (digitalRead(encoder0PinA) == HIGH) {

//     // check channel B to see which way encoder is turning
//     if (digitalRead(encoder0PinB) == LOW) {
//       encoder0Pos = encoder0Pos + 1;         // CW
//     }
//     else {
//       encoder0Pos = encoder0Pos - 1;         // CCW
//     }
//   }

//   else   // must be a high-to-low edge on channel A
//   {
//     // check channel B to see which way encoder is turning
//     if (digitalRead(encoder0PinB) == HIGH) {
//       encoder0Pos = encoder0Pos + 1;          // CW
//     }
//     else {
//       encoder0Pos = encoder0Pos - 1;          // CCW
//     }
//   }
//   //Serial.println (encoder0Pos, DEC);
//   // use for debugging - remember to comment out
// }


// void doEncoderB() {
//   // look for a low-to-high on channel B
//   if (digitalRead(encoder0PinB) == HIGH) {

//     // check channel A to see which way encoder is turning
//     if (digitalRead(encoder0PinA) == HIGH) {
//       encoder0Pos = encoder0Pos + 1;         // CW
//     }
//     else {
//       encoder0Pos = encoder0Pos - 1;         // CCW
//     }
//   }

//   // Look for a high-to-low on channel B

//   else {
//     // check channel B to see which way encoder is turning
//     if (digitalRead(encoder0PinA) == LOW) {
//       encoder0Pos = encoder0Pos + 1;          // CW
//     }
//     else {
//       encoder0Pos = encoder0Pos - 1;          // CCW
//     }
//   }

// }


void doEncoderA() {
  // look for a low-to-high on channel A
  encoder0Pos++;
  //Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}
