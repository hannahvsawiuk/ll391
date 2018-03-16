#include <Math.h>
#define encoder0PinA  2
#define encoder0PinB  3
#define reachPoint 13
#define largeUndershoot 8
#define smallUndershoot 7
#define smallError 6
#define smallOvershoot 5
#define largeOvershoot 4

#include <TimerOne.h>


// float encoder0Pos = 0; //set the encoder0Poss of the encoder
volatile float encoder0Pos = 0;
float angle = 0;//set the angles
float lastAngle=0;
// boolean A,B;
// byte state, statep;
float desired = 0.79;
float setpoint = desired;
// float setpoint = waypoints[i];
float currEncoder0Pos;

float waypoints[] = { 3.14159/4, 3.14159/8, -3.14159/16};
float pwm = 9;// this is the PWM pin for the motor for how much we move it to correct for its error
const int dir1 = 11;//these pins are to control the direction of the motor (clockwise/encoder0Poser-clockwise)
const int dir2 = 12;

// float min_setpoint = 85;//I am setting it to move through 100 degrees
// float max_setpoint = 95;
// float Kp = 0.32;// you can set these constants however you like depending on trial & error
// float Ki = 0.1;
// float Kd = 0.3;
float sampleTime=0.002;

// float Kp = 200;
// float Ki = 35;
// float Kd = 18; 
// float N=80;         //vibrates

// volatile int accum0Pos = 0;
// volatile int speed = 0;      

// float Kp = 206;
// float Ki = 15;
// float Kd = 20; 
// float N=100;         // reaches point but too slow

// float Kp = 207;
// float Ki = 40;
// float Kd = 30; 
// float N=100;  

// float Kp = 205.101;
// float Ki = 0;
// float Kd = 11.7; 
// float N=100;        settles at smaller angle

// float Kp = 205.2;
// float Ki = 0;
// float Kd = 0.983*12; 
// float N=95;  

//  float Kp = 207;
//  float Ki = 20;
//  float Kd = 20; 
//  float N=100;  


// float Kp = 205;
// float Ki = 30;
// float Kd = 15.1; 
// float N=100;         //vibrates / WORKS

float Kp = 207.5;
float Ki = 30;
float Kd = 16.5; 
float N=100;         //vibrates / WORKS


// float Kp = 150;
// float Ki = 15;
// float Kd = 15.1; 
// float N=100;         //vibrates



float lastError = 0;
float error = 0;

// float changeError = 0;
// float totalError = 0;
float pidTerm = 0;
float pidTermScaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|
bool change = false;
float lastIntTerm = 0;
float intTerm = 0;
float lastDerivTerm = 0;
float derivTerm = 0;
float porTerm = 0;

// volatile unsigned long accumTime;
// volatile unsigned long lastTime;
// volatile unsigned long currTime;

int i = 0;

void setup() {
  Serial.begin(115200);
  pinMode(encoder0PinA, INPUT);//encoder pins
  pinMode(encoder0PinB, INPUT);

  pinMode(reachPoint, OUTPUT); 
  pinMode(smallOvershoot, OUTPUT);
  pinMode(largeOvershoot, OUTPUT); 
  pinMode(smallError, OUTPUT);
  pinMode(smallUndershoot, OUTPUT); 
  pinMode(largeOvershoot, OUTPUT);

  attachInterrupt(0,doEncoderA,CHANGE);//interrupt pins for encoder
  attachInterrupt(1,doEncoderB,CHANGE);
  Serial.begin(115200);
  // Serial.println("Setting up...");
  pinMode(pwm, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  Timer1.initialize(sampleTime*1000000);         // initialize timer1, and set a 1/2 second period
  // Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(timerISR);  // attaches callback() as a timer overflow interrupt
  digitalWrite(reachPoint,LOW);
  digitalWrite(dir1, LOW);// Stop
  digitalWrite(dir2, LOW);
  digitalWrite(largeUndershoot, LOW);
  digitalWrite(smallOvershoot, LOW);
  digitalWrite(smallError, LOW);
  digitalWrite(smallUndershoot, LOW);
  digitalWrite(largeOvershoot, LOW);

}

void loop(){

  if (change)
  {
    pidCalculation();
     Serial.print(pidTermScaled);
     Serial.print("\t");
     Serial.println(currEncoder0Pos*0.01570795);
    change = false;
  }

  if (angle == setpoint)
  {
    reachPointLight();
    // delay(100);
    lastIntTerm=0;
    intTerm=0;
    lastDerivTerm=0;
    lastError = angle -setpoint;
    Serial.println("A");
  }
  else{

    if (angle < setpoint) {
    
      digitalWrite(dir1, LOW);// Forward motion
      digitalWrite(dir2, HIGH);

      if (error <= 0.0174533){   //1 degrees

        smallErrorLight();
      }

      else if (error < 0.05) //2.864789 degrees
      {
        
        smallUndershootLight();

      }
     
      else { //5.72958 degrees

        largeUndershootLight();
        Serial.println("Large undershoot");
      }

    } 
    else {
      digitalWrite(dir1, HIGH);//Reverse motion
      digitalWrite(dir2, LOW);

      if (abs(error) <= 0.0174533){   // 0.5729578 degrees

        smallErrorLight();

      }

      else if (abs(error) < 0.05) //2.864789 degrees
      {

        smallOvershootLight();

      }
      else { // larger than 5.72958 degrees

        largeOvershootLight();

      }
    }
  }

  analogWrite(pwm, pidTermScaled);

}

void reachPointLight()
{
    digitalWrite(reachPoint, HIGH);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, LOW);
}

void smallErrorLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, HIGH);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, LOW);
}

void smallUndershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, HIGH);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, LOW);
}

void largeUndershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, HIGH);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, LOW);
}

void smallOvershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, HIGH);
    digitalWrite(largeOvershoot, LOW);
}

void largeOvershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, HIGH);
}


void timerISR()
{
  currEncoder0Pos = (encoder0Pos);   //convert to radians
  change = true;
}

void pidCalculation(){
  //angle = (0.9 * encoder0Pos);//encoder0Pos to angle conversion


  //changeError = error - lastError; // derivative term
  angle = currEncoder0Pos * 0.015708;
  error = setpoint - angle;
  intTerm=(lastIntTerm + Ki*error*(sampleTime));

  porTerm=Kp*error;

  derivTerm=1/(1+N*sampleTime)*lastDerivTerm+Kd*N/(1+N*sampleTime)*(error-lastError);
//   derivTerm = (error-lastError)/sampleTime * Kd;
//   derivTerm = (error - lastError) * Kd;
  pidTerm=porTerm+intTerm+derivTerm;

  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTermScaled = abs(pidTerm);//make sure it's a positive value

  lastIntTerm=intTerm;
  lastDerivTerm=derivTerm;
  lastError = error;
  lastAngle=angle;
  change=false;
  // Serial.print("Error: ");
  // Serial.println(error);
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


