#include <Math.h>
#define encoder0PinA  2
#define encoder0PinB  3

// double encoder0Pos = 0; //set the encoder0Poss of the encoder
double encoder0Pos = 0;
double angle = 0;//set the angles
boolean A,B;
byte state, statep;
double setpoint = 1;
int eval_setpoint;
int eval_angle;

double pwm = 9;// this is the PWM pin for the motor for how much we move it to correct for its error
const int dir1 = 11;//these pins are to control the direction of the motor (clockwise/encoder0Poser-clockwise)
const int dir2 = 12;

double min_setpoint = 85;//I am setting it to move through 100 degrees
double max_setpoint = 95;
// double Kp = 0.32;// you can set these constants however you like depending on trial & error
// double Ki = 0.1;
// double Kd = 0.3;

double Kp= 1; //1*45/(3.14159/4)*(12/45)*(255/12) PWM/rad
double Ki =0;
double Kd = 0;             


float last_error = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|
bool change = false;
float deriv_error=0;
volatile unsigned long accum_time;
volatile unsigned long last_time;
volatile unsigned long curr_time;

void setup() {
  Serial.begin(115200);
  pinMode(encoder0PinA, INPUT);//encoder pins
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(0,doEncoderA,CHANGE);//interrupt pins for encoder
  attachInterrupt(1,doEncoderB,CHANGE); 
  
  // Serial.println("Setting up...");
  // pinMode(pwm, OUTPUT);
  // pinMode(dir1, OUTPUT);
  // pinMode(dir2, OUTPUT);

}

void loop(){
  
  // PIDcalculation();// find PID value
 
  angle = (encoder0Pos)*3.14159/200;   //convert to radians
  error = angle - setpoint;
  // pidTerm_scaled = 255;
  pidTerm = error/(255/(3.14159/2));
  pidTerm_scaled = abs(constrain(pidTerm, -255, 255));
  if (angle==setpoint)
  {
    digitalWrite(dir1, LOW);// Stop
    digitalWrite(dir2, LOW);
  }
  else if (angle < setpoint) {
    digitalWrite(dir1, LOW);// Forward motion
    digitalWrite(dir2, HIGH);
  } else {
    digitalWrite(dir1, HIGH);//Reverse motion
    digitalWrite(dir2, LOW);
  }

  analogWrite(pwm, pidTerm_scaled);

  if (change)
  {
    long unsigned curr_time = millis();
    Serial.print(curr_time);
    Serial.print("\t");
    Serial.print(pidTerm_scaled);
    Serial.print("\t");
    Serial.print(error);
    Serial.print("\t");
    Serial.println(angle);
    change = false;
  }
    // if (millis() % 3 == 0)
    // {
    //   accum_time = millis();
    //   Serial.print(accum_time);
    //   Serial.print("\t");
    //   Serial.println(encoder0Pos);
    // }


}

void PIDcalculation(){
  //angle = (0.9 * encoder0Pos);//encoder0Pos to angle conversion

  angle = (encoder0Pos)*3.14159/200;   //convert to radians
  error = setpoint - angle;
  // curr_time= millis();
  // changeError = error - last_error; // derivative term
  // totalError += error; //accumulate errors to find integral term
  // deriv_error=changeError/(double(curr_time-last_time))*1000;

  // // pidTerm = (Kp * error) + (Ki * totalError) + (Kd * deriv_error);//total gain
  
  // pidTerm=(Kp*error+Kd*deriv_error)*255/(3.14159/4);
  pidTerm = error*255/(3.14159/4);
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_time=curr_time;
  last_error = error;
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
  change = true;
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
  change = true;
}


