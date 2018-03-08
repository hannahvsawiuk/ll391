#include <Math.h>
#define encoder0PinA  2
#define encoder0PinB  3
#define reach_point 13
#include <TimerOne.h>

// float encoder0Pos = 0; //set the encoder0Poss of the encoder
float encoder0Pos = 0;
float angle = 0;//set the angles
float last_angle=0;
// boolean A,B;
// byte state, statep;
float setpoint = 3.14159/4;
float curr_encoder0Pos;



float pwm = 9;// this is the PWM pin for the motor for how much we move it to correct for its error
const int dir1 = 12;//these pins are to control the direction of the motor (clockwise/encoder0Poser-clockwise)
const int dir2 = 11;

// float min_setpoint = 85;//I am setting it to move through 100 degrees
// float max_setpoint = 95;
// float Kp = 0.32;// you can set these constants however you like depending on trial & error
// float Ki = 0.1;
// float Kd = 0.3;
float sample_time=0.002;

// float Kp = 200;
// float Ki = 40;
// float Kd = 18; 
// float N=100;         //vibrates

// volatile int accum0Pos = 0;
// volatile int speed = 0;      

// float Kp = 206;
// float Ki = 40;
// float Kd = 20; 
// float N=100;          reaches point but too slow

// float Kp = 207;
// float Ki = 40;
// float Kd = 30; 
// float N=100;  

// float Kp = 205.101;
// float Ki = 0;
// float Kd = 11.7; 
// float N=100;        settles at smaller angle

float Kp = 210;
float Ki = 40;
float Kd = 15.1; 
float N=100;  



float last_error = 0;
float error = 0;

// float changeError = 0;
// float totalError = 0;
float pidTerm = 0;
float pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|
bool change = false;
float last_int_term=0;
float int_term=0;
float last_deriv_term=0;
float deriv_term=0;
float por_term=0;

volatile unsigned long accum_time;
volatile unsigned long last_time;
volatile unsigned long curr_time;

void setup() {
  Serial.begin(115200);
  pinMode(encoder0PinA, INPUT);//encoder pins
  pinMode(encoder0PinB, INPUT);
  pinMode(reach_point, OUTPUT); 
  attachInterrupt(0,doEncoderA,CHANGE);//interrupt pins for encoder
  attachInterrupt(1,doEncoderB,CHANGE); 
  
  // Serial.println("Setting up...");
  // pinMode(pwm, OUTPUT);
  // pinMode(dir1, OUTPUT);
  // pinMode(dir2, OUTPUT);
  Timer1.initialize(sample_time*1000000);         // initialize timer1, and set a 1/2 second period
  // Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(timer_ISR);  // attaches callback() as a timer overflow interrupt
  digitalWrite(reach_point,LOW);

}

void loop(){
  // find PID value

  if (change)
  {
    PIDcalculation();
    // Serial.print(pidTerm_scaled);
    // Serial.print("\t");
    // Serial.println(curr_encoder0Pos*0.9);
    change = false;
  }


  if (angle==setpoint)
  {
    digitalWrite(dir1, LOW);// Stop
    digitalWrite(dir2, LOW);
    digitalWrite(reach_point,HIGH);
  }
  else{
    digitalWrite(reach_point,LOW);
    if (angle < setpoint) {
      digitalWrite(dir1, LOW);// Forward motion
      digitalWrite(dir2, HIGH);
  } else {
    digitalWrite(dir1, HIGH);//Reverse motion
    digitalWrite(dir2, LOW);
    }
  }

  analogWrite(pwm, pidTerm_scaled);


}

void timer_ISR()
{
  curr_encoder0Pos = (encoder0Pos);   //convert to radians
  change = true;
}

void PIDcalculation(){
  //angle = (0.9 * encoder0Pos);//encoder0Pos to angle conversion


  //changeError = error - last_error; // derivative term
  angle = curr_encoder0Pos * 0.015708;
  error = setpoint - angle;
  int_term=(last_int_term + Ki*error*(sample_time));

  por_term=Kp*error;

  deriv_term=1/(1+N*sample_time)*last_deriv_term+Kd*N/(1+N*sample_time)*(error-last_error);

  pidTerm=por_term+int_term+deriv_term;

  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  last_int_term=int_term;
  last_deriv_term=deriv_term;
  last_error = error;
  last_angle=angle;
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


