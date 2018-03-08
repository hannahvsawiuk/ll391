// double count = 0; //set the counts of the encoder
// double angle = 0;//set the angles
// boolean A,B;
// byte state, statep;

// double pwm = 9;// this is the PWM pin for the motor for how much we move it to correct for its error
// const int dir1 = 12;//these pins are to control the direction of the motor (clockwise/counter-clockwise)
// const int dir2 = 11;

// double setpoint = 90;
// double min_setpoint = 91;//I am setting it to move through 100 degrees
// double max_setpoint = 89;
// double Kp = 0.32;// you can set these constants however you like depending on trial & error
// double Ki = 0.1;
// double Kd = 0.3;

// float last_error = 0;
// float error = 0;
// float changeError = 0;
// float totalError = 0;
// float pidTerm = 0;
// float pidTerm_scaled = 0;// if the total gain we get is not in the PWM range we scale it down so that it's not bigger than |255|
// bool change = false;
// long unsigned accum_time;

// void setup() {
//   Serial.begin(38400);
//   pinMode(2, INPUT);//encoder pins
//   pinMode(3, INPUT);
//   attachInterrupt(0,Achange,CHANGE);//interrupt pins for encoder
//   attachInterrupt(1,Bchange,CHANGE); 
  
//   pinMode(pwm, OUTPUT);
//   pinMode(dir1, OUTPUT);
//   pinMode(dir2, OUTPUT);


// }

// void loop(){
  
//       if (millis() % 7 == 0)
//       {
//         accum_time = millis();
//         Serial.print(accum_time);
//         Serial.print("\t\t");
//         Serial.println(count*0.9);
//       }
// }

  
// void Achange() //these functions are for finding the encoder counts
// {
//   A = digitalRead(2);
//   B = digitalRead(3);

//   if ((A==HIGH)&&(B==HIGH)) state = 1;
//   if ((A==HIGH)&&(B==LOW)) state = 2;
//   if ((A==LOW)&&(B==LOW)) state = 3;
//   if((A==LOW)&&(B==HIGH)) state = 4;
//   switch (state)
//   {
//     case 1:
//     {
//       if (statep == 2) count++;
//       if (statep == 4) count--;
//       break;
//     }
//     case 2:
//     {
//       if (statep == 1) count--;
//       if (statep == 3) count++;
//       break;
//     }
//     case 3:
//     {
//       if (statep == 2) count --;
//       if (statep == 4) count ++;
//       break;
//     }
//     default:
//     {
//       if (statep == 1) count++;
//       if (statep == 3) count--;
//     }
//   }
//   statep = state;
//   change = true;
// }

// void Bchange()
// {
//   A = digitalRead(2);
//   B = digitalRead(3);

//   if ((A==HIGH)&&(B==HIGH)) state = 1;
//   if ((A==HIGH)&&(B==LOW)) state = 2;
//   if ((A==LOW)&&(B==LOW)) state = 3;
//   if((A==LOW)&&(B==HIGH)) state = 4;
//   switch (state)
//   {
//     case 1:
//     {
//       if (statep == 2) count++;
//       if (statep == 4) count--;
//       break;
//     }
//     case 2:
//     {
//       if (statep == 1) count--;
//       if (statep == 3) count++;
//       break;
//     }
//     case 3:
//     {
//       if (statep == 2) count --;
//       if (statep == 4) count ++;
//       break;
//     }
//     default:
//     {
//       if (statep == 1) count++;
//       if (statep == 3) count--;
//     }
//   }
//   change = true;
//   statep = state;
  
// }

/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#define enA 9
#define in1 12
#define in2 11
#define motorIn 5

int rotDirection = 0;
int pressed = false;
#define encoder0PinA  2
#define encoder0PinB  3
volatile unsigned int encoder0Pos = 0;
long unsigned accum_time;
// int voltage = 0;

void setup() {
    Serial.begin(38400);
    // Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(motorIn, INPUT);

    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);

    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doEncoderA, CHANGE);

    // encoder pin on interrupt 1 (pin 3)
    attachInterrupt(1, doEncoderB, CHANGE);
    // Set initial rotation direction
    // Serial.println("Setting initial direction");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    
}
void loop() {
    // int pwmOutput;
    // double voltage;
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);
    // int voltage;
   // Serial.println("Setting PWM signal");
    // int pwmIn = 0; // speed of 3234 RPM

    //pwmOutput = map(pwmIn, 0, 100, 0, 255);
  
  //  Serial.println("Sent PWM Signal");
  //  Serial.println(encoder0Pos);
   
   // Serial.println("Now changing to backward rotation");
    // digitalWrite(in1, HIGH);
    // digitalWrite(in2, LOW);

  
      if (millis() % 7 == 0)
      {
        accum_time = millis();
        Serial.print(accum_time);
        Serial.print("\t");
        Serial.println(encoder0Pos);
      }

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


