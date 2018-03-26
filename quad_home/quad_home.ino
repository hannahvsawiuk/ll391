#include <Math.h>
#include <TimerOne.h>
#define oePin  4 //active low
#define DPin 5
#define sel1Pin  13
#define sel2Pin  12
#define daPin0   22
#define daPin1   23
#define daPin2  24
#define daPin3   25
#define daPin4   26
#define daPin5   27
#define daPin6   28
#define daPin7   29

#define index_in 31
#define index_out 30

#define reachPoint 13
#define largeUndershoot 3
#define smallUndershoot 10
#define smallError 8
#define smallOvershoot 7
#define largeOvershoot 6

#define dir1 35
#define dir2 33
#define pwm 9

// byte encoderPos;
 int encoder0Pos = 0;

// int position=0;
bool begin=0;
// char pos[8];

// float encoder0Pos = 0; //set the encoder0Poss of the encoder
// volatile float encoder0Pos = 0;
float angle = 0;//set the angles
float lastAngle=0;
// boolean A,B;
// byte state, statep;
float desired = 1.58;

float setpoint = desired;
// float setpoint = waypoints[i];
// float currEncoder0Pos;

float waypoints[] = { 3.14159/8, 2*3.14159/8, 3*3.14159/8, 4*3.14159/8, 
                      3*3.14159/8, 2*3.14159/8, 3.14159/8,
                      0, -3.14159/8, 2*-3.14159/8, 3*-3.14159/8, 4*-3.14159/8,
                      3*-3.14159/8, 2*-3.14159/8, -3.14159/8, 0};

                      



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

// float Kp = 207.5;
// float Ki = 30;
// float Kd = 16.5; 
// float N=100;         //vibrates / WORKS


// float Kp = 155;
// float Ki = 0;
// float Kd = 30; 
// float N=100;   

// float Kp = 210;
// float Ki = 0;    //works for 45 degrees
// float Kd =22; 
// float N=100;  



float Kp = 205;
float Ki = 0;    
float Kd =22;
float N=100;  

// float Kp = 209;
// float Ki = 0;
// float Kd =21.8; 
// float N=100;  
 


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
bool homing = false;
// int i;
void setup() {
    
    Serial.begin(115200);

    pinMode(reachPoint, OUTPUT); 
    pinMode(smallOvershoot, OUTPUT);
    pinMode(largeOvershoot, OUTPUT); 
    pinMode(smallError, OUTPUT);
    pinMode(smallUndershoot, OUTPUT); 
    pinMode(largeOvershoot, OUTPUT);

  
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeOvershoot, LOW);



    Timer1.initialize(sampleTime*1000000);         // initialize timer1, and set a 1/2 second period
    // Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
    Timer1.attachInterrupt(timerISR);  // attaches callback() as a timer overflow interrupt
    digitalWrite(reachPoint,LOW);


    pinMode(DPin, INPUT);
    pinMode(daPin0, INPUT);//encoder pins
    pinMode(daPin1, INPUT);//encoder pins
    pinMode(daPin2, INPUT);//encoder pins
    pinMode(daPin3, INPUT);//encoder pins
    pinMode(daPin4, INPUT);//encoder pins
    pinMode(daPin5, INPUT);//encoder pins
    pinMode(daPin6, INPUT);//encoder pins
    pinMode(daPin7, INPUT);//encoder pins


    pinMode(sel1Pin, OUTPUT);//encoder pins
    pinMode(sel2Pin, OUTPUT);//encoder pins
    pinMode(oePin, OUTPUT);

    digitalWrite(sel1Pin, HIGH);
    digitalWrite(sel2Pin, LOW); //reads lowest 
    digitalWrite(oePin, LOW);

    pinMode(index_in, INPUT);
    pinMode(index_out, OUTPUT);

        // Serial.println("Setting up...");
    pinMode(pwm, OUTPUT);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);


    analogWrite(pwm, 255);
    digitalWrite(dir1, LOW);// Stop
    digitalWrite(dir2, LOW);

    i = 0;
    // // Serial.println("Setting up decoder...");
    // digitalWrite(index_out, HIGH);
    // delay(500);
    // digitalWrite(index_out, LOW);
    // digitalWrite(dir1, LOW);// Stop
    // digitalWrite(dir2, LOW);
    // analogWrite(pwm, 0);

    
}

void loop(){

  // if (change)
  // {
  //   getEncoderPos();
  //   pidCalculation();
  //   Serial.print(pidTermScaled);
  //   Serial.print("\t");
  //   Serial.println(encoder0Pos*0.9);
  //   change = false;
  // }
      while(homing == false) {
        if (digitalRead(index_in) == 0)
        {
                homing = true;
                digitalWrite(index_out, HIGH);
                delay(500);
                digitalWrite(index_out, LOW);
                digitalWrite(dir1, LOW);// Stop
                digitalWrite(dir2, LOW);
        }

        digitalWrite(dir1, LOW);// Stop
        digitalWrite(dir2, HIGH); 
        delay(10);
        Serial.println("LH");
        // Serial.print(dir1);
        // Serial.print("\t");
        // Serial.print(dir2);
        digitalWrite(dir1, LOW);// Stop
        digitalWrite(dir2, LOW);
        Serial.println("LL");
        // Serial.print(dir1);
        // Serial.print("\t");
        // Serial.println(dir2);
        delay(100);
        // Serial.println("Homing...");       
    }


        setpoint = waypoints[i];
        // getEncoderPos();
        // Serial.println(encoder0Pos);
        pidCalculation();
        // Serial.print(pidTermScaled);
        // Serial.print("\t");
        // Serial.println(encoder0Pos*0.9);



    // getEncoderPos();
    // Serial.print(pidTermScaled);
    // Serial.print("\t");
    // Serial.println(encoder0Pos*0.9);
    if (angle == setpoint)
    {
        reachPointLight();
        // delay(100);
        // lastIntTerm=0;
        // intTerm=0;
        // lastDerivTerm=0;
        //lastError = angle -setpoint;
        //Serial.println("A");
        //setpoint = -setpoint;
        digitalWrite(dir1, LOW);// Forward motion
        digitalWrite(dir2, LOW);
        //delay(1000);
    }

    else if (abs(angle - setpoint) <= 0.0174533){   //1 degrees

            smallErrorLight();
            digitalWrite(dir1, LOW);// Forward motion
            digitalWrite(dir2, LOW);
            // setpoint = -setpoint;
            // delay(1000);

            lastIntTerm=0;
            intTerm=0;
            lastDerivTerm=0;
            i++;
            if (i == 16)
            {
                i = 0;
            }

            // Serial.print(setpoint);
            // Serial.print("\t");
            //Serial.println(encoder0Pos*0.9);

            //delay(50);



        }

    else{

        if (angle < setpoint) {
        
        digitalWrite(dir1, LOW);// Forward motion
        digitalWrite(dir2, HIGH);

        //   if (error <= 0.0174533){   //1 degrees

        //     smallErrorLight();
        //     //setpoint = -0.79;
        //     //delay(2000);
        //   }

        if (error < 0.05) //2.864789 degrees
        {
            
            smallUndershootLight();
        

        }
        
        else { //5.72958 degrees

            largeUndershootLight();
            // Serial.println("Large undershoot");
        }

        } 
        else {
        digitalWrite(dir1, HIGH);//Reverse motion
        digitalWrite(dir2, LOW);

        //   if (abs(error) <= 0.0174533){   // 0.5729578 degrees

        //     smallErrorLight();

        //   }

        if (abs(error) < 0.05) //2.864789 degrees
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
  // getEncoderPos();   //convert to radians
  getEncoderPos();
}

void pidCalculation(){
  //angle = (0.9 * encoder0Pos);//encoder0Pos to angle conversion


  //changeError = error - lastError; // derivative term
  //angle = encoder0Pos * 0.015708;
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

void getEncoderPos()
{
    digitalWrite(sel1Pin, HIGH);
    digitalWrite(sel2Pin, LOW); //reads lowest byte
    
    // delayMicroseconds(5);

    int pos0 = digitalRead(daPin0);
    int pos1 = digitalRead(daPin1);
    int pos2 = digitalRead(daPin2); 
    int pos3 = digitalRead(daPin3);
    int pos4 = digitalRead(daPin4);
    int pos5 = digitalRead(daPin5);
    int pos6 = digitalRead(daPin6);
    int pos7 = digitalRead(daPin7);

    // delayMicroseconds(5);

    digitalWrite(sel1Pin, LOW);
    digitalWrite(sel2Pin, LOW); //reads second lowest byte 

    int pos8 = digitalRead(daPin0);
    int pos9 = digitalRead(daPin1);
    int pos10 = digitalRead(daPin2);
    int pos11 = digitalRead(daPin3);
    int pos12 = digitalRead(daPin4);
    int pos13 = digitalRead(daPin5);
    int pos14 = digitalRead(daPin6);
    int pos15 = digitalRead(daPin7);

    bitWrite(encoder0Pos, 0, pos0);
    bitWrite(encoder0Pos, 1, pos1);
    bitWrite(encoder0Pos, 2, pos2);
    bitWrite(encoder0Pos, 3, pos3);
    bitWrite(encoder0Pos, 4, pos4);
    bitWrite(encoder0Pos, 5, pos5);
    bitWrite(encoder0Pos, 6, pos6);
    bitWrite(encoder0Pos, 7, pos7);
    bitWrite(encoder0Pos, 8, pos8);
    bitWrite(encoder0Pos, 9, pos9);
    bitWrite(encoder0Pos, 10, pos10);
    bitWrite(encoder0Pos, 11, pos11);
    bitWrite(encoder0Pos, 12, pos12);
    bitWrite(encoder0Pos, 13, pos13);
    bitWrite(encoder0Pos, 14, pos14);
    bitWrite(encoder0Pos, 15, pos15);
    angle = encoder0Pos * 0.015708;

    

   //Serial.println(encoder0Pos*0.9);

}