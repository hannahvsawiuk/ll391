#include <TimerOne.h>   

#define oePin  46 //active low
#define reset 49

#define enA 8
#define in1 9
#define in2 10

const float sampleTime = 1000; // us

const float scaleGains = 2*3.14159/400;
const float scaleAngle = 400/(2*3.1459);

const float KP_X = 200*scaleGains;
const float KD_X = 30*scaleGains;
const float N_X = 1;
const float LASTD_MULT = 1/(1 + N_X*sampleTime);
const float D_MULT = N_X/(1 + N_X*sampleTime);

int posX;
int desiredX = 45.1415/180*scaleAngle;
int errorX;
int last_errorX;
float pX;
float dX;
float last_dX;
float pidX;

bool change = false;


void setup() {
  Serial.begin(115200);

  pinMode(oePin, OUTPUT);
  pinMode(reset, OUTPUT);

  digitalWrite(oePin, LOW);
  digitalWrite(reset, LOW);
  delay(100);
  digitalWrite(reset, HIGH);
  
  // Add homing function

    // Port registers
    DDRC = B01100100;
    DDRL = B00011001;
    DDRG = B00000100;
    DDRD = B00000000;

    DDRB = B00010000;
    DDRH = B01000000;

    Timer1.initialize(1000);  // 3ms  // initialize timer1, and set a 1/2 second period
    Timer1.attachInterrupt(control);  // attaches callback() as a timer overflow interrupt
    
    analogWrite(enA, 0);              // Send PWM signal to L298N Enable pin
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW); 
  
}

void loop () {

    if (change)
    {
        noInterrupts();

        // Get position

        posX = 0;
        // set SEL1 HIGH and SEL2 LOW
        // RESET is always high
        PORTL = B00010001;
        PORTG = B00000000;

        // write lower bits to posX

        posX = (PINL & B00100000) >> 5 |
                (PINC & B00000010) |
                (PINC & B00000001) << 2 |
                (PIND & B10000000) >> 4 |
                (PING & B00000010) << 3 |
                (PING & B00000001) << 5 |
                (PINL & B10000000) >> 1 |
                (PINL & B00000010) << 6; 

        // long unsigned end = micros();
        // Serial.print(end - start);

            // set SEL1 LOW and SEL2 LOW
            // RESET is always high
        PORTL = B00000001;
        PORTG = B00000000;

        // write higher bits to posX

        posX = posX |
                ((PINL & B00100000) >> 5) << 8 |
                ((PINC & B00000010)) << 8 |
                ((PINC & B00000001) << 2) << 8 |
                ((PIND & B10000000) >> 4) << 8 |
                ((PING & B00000010) << 3) << 8 |
                ((PING & B00000001) << 5) << 8 |
                ((PINL & B10000000) >> 1) << 8 |
                ((PINL & B00000010) << 6) << 8; 

        Serial.print(posX);
        // Perform PID calculation
        errorX = desiredX - posX;
        pX = KP_X*errorX;
        dX = LASTD_MULT*last_dX + KD_X*D_MULT*(errorX - last_errorX);
        pidX = pX + dX;

        last_dX = dX;
        last_errorX = errorX;
        // Take absolute value of pidterm and constrain

        if (pidX < 0)
        {
            pidX = -1*pidX;
        }

        if (pidX > 255)
        {
            pidX = 255;
        }

        Serial.print("\t");
        Serial.println(pidX);

        // Write values to motor
        analogWrite(enA, pidX); // TODO: optimize

        if (posX < desiredX)
        {
            PORTB = B00010000;
            PORTH = B00000000; // in2 low
            // digitalWrite(in1, HIGH);
            // digitalWrite(in2, LOW); 
        }
        else
        {
            PORTB = B00000000;
            PORTH = B01000000; // in2 HIGH
            // digitalWrite(in1, LOW);
            // digitalWrite(in2, HIGH); 
        }

        change = false;

        interrupts();
       
    }

}

void control()
{
    change = true;
}

