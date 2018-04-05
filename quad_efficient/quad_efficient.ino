#include <TimerOne.h>   
#define oePin1  46 
#define reset1 49

#define oePin2 32
#define reset2 35


#define enA 8
#define in1 9
#define in2 10


// byte encoderPos;
int encoder0Pos = 0;

// int position=0;
bool begin=0;
// char pos[8];
bool change = false;
volatile int encoderPos = 0;
volatile int pos;
volatile int start = 0;
volatile int speed = 0;
void setup() {
  Serial.begin(115200);


//   pinMode(sel1Pin, OUTPUT);//encoder pins
//   pinMode(sel2Pin, OUTPUT);//encoder pins
  
  pinMode(oePin1, OUTPUT);

  pinMode(reset1, OUTPUT);

  digitalWrite(reset1, LOW);
  delay(100);
  digitalWrite(reset1, HIGH);

    pinMode(oePin2, OUTPUT);

  pinMode(reset2, OUTPUT);

  digitalWrite(reset2, LOW);
  delay(100);
  digitalWrite(reset2, HIGH);
    
    
    // qd1
    DDRC = B01100100;
    DDRL = B00011001;
    DDRG = B00000100;
    DDRD = B00000000;
    
    //qd2
    DDRA = B00001000;
    
 pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
 analogWrite(enA, 255); // Send PWM signal to L298N Enable pin
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH); 
    Timer1.initialize(3000);  // 3ms          // initialize timer1, and set a 1/2 second period
    // Timer1.attachInterrupt(qd1);  // attaches callback() as a timer overflow interrupt
     Timer1.attachInterrupt(qd2);  // attaches callback() as a timer overflow interrupt
 
}

void loop () {

    if (change)
    {
        // qd1();
        // Serial.print(start);
        // Serial.print("\t");
        // Serial.print(pos);
        // Serial.print("\t");
        // Serial.println((pos - start)*0.833*60);
        // start = pos;
        // change = false;
        Serial.println(pos);
    }

}

void timerISR()
{
    change = true;
}

void qd1()
{
    // int pos = 0;
    // long unsigned start = micros();

    pos = 0;
    // set SEL1 HIGH and SEL2 LOW
    // RESET is always high
    PORTL = B00010001;
    PORTG = B00000000;

    // write lower bits to pos

    pos = (PINL & B00100000) >> 5 |
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

    // write higher bits to pos

    pos = pos |
          ((PINL & B00100000) >> 5) << 8 |
          ((PINC & B00000010)) << 8 |
          ((PINC & B00000001) << 2) << 8 |
          ((PIND & B10000000) >> 4) << 8 |
          ((PING & B00000010) << 3) << 8 |
          ((PING & B00000001) << 5) << 8 |
          ((PINL & B10000000) >> 1) << 8 |
          ((PINL & B00000010) << 6) << 8; 
    
        // Serial.print("\t");
    // Seria                                                                                                                                  l.println(pos);
    // speed = pos*0.8333*60;
    change = true;
    // PORTB = B00000010;
    // PORTB = B00000000;
    // Serial.println(pos);

    // Serial.print("\t");

}

void qd2()
{
    pos = 0;

    // set SEL1 HIGH and SEL2 LOW
    // RESET is always high
    PORTC = B01000100;
    PORTG = B00000000;

    // write lower bits to pos

    pos = (PINC & B10000000) >> 7 |
          (PINA & B00000001) << 1 |
          (PINA & B00000010) << 1 |
          (PINA & B00000100) << 1 |
          (PINA & B00010000)      |
          (PINA & B00100000)      |
          (PINA & B01000000)      |
          (PINC & B00001000) << 4; 
    
    // long unsigned end = micros();
    // Serial.print(end - start);

        // set SEL1 LOW and SEL2 LOW
        // RESET is always high
    PORTC = B00000100;
    PORTG = B00000000;

    // write higher bits to pos

    pos = pos |
          ((PINC & B10000000) >> 7) << 8 |
          ((PINA & B00000001) << 1) << 8 |
          ((PINA & B00000010) << 1) << 8 |
          ((PINA & B00000100) << 1) << 8 |
          ((PINA & B00010000)     ) << 8 |
          ((PINA & B00100000)     ) << 8 |
          ((PINA & B01000000)     ) << 8 |
          ((PINC & B00001000) << 4) << 8; 
    
        // Serial.print("\t");
    // Seria                                                                                                                                  l.println(pos);
    // speed = pos*0.8333*60;
    change = true;
    // PORTB = B00000010;
    // PORTB = B00000000;
    // Serial.println(pos);

    // Serial.print("\t");

}

