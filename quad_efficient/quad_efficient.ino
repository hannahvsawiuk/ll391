#define oePin  46 //active low
#define dirPin 47
//#define indexPin 2
#define reset 49
#define sel1Pin  45
#define sel2Pin  39
#define daPin0   44
#define daPin1   36
#define daPin2  37
#define daPin3   38
#define daPin4   40
#define daPin5   41
#define daPin6   42
#define daPin7   48


// byte encoderPos;
int encoder0Pos = 0;

// int position=0;
bool begin=0;
// char pos[8];


void setup() {
  Serial.begin(115200);


//   pinMode(sel1Pin, OUTPUT);//encoder pins
//   pinMode(sel2Pin, OUTPUT);//encoder pins
  
  pinMode(oePin, OUTPUT);

  pinMode(reset, OUTPUT);

  digitalWrite(sel1Pin, HIGH);
  digitalWrite(sel2Pin, LOW); //reads lowest 
  digitalWrite(oePin, LOW);

  digitalWrite(reset, LOW);
  delay(100);
  digitalWrite(reset, HIGH);



  
  
}

void loop () {

    // int pos = qd1();
    // Serial.print(pos);
    // Serial.print("\t");
    // Serial.println(pos*0.9);
    
    qd1();
    

    

    //


    delay(10);


}

void qd1()
{
    int pos = 0;
    // long unsigned start = micros();
    DDRC = B01100100;
    DDRL = B00011001;
    DDRG = B00000100;
    DDRD = B00000000;

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
    Serial.println(pos*0.9);
    
 
    // Serial.print("\t");

}



