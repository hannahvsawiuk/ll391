#define oePin  46 //active low
#define dirPin 47
//#define indexPin 2
#define reset 49
#define sel1Pin  45
#define sel2Pin  38
#define daPin0   44
#define daPin1   36
#define daPin2  37
#define daPin3   39
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
  pinMode(dirPin, INPUT);
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

  pinMode(reset, OUTPUT);

  digitalWrite(sel1Pin, HIGH);
  digitalWrite(sel2Pin, LOW); //reads lowest 
  digitalWrite(oePin, LOW);

  digitalWrite(reset, LOW);
  delay(100);
  digitalWrite(reset, HIGH);
  
  // pinMode(indexPin, INPUT);
  // pinMode(testPin, INPUT);
  // pinMode(dirPin, INPUT);
  
  
}

void loop () {

  // pos[0] = digitalRead(daPin0);
  // pos[1] = digitalRead(daPin1);
  // pos[2] = digitalRead(daPin2);
  // pos[3] = digitalRead(daPin3);
  // pos[4] = digitalRead(daPin4);
  // pos[5] = digitalRead(daPin5);
  // pos[6] = digitalRead(daPin6);
  // pos[7] = digitalRead(daPin7);

  digitalWrite(sel1Pin, HIGH);
  digitalWrite(sel2Pin, LOW); //reads lowest byte
  // int index = digitalRead(indexPin);
  int dir = digitalRead(dirPin);
  int pos0 = digitalRead(daPin0);
  int pos1 = digitalRead(daPin1);
  int pos2 = digitalRead(daPin2);
  int pos3 = digitalRead(daPin3);
  int pos4 = digitalRead(daPin4);
  int pos5 = digitalRead(daPin5);
  int pos6 = digitalRead(daPin6);
  int pos7 = digitalRead(daPin7);
  
  // delay(5);

  digitalWrite(sel1Pin, LOW);
  digitalWrite(sel2Pin, LOW); //reads second lowest byte 
  
  delayMicroseconds(5);
  
  int pos8 = digitalRead(daPin0);
  int pos9 = digitalRead(daPin1);
  int pos10 = digitalRead(daPin2);
  int pos11 = digitalRead(daPin3);
  int pos12 = digitalRead(daPin4);
  int pos13 = digitalRead(daPin5);
  int pos14 = digitalRead(daPin6);
  int pos15 = digitalRead(daPin7);
  
  
  Serial.print(pos15);
  Serial.print(pos14);
  Serial.print(pos13);
  Serial.print(pos12);
  Serial.print(pos11);
  Serial.print(pos10);
  Serial.print(pos9);
  Serial.print(pos8);
  Serial.print(pos7);
  Serial.print(pos6);
  Serial.print(pos5);
  Serial.print(pos4);
  Serial.print(pos3);
  Serial.print(pos2);
  Serial.print(pos1);
  Serial.print(pos0);

  Serial.print("\t");

  
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


    Serial.print(encoder0Pos);
    Serial.print("\t");
    Serial.println(encoder0Pos*0.9);
    // Serial.print("\t Index: ");
    // Serial.print(index);
    // Serial.print("\t Direction: ");
    // Serial.println(dir);
    // Serial.print("\t");
    // Serial.println((~encoder0Pos+1)*0.9);
// 

    delay(200);


}
  