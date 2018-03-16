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


// byte encoderPos;
byte encoder0Pos = 0;
// int position=0;
bool begin=0;
char pos[8];


void setup() {
  Serial.begin(115200);
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
  
  int pos0 = digitalRead(daPin0);
  int pos1 = digitalRead(daPin1);
  int pos2 = digitalRead(daPin2);
  int pos3 = digitalRead(daPin3);
  int pos4 = digitalRead(daPin4);
  int pos5 = digitalRead(daPin5);
  int pos6 = digitalRead(daPin6);
  int pos7 = digitalRead(daPin7);
  
  
  
  Serial.print(pos0);
  Serial.print(pos1);
  Serial.print(pos2);
  Serial.print(pos3);
  Serial.print(pos4);
  Serial.print(pos5);
  Serial.print(pos6);
  Serial.print(pos7);
  Serial.print("\t");

  
  bitWrite(encoder0Pos, 0, digitalRead(daPin0));
  bitWrite(encoder0Pos, 1, digitalRead(daPin1));
  bitWrite(encoder0Pos, 2, digitalRead(daPin2));
  bitWrite(encoder0Pos, 3, digitalRead(daPin3));
  bitWrite(encoder0Pos, 4, digitalRead(daPin4));
  bitWrite(encoder0Pos, 5, digitalRead(daPin5));
  bitWrite(encoder0Pos, 6, digitalRead(daPin6));
  bitWrite(encoder0Pos, 7, digitalRead(daPin7));


  if (DPin == 0) {
    Serial.print("-");
  } 
    Serial.print(encoder0Pos);
    Serial.print("\t");
    Serial.println(encoder0Pos*0.9);

    delay(200);


}
  