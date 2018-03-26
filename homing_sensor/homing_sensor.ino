#define homePin  38 //active low
#define dir1 35
#define dir2 33
#define pwm 9



// byte encoderPos;
bool home;



void setup() {
  Serial.begin(115200);
  pinMode(homePin, INPUT);
pinMode(pwm, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
    digitalWrite(dir1, LOW);// Stop
  digitalWrite(dir2, HIGH);
  analogWrite(pwm, 100);
}

void loop () {
  
  home= digitalRead(homePin);
  Serial.println(home);
  if (home){
    digitalWrite(dir1, LOW);// Stop
    digitalWrite(dir2, LOW);
  }
  else {
    digitalWrite(dir1, LOW);// Stop
    digitalWrite(dir2, HIGH); 
    delay(10);
    digitalWrite(dir1, LOW);// Stop
    digitalWrite(dir2, LOW);
    delay(100);

  }
}