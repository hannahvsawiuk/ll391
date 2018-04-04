void setup() { 
  Serial.begin(9600); 
  }
  

void loop() {
  if (Serial.available() > 0) { /* is a character available?*/
    int rand = random(5,30);
    Serial.write(rand);
    delay(50);
    }
  }
