/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#define enA 9
#define in1 6
#define in2 7

int rotDirection = 0;
int pressed = false;
void setup() {
    Serial.begin(9600);
    Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    // Set initial rotation direction
    Serial.println("Setting initial direction");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}
void loop() {
    // int potValue = analogRead(A0); // Read potentiometer value
    // int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
    Serial.println("Setting PWM signal");
    int pwmOutput = 75;
    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
    Serial.println("Sent PWM Signal");
   
    // if ( rotDirection == 0) {
    //     digitalWrite(in1, HIGH);
    //     digitalWrite(in2, LOW);
    //     delay(5000);
    // }
    // else
    // {
    //     digitalWrite(in1, LOW);
    //     digitalWrite(in2, HIGH);
    //     delay(5000);
    // }
    Serial.println("Now changing to backward rotation");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    delay(2000);
    Serial.println("Rotating backwards");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    Serial.println("Set both low");
    // delay(500);
    Serial.println("Now changing to forward rotation");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Serial.println("Rotating forwards");
    delay(2000);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    Serial.println("Set both low");
    // delay(500);


}
