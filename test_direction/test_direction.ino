/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#define enA 8
#define in1 9
#define in2 10


#define enB 13
#define in1_B 11
#define in2_B 12



int rotDirection = 0;
int pressed = false;
#define encoder0PinA  2
#define encoder0PinB  3
volatile signed int encoder0Pos = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // pinMode(enB, OUTPUT);
    // pinMode(in1_B, OUTPUT);
    // pinMode(in2_B, OUTPUT);

    // Serial.println("Setting initial direction");
    // digitalWrite(in1, HIGH);
    // digitalWrite(in2, LOW); 

    DDRB = B00010000;
    DDRH = B01000000;
}

void loop() {
    int pwmOutput;
   // Serial.println("Setting PWM signal");
    // int pwmIn = 100; // speed of 3234 RPM

    // pwmOutput = map(pwmIn, 0, 100, 0, 255);
    analogWrite(enA, 100); // Send PWM signal to L298N Enable pin

    PORTB = B00010000;
    PORTH = B00000000;

    delay(1000);

    PORTB = B00000000;
    PORTH = B01000000;

    delay(1000);

}

