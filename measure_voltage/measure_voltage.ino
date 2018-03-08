/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#define enA 9
#define in1 12
#define in2 11
#define motorIn 5

int rotDirection = 0;
int pressed = false;
#define encoder0PinA  2
#define encoder0PinB  3
volatile unsigned int encoder0Pos = 0;
// int voltage = 0;


void setup() {
    Serial.begin(9600);
    Serial.println("Setting up pins");
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(motorIn, INPUT);
    Serial.println("Setting initial direction");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    
}
void loop() {
    // int pwmOutput;
    double voltage;
    // int voltage;

    int pwmOutput =50;
    pwmOutput = map(pwmOutput, 0, 100, 0, 255);

    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin


    // for (int i = 0; i <= 255; i++)
    // {
    //   pwmOutput = i;
    //   analogWrite(enA, pwmOutput);
    //   delay (200);
    //   voltage = 0.0049*analogRead(motorIn);
    //   // voltage = analogRead(motorIn);
    //   delay(300);
    //   Serial.print(pwmOutput);
    //   Serial.print("\t");
    //   Serial.println(voltage);
    //   // Serial.println("\n");
    // }

}


