#include <iostream>
using namespace std;

struct encoder
{
    int oePin;   //active low
    int DPin; 
    int sel1Pin;
    int sel2Pin;
    int daPin0;
    int daPin1;
    int daPin2;
    int daPin3;
    int daPin4;
    int daPin5;
    int daPin6;
    int daPin7;
    int index;
}

struct led
{
    int largeUndershoot;
    int smallUndershoot ;
    int smallError;
    int smallOvershoot;
    int largeOvershoot;
}

struct driver
{
    int dir1;
    int dir2;
    int pwm;
}

struct pid
{
    float Kp;
    float Ki;
    float Kd; 
    float N; 
}

struct homing
{
    int trigger;
    bool home;
}


class Memory
{
    private:
        float curr;
        float last;
    
    public:
         Memory(){};
         ~Memory();
         float getCurr() { return curr; };
         float getLast() { return last; };
         void setCurr(float currentTerm) { curr = currentTerm; };
         void setLast(float lastTerm) { last = lastTerm; };

} 

class Motor
{
    private:
        int pos;
        Memory error;
        Memory angle;
        Memory prop;
        Memory integ;
        Memory deriv;
        float pidTerm;
        encoder encoder;
        led led;
        driver driver;
        homing homing;

    public:
        Motor(){};
        ~Motor();
    
}
