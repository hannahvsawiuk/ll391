
#include "Encoder.h"
#include "LEDS.h"
// #include "Driver.h"

using namespace std;


struct driver
{
    int dir1;
    int dir2;
    int pwm;
};

struct pid
{
    float Kp;
    float Ki;
    float Kd; 
    float N; 
};

struct homing
{
    int trigger;
    bool home;
};


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

}; 

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
        // driver driver;
        homing homing;

    public:
        Motor(){};
        ~Motor();

        Encoder encoder;
        Leds led;
        

    
};