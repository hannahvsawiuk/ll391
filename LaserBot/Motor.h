#include <Arduino.h>
#include "Encoder.h"
#include "LEDS.h"
#include "Driver.h"
#include "PID.h"

using namespace std;

struct homing
{
    int trig;
    bool index;
};

class Motor: public Encoder, public Driver, public Leds, public PID
{
    private:
        homing homing;
        int homingSpeed;
    public:
        Motor(){};
        Motor(float P, float I, float D, float filter, float t)
        : PID(P, I, D, filter, t) {};
        
        ~Motor(){};

        bool control();
        bool home();

        void setIndex(int pin)          { homing.trig = pin; };
        void setHomingSpeed (int speed) { homingSpeed = speed; };    
};


bool Motor::control()
{
    float error = angle - desired;
    bool reached;

    if (abs(error) <= resolution)
    {   //1 degrees

            smallErrorLight();
            stop();
            reached = true;
    }

    else if (angle < desired) 
    {
        bool reached = false;
        forward();

        if (error < 2.5*resolution) //2.864789 degrees
        {
            smallUndershootLight();
        }
        else 
        { //5.72958 degrees
            largeUndershootLight();
        } 
    }
    else {
        reached = false;
        reverse();

        if (abs(error) < 2.5*resolution) //2.864789 degrees
        {

            smallOvershootLight();

        }
        else { // larger than 5.72958 degrees

            largeOvershootLight();

        }
    }
    setPWM(pidTerm);
    return reached;
}

bool Motor::home()
{
    bool origin = false;
    setPWM(homingSpeed);
    while (origin == false)
    {
        if (digitalRead(homing.trig) == 0)
        {
                    origin = true;
                    digitalWrite(homing.index, HIGH);
                    delay(500);
                    digitalWrite(homing.index, LOW);
                    stop();
        }
    forward();
    delay(10);
    stop();
    delay(100);
    }

    return origin;
}