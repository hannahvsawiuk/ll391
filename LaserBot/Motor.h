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

class Motor
{
    private:
        int pos;
        float angle;
        float pidTerm;
        float desired;
        homing homing;
        int homingSpeed;
        bool dir1;
        PID pid;


    public:
        Motor(){};
        Motor(float P, float I, float D, float filter, float t)
        {
            PID pid(P, I, D, filter, t);
        };
        
        ~Motor(){};

        Encoder encoder;
        Leds led;
        Driver driver;

        void calculate();
        bool control();
        void updateAngle();
        bool home();

        void setIndex(int pin)          { homing.trig = pin; };
        void setHomingSpeed (int speed) { homingSpeed = speed; };

        float getAngle()   { return angle;   };
        int getPos()       { return pos;     };
        float getPidTerm() { return pidTerm; };

    
};

void Motor::updateAngle()
{
    pos = encoder.getPos();
    angle = pos * 0.015708;
}

void Motor::calculate()
{
    pidTerm = pid.calculate(angle);
}

bool Motor::control()
{
    float error = angle - desired;
    bool reached;

    if (abs(error) <= 0.0174533)
    {   //1 degrees

            led.smallErrorLight();
            driver.stop();
            reached = true;
    }

    else if (angle < desired) 
    {
        bool reached = false;
        driver.forward();

        if (error < 0.05) //2.864789 degrees
        {
            led.smallUndershootLight();
        }
        else 
        { //5.72958 degrees
            led.largeUndershootLight();
        } 
    }
    else {
        reached = false;
        driver.reverse();

        if (abs(error) < 0.05) //2.864789 degrees
        {

            led.smallOvershootLight();

        }
        else { // larger than 5.72958 degrees

            led.largeOvershootLight();

        }
    }
    driver.setPWM(pidTerm);
    return reached;
}

bool Motor::home()
{
    bool reached = false;
    driver.setPWM(homingSpeed);
    while (reached == false)
    {
        if (digitalRead(homing.trig) == 0)
        {
                    reached = true;
                    digitalWrite(homing.index, HIGH);
                    delay(500);
                    digitalWrite(homing.index, LOW);
                    driver.stop();
        }
    driver.forward();
    delay(10);
    driver.stop();
    delay(100);
    }

    return reached;
}