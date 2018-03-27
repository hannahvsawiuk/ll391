#include <Arduino.h>
#include "Memory.h"

using namespace std;

class PID
{
    private:
        Memory error;
        Memory prop;
        Memory integ;
        Memory deriv;
        float desired;
        float sampleTime;
        float Kp;
        float Ki;
        float Kd;
        float N;
    
    public:
        PID(){};
        PID(float P, float I, float D, float filter, float t)
        {
            Kp = P;
            Ki = I;
            Kd = D;
            N = filter;
            sampleTime = t;
        }; 
        ~PID(){};

        void setDesired(float desiredAngle) 
        { 
            desired = desiredAngle;
            prop.setPrev(0);
            integ.setPrev(0);
            deriv.setPrev(0);
        };

        float calculate(float currAngle);

};

float PID::calculate(float currAngle)
{
  float pidTerm;

  error.setCurr(desired - currAngle);
  
  integ.setCurr(integ.getPrev() + Ki*error.getCurr()*(sampleTime));
  prop.setCurr(Kp*error.getCurr());
  deriv.setCurr(1/(1 + N*sampleTime)*deriv.getPrev() + Kd*N/(1 + N*sampleTime)*(error.getCurr() - error.getPrev()));
  
  pidTerm = prop.getCurr() + integ.getCurr() + deriv.getCurr();
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm  = abs(pidTerm);//make sure it's a positive value

  integ.setPrev(integ.getCurr());
  deriv.setPrev(deriv.getCurr());
  error.setPrev(error.getCurr());

  return pidTerm;

}