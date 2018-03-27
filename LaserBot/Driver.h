#include <Arduino.h>
using namespace std;
class Driver
{
    private:
        int dir1;
        int dir2;
        int pwm;
    
    public:
        Driver(){};
        ~Driver(){};

        void setDir1(int pin) { dir1 = pin; pinMode(pin, OUTPUT);  };
        void setDir2(int pin) { dir2 = pin; pinMode(pin, OUTPUT);  };
        void setPWM(int pin)  { pwm = pin;  pinMode(pin, OUTPUT);  };
        
        void forward() { digitalWrite(dir1, HIGH); digitalWrite(dir2, LOW); };
        void reverse() { digitalWrite(dir2, HIGH); digitalWrite(dir1, LOW); };
        void stop()    { digitalWrite(dir1, LOW);  digitalWrite(dir2, LOW); };
        
        void turnPWM(float pwmVal)  { analogWrite(pwm, pwmVal); };
        
};
