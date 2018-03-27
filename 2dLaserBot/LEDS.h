using namespace std;
class Leds
{
    private:
        int largeUndershoot;
        int smallUndershoot ;
        int smallError;
        int smallOvershoot;
        int largeOvershoot;
    
    public:
        Leds(){};
        ~Leds(){};

        void setLargeUndershoot(int pin) { largeUndershoot = pin; pinMode(pin, OUTPUT);  };
        void setSmallUndershoot(int pin) { smallUndershoot = pin; pinMode(pin, OUTPUT);  };
        void setSmallError(int pin)      { smallError = pin;      pinMode(pin, OUTPUT);  };
        void smallOvershoot(int pin)     { smallOvershoot  = pin; pinMode(pin, OUTPUT);  };
        void largeOvershoot(int pin)     { largeOvershoot = pin;  pinMode(pin, OUTPUT);  };

        void largeUndershootLight();
        void smallUndershootLight();
        void smallErrorLight();
        void smallOvershootLight();
        void largeOvershootLight();
};

void Leds::largeUndershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, HIGH);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, LOW);
}

void Leds::smallOvershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, HIGH);
    digitalWrite(largeOvershoot, LOW);
}

void Leds::smallErrorLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, HIGH);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, LOW);
}

void Leds::smallOvershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, HIGH);
    digitalWrite(largeOvershoot, LOW);
}

void Leds::largeOvershootLight()
{
    digitalWrite(reachPoint, LOW);
    digitalWrite(smallError, LOW);
    digitalWrite(smallUndershoot, LOW);
    digitalWrite(largeUndershoot, LOW);
    digitalWrite(smallOvershoot, LOW);
    digitalWrite(largeOvershoot, HIGH);
}