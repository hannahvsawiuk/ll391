#include <Arduino.h>
using namespace std;

class Encoder
{
    private:
        int oe;   //active low
        int d; 
        int sel1;
        int sel2;
        int da0;
        int da1;
        int da2;
        int da3;
        int da4;
        int da5;
        int da6;
        int da7;
        int index;
    
    public:
        Encoder(){};
        ~Encoder(){};
        
        void setOe(int pin)    { oe = pin;    pinMode(pin, OUTPUT);  };
        void setD(int pin)     { d = pin;     pinMode(pin, INPUT);   };
        void setSel1(int pin)  { sel1 = pin;  pinMode(pin, OUTPUT);  };
        void setSel2(int pin)  { sel2 = pin;  pinMode(pin, OUTPUT);  };
        void setDa0(int pin)   { da0 = pin;   pinMode(pin, INPUT);   };
        void setDa1(int pin)   { da1 = pin;   pinMode(pin, INPUT);   };
        void setDa2(int pin)   { da2 = pin;   pinMode(pin, INPUT);   };
        void setDa3(int pin)   { da3 = pin;   pinMode(pin, INPUT);   };
        void setDa4(int pin)   { da4 = pin;   pinMode(pin, INPUT);   };
        void setDa5(int pin)   { da5 = pin;   pinMode(pin, INPUT);   };
        void setDa6(int pin)   { da6 = pin;   pinMode(pin, INPUT);   };
        void setDa7(int pin)   { da7 = pin;   pinMode(pin, INPUT);   };
        void setIndex(int pin) { index = pin; pinMode(pin, INPUT);   };

        void getOe()    { return oe;    };
        void getD()     { return d;     };
        void getSel1()  { return sel1;  };
        void getSel2()  { return sel2;  };
        void getDa0()   { return da0;   };
        void getDa1()   { return da1;   };
        void getDa2()   { return da2;   };
        void getDa3()   { return da3;   };
        void getDa4()   { return da4;   };
        void getDa5()   { return da5;   };
        void getDa6()   { return da6;   };
        void getDa7()   { return da7;   };
        void getIndex() { return index; };

        int getPos();
};

int Encoder::getPos()
{
    int encoderPos = 0;
    digitalWrite(sel1, HIGH);
    digitalWrite(sel2, LOW); //reads lowest byte

    int pos0 = digitalRead(da0);
    int pos1 = digitalRead(da1);
    int pos2 = digitalRead(da2); 
    int pos3 = digitalRead(da3);
    int pos4 = digitalRead(da4);
    int pos5 = digitalRead(da5);
    int pos6 = digitalRead(da6);
    int pos7 = digitalRead(da7);

    digitalWrite(sel1, LOW);
    digitalWrite(sel2, LOW); //reads second lowest byte 

    int pos8 = digitalRead(da0);
    int pos9 = digitalRead(da1);
    int pos10 = digitalRead(da2);
    int pos11 = digitalRead(da3);
    int pos12 = digitalRead(da4);
    int pos13 = digitalRead(da5);
    int pos14 = digitalRead(da6);
    int pos15 = digitalRead(da7);

    bitWrite(encoderPos, 0, pos0);
    bitWrite(encoderPos, 1, pos1);
    bitWrite(encoderPos, 2, pos2);
    bitWrite(encoderPos, 3, pos3);
    bitWrite(encoderPos, 4, pos4);
    bitWrite(encoderPos, 5, pos5);
    bitWrite(encoderPos, 6, pos6);
    bitWrite(encoderPos, 7, pos7);
    bitWrite(encoderPos, 8, pos8);
    bitWrite(encoderPos, 9, pos9);
    bitWrite(encoderPos, 10, pos10);
    bitWrite(encoderPos, 11, pos11);
    bitWrite(encoderPos, 12, pos12);
    bitWrite(encoderPos, 13, pos13);
    bitWrite(encoderPos, 14, pos14);
    bitWrite(encoderPos, 15, pos15);

    return encoderPos;
}