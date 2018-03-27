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
        ~Encoder();
        
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
};
