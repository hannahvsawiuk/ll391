#include <Arduino.h>
using namespace std;

class Memory
{
    private:
        float curr;
        float prev;
    
    public:
         Memory(){};
         ~Memory(){};
         float getCurr() { return curr; };
         float getPrev() { return prev; };
         void setCurr(float currentTerm) { curr = currentTerm; };
         void setPrev(float prevTerm) { prev = prevTerm; };

}; 
