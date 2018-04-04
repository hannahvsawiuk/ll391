#include "MegaEncoderCounter_v5.h"

void setup()
{
    Serial.begin(115200);
    PitchReset();

}

void loop()
{
    int pos = PitchGetCount();
    Serial.println(pos);
}