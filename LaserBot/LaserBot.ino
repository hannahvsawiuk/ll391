#include "Motor.h"

Motor x;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  x.encoder.setOe(4);
  x.encoder.setD(5);
  x.encoder.setDa0(22);
  x.encoder.setDa1(23);
  x.encoder.setDa2(24);
  x.encoder.setDa3(25);
  x.encoder.setDa4(26);
  x.encoder.setDa5(27);
  x.encoder.setDa6(28);
  x.encoder.setDa7(29);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  x.updateAngle();
  float angle = x.getPos();
  Serial.println(angle);
}
