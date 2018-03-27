#include "Motor.h"

Motor x;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  x.setOe(4);
  x.setD(5);
  x.setDa0(22);
  x.setDa1(23);
  x.setDa2(24);
  x.setDa3(25);
  x.setDa4(26);
  x.setDa5(27);
  x.setDa6(28);
  x.setDa7(29);
  x.setSlots(100);
  x.setGearRatio(1);

}

void loop() {
  // put your main code here, to run repeatedly:
  x.readPos();
  float angle = x.getAngle();
  Serial.println(angle);
}
