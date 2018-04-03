int nrpt = 100;
int ndisp = 20000;
#define streams 2

void setup() {
  for (int i = 0; i < streams; i++)
    pinMode(A0 + i, INPUT);
  Serial.begin(57600);
  analogReference(INTERNAL); // 2.56 volts on the YUN, 1.1 on UNO
}

void loop() {
  static int linesShown = 0;
  unsigned long sums[2]; // doesn't want to work with NAN as a dimension
  getAvgDAQ(sums);
  if (linesShown < ndisp) {
    showSums(sums);
    linesShown++;
  }
  while (millis() % nrpt);
}

void getAvgDAQ(unsigned long *sums) {
  sums[0] = (unsigned long)millis();
  for (int i = 0; i < streams; i++) // initializes the sums array
    sums[i] = 0;
  for (int i = 0; i < streams; i++) {
    sums[i] = random(analogRead(A0));
    delayMicroseconds(1);
  }
  for (int i = 0; i < streams; i++) {
    // sums[i] *= 1404;  //convert to mv for 1.404 V AREF
    sums[i] *= 2560;  //convert to mv for 2.56 V internal ref
    // sums[i] *= 1100; // convert to mv for 1.10 V internal ref
    sums[i] /= 1024; // converting from analog
    sums[i] /= streams; // average over NAVG samples
  }
}

void showSums(unsigned long *sums) {
  Serial.print(sums[0]);
  Serial.print(",");
  Serial.print(sums[1]);
  Serial.print(",");
  Serial.print(millis());
  Serial.print("\n");
  
}
