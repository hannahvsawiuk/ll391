int d = 1;
int navg = 2;
int nan = 6;
int nrpt = 500;
int ndisp = 20000;

void setup() {
  for (int i = 0; i < nan; i++)
    pinMode(A0 + i, INPUT);
  Serial.begin(57600);
  analogReference(INTERNAL); // 2.56 volts on the YUN, 1.1 on UNO
  // analogReference(EXTERNAL);  // based on the input to AREF
  // Serial.print("\n\nAvgDAQ\n");
}

void loop() {
  static int linesShown = 0;
  unsigned long sums[10]; // doesn't want to work with NAN as a dimension
  getAvgDAQ(sums);
  if (linesShown < ndisp) {
    showSums(sums);
    linesShown++;
  }
  while (millis() % nrpt);
}

void getAvgDAQ(unsigned long *sums) {
  sums[0] = (unsigned long)millis();
  for (int i = 0; i < nan; i++)
    sums[i] = 0;
  for (int i = 0; i < navg; i++)
    for (int j = 0; j < nan; j++) {
      sums[j] += analogRead(A0 + j);
      delayMicroseconds(d);
    }
  for (int i = 0; i < nan; i++) {
    // sums[i] *= 1404;  //convert to mv for 1.404 V AREF
    sums[i] *= 2560;  //convert to mv for 2.56 V internal ref
    // sums[i] *= 1100; // convert to mv for 1.10 V internal ref
    sums[i] /= 1024;
    sums[i] /= navg; // average over NAVG samples
  }
}

void showSums(unsigned long *sums) {
  // char scratch[80];

  // desired, actual

  Serial.print(sums[0]);
  Serial.print(",");
  Serial.print(sums[1]);
  Serial.print(",");
  Serial.print(sums[2]);
  Serial.print("\n");
  // scratch[0] = 0; // when in doubt code dangerously, writing a string over itself ;-)
  // for (int i = 0; i < nan; i++)
  //   sprintf(scratch, "%s, %5d", scratch, (int)sums[i]);
  // sprintf(scratch, "%s,   *", scratch);
  // Serial.print(scratch);
  // for (int i = 0; i < 80; i++)
  //   scratch[i] = '.';
  // scratch[79] = 0;
  // for (int i = 0; i < nan; i++) {
  //   long j = sums[i] / 20;
  //   if (j > 78)
  //     j = 78;
  //   scratch[j] = '0' + i;
  // }
  // Serial.print(scratch);
}
