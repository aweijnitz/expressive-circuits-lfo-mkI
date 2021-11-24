

float floatMap(int x, int in_min, int in_max, float out_min, float out_max) {
  return (float)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

float knobToFreqSetting(int knobValue) {
  return floatMap(knobValue, 0, 4096, LFO_MIN_FREQ, LFO_MAX_FREQ);
}


// Knobs with smooting
const int numReadings = 2;

int knobToWaveMode(int knobValue) {
  static int readings[numReadings];      // the readings from the analog input
  static int readIndex = 0;              // the index of the current reading
  static int total = 0;                  // the running total
  static int average = 0;                // the average
  
  total = total - readings[readIndex];
  readings[readIndex] = knobValue;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings)
    readIndex = 0;
  average = total / numReadings;
  return map(average, 0, 4096, 0, 4); // SIN, SAW, TRI, RND
}
