#define WAVE_TABLE_LENGTH 2048
#define LFO_MIN_FREQ 0.01 // Arbitrary value
#define LFO_MAX_FREQ 22.5 // Arbitrary value
#define LFO_SIN 0
#define LFO_SAW 1
#define LFO_TRIANGLE 2
#define LFO_STEP_RANDOM 3

int sinTable[WAVE_TABLE_LENGTH]; // TODO: Move to progmem as fixed array.
int sawTable[WAVE_TABLE_LENGTH]; // TODO: Move to progmem as fixed array.
int triangleTable[WAVE_TABLE_LENGTH]; // TODO: Move to progmem as fixed array.
int *waves[4] = {sinTable, sawTable, triangleTable};


void generateSinWaveTable() {
  float dt = 1.0 / WAVE_TABLE_LENGTH;
  float t = 0.0;
  float freq = 1.0;
  for (int i = 0; i < WAVE_TABLE_LENGTH; i++) {
    sinTable[i] = (int)(511.5 + 511.5 * sin(t * PI2 * freq));
    t += dt;
  }
}


/**
   Generate a saw wave of 1Hz. One full period.
   This code was optimized for readability, not execution speed (only used from setup()).
   Math source: https://en.wikipedia.org/wiki/Triangle_wave
*/
void generateSawWaveTable() {
  float dt = 1.0 / WAVE_TABLE_LENGTH;
  float t = 0.0;
  float freq = 1.0;
  float amplitude = 511.5;
  float offset = amplitude;
  float period = 1.0;
  for (int i = 0; i < WAVE_TABLE_LENGTH; i++) {
    float val = (4 * amplitude / period * 0.5) * fabs((((t - period / 4)) + period) - period / 2) - amplitude;
    sawTable[i] = (int)(offset + val);
    t += dt;
  }
}

/**
   Generate a triangle wave of 1Hz. One full period.
   This code was optimized for readability, not execution speed (only used from setup()).
   Math source: https://en.wikipedia.org/wiki/Triangle_wave
*/
void generateTriangleWaveTable() {
  float dt = 1.0 / WAVE_TABLE_LENGTH;
  float t = 0.0;
  float freq = 1.0;
  float amplitude = 1023;
  float offset = amplitude;
  float period = 1.0;
  float P = period / 2.0;

  for (int i = 0; i < WAVE_TABLE_LENGTH; i++) {
    float val = (amplitude / P) * (P - fabs(fmod(t, period) - P));
    triangleTable[i] = (int)(val);
    t += dt;
  }
}
