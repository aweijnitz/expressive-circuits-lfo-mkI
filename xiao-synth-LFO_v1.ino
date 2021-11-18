/**
   Basic LFO

   _Principle of operation_

   The LFO generates a wave on the DAC (pin A0) using a timer interrupt to calculate the output in the background.
   The main loop reads control input and sets the corresponding values (LFO freq).

   The wave forms are generated from pre-calculated lookup tables ("wave tables") for increased performance.

   PWM is generated on pin LFO_LED to fade an LED in time with the LFO output.


   Code written for Seeduino XIAO (https://wiki.seeedstudio.com/Seeeduino-XIAO).

   PINS USED
   See defines below to change values.

   - A0 DAC (output)
   - A2 LFO LED (PWM output)
   - A4 FREQ_PIN (input), 12bit ADC expects a potentiometer to set LFO frequency. See waveTables.h for upper and lower limits.
   - A6 EXT CLK TRIGGER (input). Triggers LFO sync.
   - A7 FREQ CV (input)
   - A9 WAVE SELECT KNOB (input) 2bit ADC expects a potentiometer and maps resistance to one of four wave modes.


   DEPENDENCIES
   - TimerTC3.h (Find via Arduino Libraries Manager, but you need to point to the XIAO boards URL first. All part of setting up for XIAO development.)

   Author: Anders Weijnitz



   LICENSE: ISC

   Copyright 2021 Anders Weijnitz

   Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

#include <math.h>
#include <TimerTC3.h> // Doc: https://www.pjrc.com/teensy/td_libs_TimerOne.html and https://github.com/PaulStoffregen/TimerThree

#define DAC_PIN A0
#define LFO_LED A2
#define FREQ_PIN A4
#define EXT_CLK_TRIG A6
#define FREQ_CV A7
#define WAVE_SELECT A9
#define TO_MICROS(x) ((unsigned long)(x*1000000.0)) // Convert seconds to micro seconds
#define HZ2PERIOD(hz) ((float)1/hz) // Calculate period of wave in seconds, given a frequency in Hz
#define UPDATE_FREQ 96000.0 // Interrupt trigger frequency. Think of this as "sample rate in Hz". 'tuningFactor' is impacted by this value.
#define PI2 6.283185
#define CLK_TIMEOUT TO_MICROS(5)

#include "waveTables.h"
#include "utils.h"

// LFO State
float lfoFrequency = 0.75;
float period = (float)1 / lfoFrequency;
int currentWaveForm = LFO_SIN; // Default wave. Index into the waves array.
int *curWaveTable = waves[currentWaveForm];


// Variables used for calculating the DAC output
const float updateInterval = TO_MICROS(HZ2PERIOD(UPDATE_FREQ)); // The timer interval used to drive the "sound engine".
volatile float tuningFactor = 0.20715; // Magic number to compensate for inaccuracy in interrupt timing in combination with ISR time
volatile float incrementUpdateFactor = tuningFactor / updateInterval; // Precalculated factor to save time in the interrupt execution
volatile float stepIncrement = lfoFrequency * incrementUpdateFactor;
volatile float tableIndex = 0;

// Ext clock state
volatile boolean useExternalClock = false;
volatile long timeLastTriggerMicros = micros();


/**
   This function is invoked by a timer interrupt every 'updateInterval' micro seconds.
*/
void timerIsr() {
  soundOut();
}

void clkTrigIsr() {
  onExternalTrigger();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Used for debugging
  pinMode(LFO_LED, OUTPUT);
  pinMode(FREQ_PIN, INPUT);
  pinMode(EXT_CLK_TRIG, INPUT_PULLUP);
  pinMode(FREQ_CV, INPUT_PULLDOWN);
  pinMode(WAVE_SELECT, INPUT);
  attachInterrupt(digitalPinToInterrupt(EXT_CLK_TRIG), clkTrigIsr, RISING);

  analogWriteResolution(10); // Set analog out resolution to max, 10-bits
  analogReadResolution(12); // Set analog input resolution to max, 12-bits

  generateSinWaveTable();
  generateSawWaveTable();
  generateTriangleWaveTable();

  TimerTc3.initialize(TO_MICROS(HZ2PERIOD(UPDATE_FREQ)));
  TimerTc3.attachInterrupt(timerIsr);
  TimerTc3.start();
  //SerialUSB.begin(115200);
}


/**
   While the timer interrup and trigger interrup drives the LFO generation and hard sync in the background,
   the loop() function continously polls control knobs and CV and updates the parameters.
*/
void loop() {
  blinkLED();

  // READ FREQ KNOB
  float frqKnobRaw = analogRead(FREQ_PIN);
  float frq = knobToFreqSetting(frqKnobRaw);
  if (fabs(frq - lfoFrequency) > lfoFrequency * 0.03) { // Allow for some fluctuation of values
    lfoFrequency = frq;
    onLFOFreqChange();
  }

  // READ FREQ CV INPUT
  float frqCVRaw = (float) analogRead(FREQ_CV);
  frqCVRaw = (frqCVRaw - 2048) / 2; // "Zero center" CV reading to allow for negative values. Divide by two to scale how much CV influences knob setting.
  float cvModifiedFrequency = lfoFrequency + knobToFreqSetting(min(max(0, frqKnobRaw + frqCVRaw), 1023));
  if (fabs(cvModifiedFrequency - lfoFrequency) > lfoFrequency * 0.03) { // Allow for some fluctuation of values (analogRead always retuns *something*)
    lfoFrequency = cvModifiedFrequency;
    onLFOFreqChange();
  }

  // EXT CLOCK TRIGGER
  // (state is changed via interrup handler)
  // Note: Currently external trigger does a hard sync on the wave to the beginning of the wave table. No tempo sync.
  if (useExternalClock) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (micros() - timeLastTriggerMicros >= CLK_TIMEOUT) { // Timeout of 5s for using external clock trigger
      useExternalClock = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // READ WAVE_SELECT
  static byte damper = 0;
  if (damper++%10 == 0) { // Sensor readings are very jittery. Only check mode knob ever so often. Reduce for quicker response.
    int raw = analogRead(WAVE_SELECT);
    int mode = knobToWaveMode(analogRead(WAVE_SELECT)); // Keeps a moving average and returns the mapping.
    currentWaveForm = mode;
    if (mode != LFO_STEP_RANDOM)
      curWaveTable = waves[currentWaveForm];
  }

}

/**
   Whenever the LFO frequency changes (ex. user turns knob, or CV change), then this function will be called.
*/
float onLFOFreqChange() {
  stepIncrement = lfoFrequency * incrementUpdateFactor;
}

/**
   Reset LFO to start of wave whenever there is an external trigger (ex. CV trigger pulse).
*/
float onExternalTrigger() {

  // Book keeping
  useExternalClock = true;
  timeLastTriggerMicros = micros();

  // Reset to start of wave
  tableIndex = 0;
}


/**
   For each timer interrupt, calculate next DAC value (stepping through the wave table).
*/
volatile int lastDacOut = 511.5; // 511.5 = 1023/2 => the middle of the possible range. 
void soundOut() {

  static int changeThreshold = 55;

  int dacVoltage = 511.5;
  if (currentWaveForm == LFO_STEP_RANDOM
      && ((int)tableIndex) == 0
      && random(100) > changeThreshold) { // Likeliness of change = 35% (100-65)
    int voltStep = random(50) - 25;
    dacVoltage = lastDacOut + voltStep;
    dacVoltage = min(1023, dacVoltage);
    dacVoltage = max(0, dacVoltage);
  }
  else if (currentWaveForm != LFO_STEP_RANDOM) {
    dacVoltage = curWaveTable[(int)tableIndex];
  } else
    dacVoltage = lastDacOut;

  tableIndex += stepIncrement;
  if (tableIndex >= WAVE_TABLE_LENGTH)
    tableIndex -= WAVE_TABLE_LENGTH;

  // Generate a voltage between 0 and 3.3V.
  // 0= 0V, 1023=3.3V, 512=1.65V, etc.
  lastDacOut = dacVoltage;
  analogWrite(DAC_PIN, dacVoltage);
}


/**
   Fade LED in time with LFO output.
*/
void blinkLED() {
  int fadeVal = 0;
  if (currentWaveForm == LFO_STEP_RANDOM)
    fadeVal = (int)(255 * lastDacOut / 1023); // 255 is max PWM value. 1023 is max DAC output.
  else
    fadeVal = (int)(255 * curWaveTable[(int)tableIndex] / 1023);
  analogWrite(LFO_LED, fadeVal);
}
