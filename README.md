# Digital Voltage Controlled Low Frequency Oscillator (VCLFO)

This repository contains the code for a VCLFO, based on the Seeduino XIAO micro controller. 
The XIAO is a cheap, fast and small Arduino compatible with an onboard DAC. This makes it suitable for building basic digital modules for euro rack.

## Principle of operation

The LFO generates a wave on the DAC (pin A0) using a timer interrupt to calculate the output in the background.
The main loop reads control input and sets the corresponding values (LFO freq).
The wave forms are generated from pre-calculated lookup tables ("wave tables") for increased performance.
PWM is generated on pin LFO_LED to fade an LED in time with the LFO output.

Code written for Seeduino XIAO (https://wiki.seeedstudio.com/Seeeduino-XIAO).

### PINS USED

All pins are `#define`d in the source and can be changed if needed. The only exception is A0, which is tied to the DAC on the XIAO.

- A0 DAC (output)
- A2 LFO LED (PWM output)
- A4 FREQ_PIN (input), 12bit ADC expects a potentiometer to set LFO frequency. See waveTables.h for upper and lower limits.
- A6 EXT CLK TRIGGER (input). Triggers LFO sync.
- A7 FREQ CV (input)
- A9 WAVE SELECT KNOB (input) 12bit ADC expects a potentiometer and maps resistance to one of four wave modes.

### DEPENDENCIES

- `TimerTC3.h` (Find via Arduino Libraries Manager, but you need to point to the XIAO boards URL first. All part of setting up for XIAO development.)

### Hardware

See schema in [VCLFO_v1_schema.pdf](VCLFO_v1_schema.pdf)

### LICENSE: ISC

Copyright 2021 Anders Weijnitz

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.
THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

