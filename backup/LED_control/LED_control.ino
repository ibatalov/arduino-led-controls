#define ARM_MATH_CM4

#include "WS2812B.h"
#include <math.h>
#include <arm_math.h>

#define anim_speed (1)

int SAMPLE_RATE_HZ = 9000;             // Sample rate of the audio in hertz.
int TIMER_DELAY = 1000000/SAMPLE_RATE_HZ;
float SPECTRUM_MIN_DB = 30.0;          // Audio intensity (in decibels) that maps to low LED brightness.
float SPECTRUM_MAX_DB = 60.0;          // Audio intensity (in decibels) that maps to high LED brightness.

const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256 
                                       // without running out of memory for buffers and other state.
const int AUDIO_INPUT_PIN = 0;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.

TimerObject samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;

WS2812B LEDS;

uint32_t phase = 0;
uint32_t max_phase = 50000000;
//uint8_t wavelength = 15;
uint32_t last_update = 0;
uint16_t NUM_RGB = 135;
const float pi = 3.141592;

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

void setup() {
  Serial.begin(9600);
  
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  
  // Begin sampling audio
  samplingTimer = new TimerObject(TIMER_DELAY);
  samplingTimer.setOnTimer(&samplingCallback);
  samplingTimer.setSingleShot(false);
  samplingTimer.Start();
  
  LEDS.init(NUM_RGB, NUM_RGB * 3, 8);
  randomSeed(analogRead(0));
}

//byte r;
//byte g;
//byte b;
float hue;
float sat = 1.f;
float val = 1.f;
//float wavephase, wavephase2;
uint32_t delta;
uint8_t loop_number;
const uint8_t max_loop_number = 12;

void loop() {
  samplingTimer.Update();
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);
  
    // Restart audio sampling.
    sampleCounter = 0;
    samplingTimer.Start();
  }
  
  /*
  phase += (micros() - last_update) * anim_speed;
  if (phase >= max_phase) {
    Serial.println(phase);
    phase %= max_phase;
    Serial.println(phase);
    loop_number++;
    loop_number %= max_loop_number;
    hue = (float)random(100) / 99 * 359;

  }
  */
  uint16_t i;
  if (delta++ % 500 == 0) {
    Serial.print("delay: ");
    Serial.println(micros() - last_update);
  }
  last_update = micros();

  
  //val = getRampBrightness((float)phase / max_phase);
  for (i = 0; i < NUM_RGB; i++) {
    //    wavephase = (float)i/wavelength*6*pi - (float)phase/max_phase*6*pi;
    //    wavephase2 = (float)i/wavelength*6*pi + (float)phase/max_phase*6*pi;
    //    r = (uint8_t)((sin(wavephase - pi/2) + 1)*50)*((wavephase < -4*pi) || (wavephase > 0)*(wavephase < 2*pi));
    //    g = (uint8_t)((sin(wavephase2 - pi/2) + 1)*50)*((wavephase2 > 4*pi)*(wavephase2 < 6*pi));
    //    b = 0;
    //    LEDS.setColorRGB(i, r, g, b);
    //hue = (float)i / LEDS.num_rgb * 359;
    //val = 0.1f;
    hue = (float)i/NUM_RGB*359;
    HSVtoRGB(hue, sat, val, LEDS.getPixelGRB(i));
  }
  LEDS.render();
}

float amplitude = 0.5f;
float getRampBrightness(float fract_phase) {
  if (fract_phase < 0.1f) {
    return amplitude * sin(fract_phase * pi * 5);
  } else if (fract_phase < 0.9f) {
    return amplitude;
  } else {
    return amplitude * sin((fract_phase - 0.8) * pi * 5);
  }
}
float r;
float g;
float b;

void HSVtoRGB(float h, float s, float v, uint8_t* grb) {
  h /= 60;
  h = fmod(h, 6);
  switch ((uint8_t)h) {
    case 0:
      r = v;
      g = r * (1 - abs(fmod(h, 2) - 1)) + v - v * s;
      b = v - v * s;
      break;
    case 1:
      g = v;
      r = g * (1 - abs(fmod(h, 2) - 1)) + v - v * s;
      b = v - v * s;
      break;
    case 2:
      r = v - v * s;
      g = v;
      b = g * (1 - abs(fmod(h, 2) - 1)) + v - v * s;
      break;
    case 3:
      r = v - v * s;
      b = v;
      g = b * (1 - abs(fmod(h, 2) - 1)) + v - v * s;
      break;
    case 4:
      g = v - v * s;
      b = v;
      r = b * (1 - abs(fmod(h, 2) - 1)) + v - v * s;
      break;
    case 5:
      r = v;
      g = v - v * s;
      b = r * (1 - abs(fmod(h, 2) - 1)) + v - v * s;
      break;
  }
  *grb++ = (uint8_t)(g * 255); //setting green LED
  *grb++ = (uint8_t)(r * 255); // setting red LED
  *grb = (uint8_t)(b * 255); // setting blue LED
}

////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < FFT_SIZE/2; ++i) {
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.stop();
  }
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}

