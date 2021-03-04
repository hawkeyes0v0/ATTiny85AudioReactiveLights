/**
 * Copyright (c) 2019, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 * ATtiny13/026
 * Disco lights using FFT (Fast Fourier Transformation)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <FastLED.h>

#define N                    (16) // N-points (FFT)
#define B                    (3 * N / 4) // b-value (FFT)

#define DATA_PIN 0 // Pin 0 for ATTiny
#define NUM_LEDS 32

CRGB strip[NUM_LEDS];
int ledGrouping = (NUM_LEDS / (N>>1)); // number of LEDs per frequency range
int brightness[N>>1]; // store brightness values
unsigned int valueMax[N>>1]; // current max power values
int brightnessMax = 255; // max LED brighness
int fadeSpeed = 16; // speed of fading LEDs between pulses
int recoveryChecks = 100; // number of checks before raising sensitivity
int recoveryCounter[N>>1]; // sensitivity recovery counters per frequency range

const int W[N] = {16,15,13,9,1,-6,-10,-12,-13,-12,-10,-6,1,9,13,15}; // twiddle factors (FFT)
int samples[N]; // raw samples (ADC)
unsigned int power[N>>1]; // power spectrum (FFT)
volatile int counter = 0;

static void fft(void);

void setup(){
  /* setup */
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(strip, NUM_LEDS); // setup the strip
  for(int j = 0; j < (N>>1); j++){
    recoveryCounter[j] = 0; // set up sensitivity recover counters
  }
  delay(1000);
  
  DDRB |= _BV(DATA_PIN); // set LED pins as OUTPUT
  ADCSRA |= _BV(ADPS2)|_BV(ADPS0); // set ADC division factor to 256
  ADCSRA |= _BV(ADEN)|_BV(ADIE); // enable ADC interrupt
  ADMUX = _BV(MUX1); // set ADC2 (PB4) as audio input
  ADMUX |= _BV(ADLAR); // left adjust of ADC result
  sei(); // enable global interrupts
  ADCSRA |= _BV(ADSC); // start first signal acquisition
}
  /* loop */
void loop() {
  
  if (counter == N) {
    fft(); // do some DSP
    
    int i = 0; // declare 'i' now to allow rainbow effect across entire strip in chunks
    for(int j = 0; j < (N>>1); j++){
      if (power[j] > valueMax[j]){ // set current max power
        valueMax[j] = power[j];
      }
      int tempBrightness = map(power[j], 0, valueMax[j], 0, brightnessMax); //set up brightness of current power level
      if(power[j] < (valueMax[j] / 2)){ //recovery of sensitivity per frequency
        if(power[j] > 8){
          recoveryCounter[j]++;
          if (recoveryCounter[j] >= recoveryChecks){
            recoveryCounter[j] = 0;
            valueMax[j] -= (valueMax[j] / 8) - 1; //raise brightness by lowering max recorded value
          }
        }
      }else{
        recoveryCounter[j] = 0;
      }
      if (tempBrightness > brightnessMax){
        brightness[j] = brightnessMax; // makes sure mapped value does not exceed max brightness
      }else if (tempBrightness < 0){
        brightness[j] = 0;  // makes sure no negative values
      }else if(brightness[j] < tempBrightness){
        brightness[j] = tempBrightness;  // max jump down in brightness is half previous brightness
      }
      for (i; i < ((ledGrouping * j) + ledGrouping); i++) { //set current grouping of LEDs
        strip[i].setHSV((i * 255 / NUM_LEDS), 255, brightness[j]);
      }
      if (brightness[j] >= fadeSpeed){ // failsafe to fade LEDs even if brightness not changed
        brightness[j] -= fadeSpeed;
      }else{
        brightness[j] = 0; // sets extreme low values to 0 (not really necessary)
      }
    }
    FastLED.show();
    counter = 0; // reset samples counter
    ADCSRA |= _BV(ADSC); // trigger next signal acqusistion
  }
}
ISR(ADC_vect)
{
  if (counter < N) {
    samples[counter++] = ADCH - 128; // read raw sample <-128 to 127>
    ADCSRA |= _BV(ADSC); // trigger next signal acquisition
  }
}

/**
 * Twiddle-factor-based FFT algorithm with reduced memory access.
 */
void fft(void)
{
  int a, b, i, j;
  unsigned int re[N];
  unsigned int im[N];

  for (i = 0; i < N; ++i) {
    re[i] = 0;
    im[i] = 0;
  }
  for (i = 0; i < (N>>1); ++i) {
    a = 0;
    b = B;
    int leveler = (int)(((((N>>1) - i) * ((N>>1) - i)) / 3) + 3); //dynamic leveling of power value
    for (j = 0; j < N; ++j) {
      re[i] += (W[a%N] * samples[j]) / leveler;
      im[i] -= (W[b%N] * samples[j]) / leveler;
      a += i;
      b += i;
    }
    power[i] = (re[i] * re[i] + im[i] * im[i]) >> 4;
  }
}
