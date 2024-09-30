#define MICROPHONE_PIN 35
#define CS_PIN 5

// ----------------------- FFT configuration ----------------------------
#define FFT_N 256  // Must be a power of 2
#define SAMPLEFREQ 41000
// ----------------------------------------------------------------------

// ----------------------- LCD configuration ----------------------------
#define DISPLAY_NUMBER 4
#define MAX_X DISPLAY_NUMBER * 8
#define MAX_Y 8
// ----------------------------------------------------------------------

// ----------------------- Minimal amplitude to draw on LED matrix -------------
#define LOW_THRESHOLD 8
// ----------------------------------------------------------------------

#define ARRAYLENGTH(array) sizeof(array) / sizeof(int)

#include "ESP32_fft.h"
#include <GyverMAX7219.h>

MAX7219<DISPLAY_NUMBER, 1, CS_PIN> mtrx;

// manually choosed frq index (1 - 128)
constexpr byte spectrIndexes[32] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 24, 29, 34, 39, 44, 50, 60, 70, 80, 90, 100, 120 };

static int timer = millis();

float fft_input[FFT_N];
float fft_output[FFT_N];

ESP_fft FFT(FFT_N, SAMPLEFREQ, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

volatile bool adc_coversion_done = false;
adc_continuous_data_t* result = NULL;

constexpr uint8_t adcPins[] = { MICROPHONE_PIN };

void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

void setup() {
  Serial.begin(115200);

  mtrx.begin();       // запускаем
  mtrx.setBright(2);  // яркость 0..15
  mtrx.setFlip(false, true);

  analogContinuous(adcPins, 1, 1, 50 * 1000, &adcComplete);
  analogContinuousStart();
}

static int count = 0;

void loop() {
  if (adc_coversion_done == true) {
    adc_coversion_done = false;
    if (analogContinuousRead(&result, 0)) {
      fft_input[count] = (float)result[0].avg_read_mvolts;
      count++;
    }
  }

  if (count >= FFT_N) {
    count = 0;

    FFT.execute();
    FFT.complexToMagnitude();

    float peak = FFT.majorPeak() * 2 / FFT_N;
    int multiplier = 1;
    if (peak >= LOW_THRESHOLD) {
      multiplier = map(peak, 0, 300, 5, 1);
      multiplier = constrain(multiplier, 1, 5);
      for (int i = 1; i < FFT_N / 2; i++) { // value at 0 index is DC offset
        float amplitude = (fft_output[i] * 2 / FFT_N);
        fft_output[i] = amplitude * multiplier; // transform magnitude -> amplitude * multiplier (multiplier is optional)
      }
      matrixDrawPulse();
    }
    mtrx.clear();
  }
}

void matrixDrawPulse() {
  for (int column = 0; column < MAX_X; column++) {
    int amplitude = (int)fft_output[spectrIndexes[column]];
    int line = map(amplitude, LOW_THRESHOLD, 256, 0, MAX_Y - 1);
    line = constrain(line, 0, MAX_Y - 1);

    if (line > 0) mtrx.line(column, 0, column, line);
  }
  mtrx.update();
}