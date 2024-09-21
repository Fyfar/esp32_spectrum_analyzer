#define MICROPHONE_PIN 35

// ----------------------- FFT configuration ----------------------------
#define FFT_N 256  // Must be a power of 2
#define SAMPLEFREQ 41000
// ----------------------------------------------------------------------

// ----------------------- LCD configuration ----------------------------
#define LCD_COLUMN_NUMBER 20
#define LCD_ROW_NUMBER 4
#define LCD_LINES_IN_COLUMN 8
// ----------------------------------------------------------------------

// ----------------------- Minimal amplitude to draw on LCD -------------
#define LOW_THRESHOLD 8
// ----------------------------------------------------------------------

#define ARRAYLENGTH(array) sizeof(array) / sizeof(int)

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "ESP32_fft.h"

hd44780_I2Cexp lcd;

constexpr byte rectangle = 0b11111111;
constexpr byte line1[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111 };
constexpr byte line2[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111 };
constexpr byte line3[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111 };
constexpr byte line4[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111 };
constexpr byte line5[8] = { 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };
constexpr byte line6[8] = { 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };
constexpr byte line7[8] = { 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };

// manually choosed frq index (1 - 128)
constexpr byte spectrIndexes[20] = { 2, 3, 4, 6, 8, 10, 12, 14, 16, 18, 20, 25, 30, 35, 50, 60, 70, 80, 100, 120 };

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

  lcd.begin(LCD_COLUMN_NUMBER, LCD_ROW_NUMBER);
  createCustomChars();

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
    lcd.clear();
    count = 0;

    FFT.execute();
    FFT.complexToMagnitude();

    float peak = FFT.majorPeak() * 2 / FFT_N;
    int multiplier = 1;
    if (peak >= LOW_THRESHOLD) {
      multiplier = map(peak, 0, 300, 5, 1);
      multiplier = constrain(multiplier, 1, 5);
      for (int i = 1; i < FFT_N / 2; i++) { // value at 0 index is DC offset
        fft_output[i] = (fft_output[i] * 2 / FFT_N) * multiplier; // transform magnitude -> amplitude * multiplier (multiplier is optional)
      }
      lcdDrawPulse();
    }
  }
}

void lcdDrawPulse() {
  for (int column = 0; column < LCD_COLUMN_NUMBER; column++) {
    int amplitude = (int)fft_output[spectrIndexes[column]];
    int lineIndex = map(amplitude, LOW_THRESHOLD, 256, 0, (LCD_LINES_IN_COLUMN * LCD_ROW_NUMBER) - 1);
    lineIndex = constrain(lineIndex, 0, (LCD_LINES_IN_COLUMN * LCD_ROW_NUMBER) - 1);

    for (int row = LCD_ROW_NUMBER - 1; row >= 0 && lineIndex >= 0; row--, lineIndex -= 6) {
      if (lineIndex > 6) {
        lcd.setCursor(column, row);
        lcd.write(rectangle);
      } else {
        lcd.setCursor(column, row);
        lcd.write(lineIndex);
      }
    }
  }
}

void createCustomChars() {
  lcd.createChar(0, line1);
  lcd.createChar(1, line2);
  lcd.createChar(2, line3);
  lcd.createChar(3, line4);
  lcd.createChar(4, line5);
  lcd.createChar(5, line6);
  lcd.createChar(6, line7);
}
