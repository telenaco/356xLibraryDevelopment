/**
 * @file HX711_Calibration_Readings.ino
 * @brief Capture Raw Readings from HX711 for Calibration
 * 
 * This script captures raw data from the HX711 ADC as you apply weight to the load cell.
 * The captured data is then sent to the serial port. An interrupt mechanism is used 
 * to efficiently capture data from the HX711 as soon as it's ready.
 * 
 * @author [Your Name]
 * @date [Date of Creation]
 */

#include "HX711.h"

// HX711 Constants and Variables
const int HX711_DOUT_PIN = 23;
const int HX711_SCK_PIN = 22;

HX711 hx711Scale;
volatile boolean hx711DataReady = false;

void hx711DataReadyISR() {
  if (hx711Scale.is_ready()) {
    hx711DataReady = true;
  }
}

void setup() {
  Serial.begin(2000000);

  hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);
}

void loop() {
    static long hx711Reading = 0;

    if (hx711DataReady) {
        hx711Reading = hx711Scale.read_average(20);
        hx711DataReady = false;
    }

    // Print the HX711 reading to the serial port
    Serial.println(hx711Reading);
    delay(1);

}
