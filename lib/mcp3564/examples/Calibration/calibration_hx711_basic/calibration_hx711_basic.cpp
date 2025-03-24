/**
 * @file calibration_hx711_basic.cpp
 * @brief Captures raw readings from HX711 ADC for load cell calibration
 *
 * This program captures raw data from the HX711 ADC as weights are applied to a load cell.
 * The captured data is sent to the serial port for further calibration processing.
 * It uses an interrupt mechanism for efficient data acquisition, triggering readings
 * as soon as the HX711 has new data available.
 *
 * Hardware Connections:
 * - HX711_DOUT_PIN: Connect to HX711 DOUT pin
 * - HX711_SCK_PIN: Connect to HX711 SCK pin
 */

#include "HX711.h"

// HX711 Pin Configuration
const int HX711_DOUT_PIN = 23;
const int HX711_SCK_PIN = 22;

HX711 hx711Scale;
volatile boolean hx711DataReady = false;

/**
 * @brief Interrupt service routine for HX711 data ready signal
 * 
 * This function is called when the HX711 signals that data is ready
 * via the DOUT pin going low
 */
void hx711DataReadyISR() {
  if (hx711Scale.is_ready()) {
    hx711DataReady = true;
  }
}

void setup() {
  Serial.begin(2000000);
  
  // Initialize the HX711 ADC
  hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  
  // Attach interrupt to detect when new data is available from HX711
  attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);
}

void loop() {
    static long hx711Reading = 0;

    // Check if new data is available from the HX711
    if (hx711DataReady) {
        // Take multiple readings and average them for better stability
        hx711Reading = hx711Scale.read_average(20);
        hx711DataReady = false;
    }

    // Print the HX711 reading to the serial port
    Serial.println(hx711Reading);
    delay(1);
}
