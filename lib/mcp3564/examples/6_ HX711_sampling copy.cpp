/**
 * @file HX711_Readings_With_Timing.ino
 * 
 * @brief Interrupt-Driven Load Cell Reading with Time Measurement.
 * 
 * This Arduino code provides an interface for the HX711 analog-to-digital 
 * converter, typically used with weight scales. It leverages interrupts to 
 * detect when the ADC has new data available, ensuring efficient data acquisition 
 * without the need for continuous polling. Once data is ready, the elapsed time 
 * since the previous reading is computed and both the reading and time are printed 
 * to the serial monitor.
 * 
 * @author [Your Name]
 * @date [Date of creation or last modification]
 * 
 */


#include "HX711.h"

const int LOADCELL_DOUT_PIN = 23;
const int LOADCELL_SCK_PIN = 22;

HX711 scale;

volatile boolean newDataReady = false; 
unsigned long previousReadingTime = 0;  // Store the time of the previous reading

// Interrupt routine
void dataReadyISR() {
  if (scale.is_ready()) {
    newDataReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Attach the interrupt
  attachInterrupt(digitalPinToInterrupt(LOADCELL_DOUT_PIN), dataReadyISR, FALLING);
}

void loop() {
  if (newDataReady) {
      unsigned long currentReadingTime = micros();  // Capture the current time
      unsigned long elapsedTime = currentReadingTime - previousReadingTime;  // Calculate elapsed time
      
      long reading = scale.read();
      
      // Print reading and elapsed time
      Serial.print("HX711 reading: ");
      Serial.print(reading);
      Serial.print(", Time since last reading (microseconds): ");
      Serial.println(elapsedTime);
      
      previousReadingTime = currentReadingTime;  // Update the previous reading time
      newDataReady = false;
  }
}
