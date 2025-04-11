/**
 * @file mcp356x_comparison_hx711_sampling_rate.cpp
 * @brief Interrupt-driven load cell reading with time measurement for HX711.
 *
 * This program interfaces with an HX711 analog-to-digital converter typically used 
 * with weight scales. It leverages interrupts to detect when the ADC has new data available, 
 * ensuring efficient data acquisition without continuous polling. Once data is ready, 
 * the elapsed time since the previous reading is computed and both the reading and time 
 * are printed to the serial monitor. This provides accurate measurement of the HX711's 
 * actual sampling performance.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - HX711_DOUT_PIN (23): Connect to HX711 DOUT (Data Output)
 * - HX711_SCK_PIN (22): Connect to HX711 SCK (Serial Clock Input)
 */

#include "HX711.h"

// Pin definitions for HX711 connection
#define HX711_DOUT_PIN 23
#define HX711_SCK_PIN 22

// HX711 scale object
HX711 scale;

// Data ready flag and timing variables
volatile boolean newDataReady = false; 
unsigned long previousReadingTime = 0;  // Store the time of the previous reading

/**
 * @brief Interrupt service routine for HX711 data ready detection
 * 
 * This function is called when the HX711 ADC has new data available,
 * which is indicated by the DOUT pin going low.
 */
void dataReadyISR() {
    if (scale.is_ready()) {
        newDataReady = true;
    }
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication and the HX711 ADC. Configures
 * the interrupt handler for detecting when new data is available.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
   
    // Initialize the HX711 scale
    scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);

    // Attach the interrupt to detect when new data is available
    attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), dataReadyISR, FALLING);
   
    Serial.println("HX711 sampling rate measurement initialized");
    Serial.println("Time measurements are in microseconds");
}

/**
 * @brief Main program loop
 * 
 * Continuously checks if new data is available from the HX711.
 * When new data arrives, it calculates the time elapsed since the
 * previous reading and outputs both the reading value and elapsed time.
 */
void loop() {
    // Check if new data is available from the HX711
    if (newDataReady) {
        // Capture the current time
        unsigned long currentReadingTime = micros();
       
        // Calculate elapsed time since the previous reading
        unsigned long elapsedTime = currentReadingTime - previousReadingTime;
       
        // Read the load cell value
        long reading = scale.read();
       
        // Print reading and elapsed time information
        StringBuilder output;
        output.concatf("HX711 reading: %ld, Time since last reading: %lu us, Rate: %.2f Hz", 
                      reading, elapsedTime, 1000000.0 / elapsedTime);
        Serial.println((char*)output.string());
       
        // Update the previous reading time for the next cycle
        previousReadingTime = currentReadingTime;
       
        // Reset the data ready flag
        newDataReady = false;
    }
}