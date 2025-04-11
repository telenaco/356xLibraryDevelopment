/**
 * @file calibration_mcp356x_force_basic.cpp
 * @brief Basic example for calibrating a load cell using MCP356x ADC.
 *
 * This example demonstrates a simple load cell calibration setup using the MCP356xScale
 * class. It configures a single load cell, applies a basic scale factor calibration,
 * performs tare operation, and continuously outputs readings. This provides a foundation
 * for more advanced calibration techniques.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - CS_PIN (7): Connect to MCP356x CS (Chip Select)
 * - IRQ_PIN (6): Connect to MCP356x IRQ (Interrupt Request)
 */

 #include "MCP356xScale.h"

 // Pin definitions for SPI communication
 #define SDI_PIN 11
 #define SDO_PIN 12
 #define SCK_PIN 13
 #define CS_PIN  7
 #define IRQ_PIN 6
 
 // Pointer for MCP356xScale object
 MCP356xScale* mcpScale = nullptr;
 
 /**
  * @brief Setup function run once at startup
  * 
  * Initializes serial communication, the MCP356xScale instance,
  * applies calibration, and performs tare operation.
  */
 void setup() {
     // Initialize serial communication
     Serial.begin(115200);
     while (!Serial) {
         delay(10);
     }
 
     // Initialize the scale with 1 channel
     mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);
 
     // Set the calibration factor
     mcpScale->setScaleFactor(0, 1.0f);
 
     // Tare the load cell
     mcpScale->tare(0);
 
     Serial.println("Load cell initialized and tared.");
 }
 
 /**
  * @brief Main program loop
  * 
  * Continuously reads data from the load cell, calculates and displays
  * tare-corrected readings, original ADC values, and loop statistics.
  */
 void loop() {
     static uint32_t lastPrintTime = 0;
     static uint32_t loopCounter   = 0;
     uint32_t currentTime          = millis();
 
     // Update ADC readings
     mcpScale->updatedAdcReadings();
 
     // Increment loop counter
     loopCounter++;
 
     // Print readings every 2 seconds
     if (currentTime - lastPrintTime >= 2000) {
         lastPrintTime = currentTime;
 
         StringBuilder output;
         // Get tare-corrected reading and original ADC value
         float tareReading = mcpScale->getReading(0);  
         double adcValue = mcpScale->getRawValue(0);
 
         output.concatf("Tare-corrected reading: %.2f, Original ADC value: %.0f\n", tareReading, adcValue);
 
         // Print detailed calibration parameters
         mcpScale->printChannelParameters();
 
         // Print readings per second statistics
         mcpScale->printReadsPerSecond();
 
         // Print loop counter
         output.concatf("Loop iterations: %lu\n", loopCounter);
 
         if (output.length() > 0) {
             Serial.print((char*)output.string());
         }
 
         // Reset loop counter
         loopCounter = 0;
     }
 }