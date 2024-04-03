/**
 * @file 10_mcpForce_basic.cpp
 * @brief Demonstrates the usage of the MCP356xScale library to read load cell data.
 *
 * This program initializes an instance of the MCP356xScale class, which represents a load cell connected to an MCP356x ADC.
 * It sets the calibration factor to 1 using the setScaleFactor() function and performs a tare operation.
 * In the main loop, it continuously updates the ADC readings, retrieves the tare-corrected reading and original ADC value,
 * and prints them along with the readings per second and loop counter every second.
 *
 * Hardware Connections:
 * - SDI_PIN : Connect to MCP356x SDI (Serial Data Input).
 * - SDO_PIN : Connect to MCP356x SDO (Serial Data Output).
 * - SCK_PIN : Connect to MCP356x SCK (Serial Clock Input).
 * - CS_PIN  : Connect to MCP356x CS (Chip Select).
 * - IRQ_PIN : Connect to MCP356x IRQ (Interrupt Request).
 */

#include "MCP356xScale.h"

// Pin Definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN  7
#define IRQ_PIN 6

MCP356xScale* mcpScale = nullptr;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
    delay(10);
    }

    // if you are using more than four channels you need to add the pins for the 
    // second adc, and if using more than 8 channels you need to include the pins 
    // for the 3rd adc. 
    mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);

    // // Linear calibration factor to 1
    // mcpScale->setLinearCalibration(0, 1.0f, 0.0f);

    // // Polynomial calibration factor to 1
    // mcpScale->setPolynomialCalibration(0, 0.0f, 1.0f, 0.0f);

    // Set the calibration factor to 1
    mcpScale->setScaleFactor(0, 1.0f);

    // Tare the load cell
    mcpScale->tare(0);

    Serial.println("Load cell initialized and tared.");
}

void loop() {
    static uint32_t lastPrintTime = 0;
    static uint32_t loopCounter   = 0;
    uint32_t currentTime          = millis();

    // Update ADC readings
    mcpScale->updatedAdcReadings();

    // Increment loop counter
    loopCounter++;

    // Print the tare-corrected reading, original ADC value, readings per second, and loop counter every second
    if (currentTime - lastPrintTime >= 2000) {
        lastPrintTime = currentTime;

        StringBuilder output;
        // Currently not calibrated will return a digital value ~close to 0
        float tareReading = mcpScale->getReading(0);  
        double adcValue = mcpScale->getRawValue(0);

        output.concatf("Tare-corrected reading: %.2f tare value, Original ADC value: %.0f\n", tareReading, adcValue);

        mcpScale->printChannelParameters();

        // Print the readings per second for initialized ADCs
        mcpScale->printReadsPerSecond();

        // Print the loop counter
        output.concatf("Loop iterations: %lu\n", loopCounter);

        if (output.length() > 0) {
            Serial.print((char*)output.string());
        }

        // Reset loop counter
        loopCounter = 0;
    }
}
