/**
 * @file mcp356x_filtered_force_measurement.cpp
 * @brief Load cell measurement with Butterworth filtering using MCP356x ADC.
 *
 * This program reads data from a load cell connected to an MCP356x ADC,
 * applies Butterworth filtering to reduce noise, and outputs the raw and filtered
 * values to the serial port every 500ms.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - CS_PIN (7): Connect to MCP356x CS (Chip Select)
 * - IRQ_PIN (6): Connect to MCP356x IRQ (Interrupt Request)
 */

#include "MCP356xScale.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

// Pin Definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN 7
#define IRQ_PIN 6

// Filter Configuration for MCP356x
constexpr double MCP_SAMPLING_FREQUENCY = 11111;
constexpr double MCP_CUT_OFF_FREQUENCY = 11;
constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;

// Global Variables
MCP356xScale* mcpScale = nullptr;
auto butterworthFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
float mcpRawReading = 0;

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, the MCP356xScale instance,
 * and configures the calibration parameters.
 */
void setup() {
    // Initialize Serial Communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize the MCP356xScale with 1 ADC and 1 scale
    mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);

    // MCP Calibration Configuration
    mcpScale->setScaleFactor(0, 0.0006016569289837716f);
    // Alternative calibration methods (commented out)
    // mcpScale->setLinearCalibration(0, 0.0006027751375968074f, -538.5241383004725f);
    // mcpScale->setPolynomialCalibration(0, 8.095189745489105e-14f, 0.0006025713062654753f, -538.5539280900973f);
    
    // Tare the load cell
    mcpScale->tare(0);
}

/**
 * @brief Main program loop
 * 
 * Continuously reads from the ADC, applies filtering,
 * and outputs the raw and filtered values at regular intervals.
 */
void loop() {
    static uint32_t lastPrintTime = 0;

    // Update ADC readings
    mcpScale->updatedAdcReadings();
    mcpRawReading = mcpScale->getReading(0);
    float filteredReading = butterworthFilter(mcpRawReading);

    // Get the current time
    uint32_t currentTime = millis();

    // Print the readings every 500ms
    if (currentTime - lastPrintTime >= 500) {
        lastPrintTime = currentTime;

        // Prepare the output string
        StringBuilder output;
        output.concatf("Raw = %.2f, Filtered = %.2f\n", mcpRawReading, filteredReading);

        // Output the readings
        Serial.print((char*)output.string());
    }
}