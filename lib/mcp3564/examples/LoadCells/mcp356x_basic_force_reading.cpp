/**
 * @file mcp356x_basic_force_reading.cpp
 * @brief Interface and control routines for MCP356x ADC with load cell data processing.
 *
 * This program interfaces with the MCP356x ADC, reads data from a load cell,
 * applies necessary calibrations, and filters the readings. The filtered and calibrated readings
 * are then outputted over the serial interface every 500ms.
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
MCP356xScale *mcpScale = nullptr;
auto butterworthFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
int32_t mcpRawReading = 0;
float mcpFilteredReading = 0;
uint32_t lastSuccessfulMCPReadTime = 0;
uint32_t elapsedMCPMicros = 0;

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, the MCP356xScale instance,
 * and configures the calibration parameters.
 */
void setup()
{
    // Initialize Serial Communication
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    // Initialize the MCP356xScale with 1 ADC and 1 scale
    mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);

    // MCP Calibration Configuration
    mcpScale->setScaleFactor(0, 0.0006017f);
    // Alternative calibration methods (commented out)
    // mcpScale->setLinearCalibration(0, 0.0006028f, -538.524f);
    // mcpScale->setPolynomialCalibration(0, 0.0f, 0.0006026f, -538.554f);

    // Tare the load cell
    mcpScale->tare(0);
}

/**
 * @brief Main program loop
 *
 * Continuously reads from the ADC, measures timing between readings,
 * applies filtering, and outputs the results at regular intervals.
 */
void loop()
{
    static uint32_t lastPrintTime = 0;

    // Update ADC readings
    if (mcpScale->updatedAdcReadings())
    {
        elapsedMCPMicros = micros() - lastSuccessfulMCPReadTime;
        mcpRawReading = mcpScale->getReading(0);
        mcpFilteredReading = butterworthFilter(mcpRawReading);
        lastSuccessfulMCPReadTime = micros();
    }

    // Get the current time
    uint32_t currentTime = millis();

    // Print the readings every 500ms
    if (currentTime - lastPrintTime >= 500)
    {
        lastPrintTime = currentTime;

        // Prepare the output string
        StringBuilder output;
        output.concatf("Raw Reading: %ld, Filtered Reading: %.2f grams, Time Elapsed: %lu microseconds",
                       mcpRawReading, mcpFilteredReading, elapsedMCPMicros);

        // Output the readings
        Serial.println((char *)output.string());
    }
}