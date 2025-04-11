/**
 * @file mcp356x_basic_scale_reading.cpp
 * @brief Demonstrates basic reading from a load cell using the MCP356xScale class.
 *
 * This example shows how to initialize and use the MCP356xScale class to read
 * values from a load cell connected to an MCP356x ADC. It demonstrates basic
 * setup, reading, and output of load cell measurements.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - IRQ_PIN (3): Connect to MCP356x IRQ (Interrupt Request)
 * - CS_PIN (2): Connect to MCP356x CS (Chip Select)
 */

#include "Arduino.h"
#include "MCP356xScale.h"

// Pin definitions for SPI communication
#define SCK_PIN 13
#define SDO_PIN 12
#define SDI_PIN 11
#define IRQ_PIN 3
#define CS_PIN 2

// Total scales represents the number of scales (or channels) being used
const int TOTAL_SCALES = 1;

// Global variables
MCP356xScale *scale = nullptr;

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication and the MCP356xScale instance.
 */
void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }

    // Initialize the MCP356xScale with one scale
    scale = new MCP356xScale(TOTAL_SCALES, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);

    // Optionally, here you can set up channel mappings, perform calibration, and set scale factors
    // Example: scale->setScaleFactor(0, 0.0005f); // Set calibration factor for first scale

    Serial.println("MCP356x scale initialized and ready.");
}

/**
 * @brief Main program loop
 *
 * Continuously updates ADC readings and outputs the force reading from the scale.
 */
void loop()
{
    // Update readings from all ADCs
    scale->updatedAdcReadings();

    // For demonstration, assuming we are working with the first scale (index 0)
    int scaleIndex = 0;

    // Get the force reading from the specified scale index
    float force = scale->getForce(scaleIndex);

    // Print the force reading
    Serial.print("Force: ");
    Serial.print(force);
    Serial.println(" N");

    delay(1000); // Delay to make the output readable
}