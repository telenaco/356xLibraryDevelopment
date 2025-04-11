/**
 * @file mcp356x_multiple_load_cell_manager.cpp
 * @brief Demonstrates the usage of the MCP356xScale library to manage multiple load cells.
 *
 * This program initializes an instance of the MCP356xScale class for managing up to twelve load cells 
 * connected to MCP356x ADCs. It sets the calibration factor to 1 for each load cell using the 
 * setScaleFactor() function and performs a tare operation. In the main loop, it continuously updates 
 * the ADC readings, retrieves the tare-corrected readings and original ADC values, and prints them 
 * along with the readings per second for each load cell every two seconds.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - CS_PIN0 (7): Connect to first MCP356x CS (Chip Select)
 * - IRQ_PIN0 (6): Connect to first MCP356x IRQ (Interrupt Request)
 * - CS_PIN1 (4): Connect to second MCP356x CS (Chip Select)
 * - IRQ_PIN1 (5): Connect to second MCP356x IRQ (Interrupt Request)
 * - CS_PIN2 (2): Connect to third MCP356x CS (Chip Select)
 * - IRQ_PIN2 (3): Connect to third MCP356x IRQ (Interrupt Request)
 */

#include "MCP356xScale.h"

// Pin Definitions for SPI
#define SDI_PIN  11
#define SDO_PIN  12
#define SCK_PIN  13

// Pin Definitions for ADCs
#define CS_PIN0  7
#define IRQ_PIN0 6
#define CS_PIN1  4
#define IRQ_PIN1 5
#define CS_PIN2  2
#define IRQ_PIN2 3

#define TOTAL_NUM_CELLS 12

MCP356xScale* mcpScale = nullptr;

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, the MCP356xScale instance,
 * and sets up calibration factors and tare operations for all load cells.
 */
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);

    // Tare operation for all load cells to set the current weight as the zero point
    for (int i = 0; i < TOTAL_NUM_CELLS; i++) {
        mcpScale->setScaleFactor(i, 1.0f); // Set the calibration factor to 1 for each cell
        mcpScale->tare(i); // Tare each cell
    }

    Serial.println("All load cells initialized and tared.");
}

/**
 * @brief Main program loop
 * 
 * Continuously updates ADC readings and outputs information about all 
 * load cells, including tare-corrected readings and original ADC values.
 */
void loop() {
    static uint32_t lastPrintTime = millis();

    // Update ADC readings for all load cells
    if (mcpScale->updatedAdcReadings()) {
        // Print readings every 2000 milliseconds (2 seconds)
        if (millis() - lastPrintTime >= 2000) {
            lastPrintTime = millis();

            StringBuilder output;

            for (int i = 0; i < TOTAL_NUM_CELLS; i++) {
                // If the ADC pins are floating, it will just show noise
                float tareReading = mcpScale->getReading(i); // Get tare-corrected reading
                double adcValue = mcpScale->getRawValue(i); // Get raw ADC value

                output.concatf("Load Cell %d: Tare-corrected reading = %.2f grams, Original ADC value = %.0f\n", i + 1, tareReading, adcValue);
            }

            // Output the readings from all load cells
            Serial.print((char*)output.string());

            // Print a separator for readability
            Serial.println("------------------------------------------------");
        }
    }
}