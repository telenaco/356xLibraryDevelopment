/**
 * @file mcp356x_6axis_telemetry.cpp
 * @brief Reads data from 6-axis force/torque sensor and streams telemetry data.
 *
 * This program reads data from 12 load cells arranged in a six-axis force/torque sensor 
 * configuration using the MCP356x3axis and MCP356x6axis libraries. It combines readings 
 * from four 3-axis load cells to measure forces and torques in all six degrees of freedom.
 * The measurements are streamed via serial communication in a format suitable for
 * visualization in telemetry software.
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

#include <MCP356x6axis.h>

// Pin Definitions for SPI
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Pin Definitions for ADCs
#define CS_PIN0 7
#define IRQ_PIN0 6
#define CS_PIN1 4
#define IRQ_PIN1 5
#define CS_PIN2 2
#define IRQ_PIN2 3

#define TOTAL_NUM_CELLS 12

// Global variables
MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell1 = nullptr;
MCP356x3axis* loadCell2 = nullptr;
MCP356x3axis* loadCell3 = nullptr;
MCP356x3axis* loadCell4 = nullptr;
MCP356x6axis* sixAxisLoadCell = nullptr;

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, the MCP356xScale instance,
 * and the MCP356x3axis and MCP356x6axis objects. Sets up calibration.
 */
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);

    // Initialize the 3-axis load cells with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 0, 1, 2);
    loadCell2 = new MCP356x3axis(mcpScale, 3, 4, 5);
    loadCell3 = new MCP356x3axis(mcpScale, 6, 7, 8);
    loadCell4 = new MCP356x3axis(mcpScale, 9, 10, 11);

    // Tare operation for all load cells to set the current weight as the zero point
    for (int i = 0; i < TOTAL_NUM_CELLS; i++) { 
        mcpScale->tare(i);
    }

    // Initialize the 6-axis load cell using the four 3-axis load cells
    float plateWidth = 0.125;  // Example plate width in meters
    float plateLength = 0.125; // Example plate length in meters
    sixAxisLoadCell = new MCP356x6axis(loadCell1, loadCell2, loadCell3, loadCell4, plateWidth, plateLength);

    // Define calibration matrices for the 3-axis load cells (if needed)
    // Add calibration code here if required

    Serial.println("Load cells initialized and tared.");
}

/**
 * @brief Main program loop
 * 
 * Continuously updates ADC readings, processes the 6-axis force/torque data,
 * and outputs the readings along with elapsed time for telemetry visualization.
 */
void loop() {
    uint32_t startMicros = micros(); // Start time for this loop iteration

    // Update the ADC readings from all connected load cells
    if (mcpScale->updatedAdcReadings()) {
        StringBuilder output;
        
        // Construct the message with readings for each 3-axis load cell
        for (int i = 1; i <= 4; i++) {
            MCP356x3axis* currentLoadCell = nullptr;
            switch(i) {
                case 1: currentLoadCell = loadCell1; break;
                case 2: currentLoadCell = loadCell2; break;
                case 3: currentLoadCell = loadCell3; break;
                case 4: currentLoadCell = loadCell4; break;
            }

            Matrix<3, 1> gfReading = currentLoadCell->getGfReading();

            // Append the integer values of grams-force for each axis to the output
            output.concat((int)round(gfReading(0)));
            output.concat(',');
            output.concat((int)round(gfReading(1)));
            output.concat(',');
            output.concat((int)round(gfReading(2)));
            if (i < 4) {
                output.concat(','); // Separator between load cell readings
            }
        }

        // Append the elapsed time in microseconds to the output
        output.concat(',');
        int elapsedTime = micros() - startMicros;
        output.concat(elapsedTime);

        // Send the combined grams-force readings and elapsed time for all load cells over serial
        Serial.println((char*)output.string());
    }
}