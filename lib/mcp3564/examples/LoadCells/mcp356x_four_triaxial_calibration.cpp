/**
 * @file mcp356x_four_triaxial_calibration.cpp
 * @brief Demonstrates calibration and reading of four 3-axis load cells using MCP356x ADCs.
 *
 * This program initializes and calibrates four 3-axis load cells connected to MCP356x ADCs.
 * It applies matrix-based calibration to each 3-axis load cell and outputs the calibrated
 * force readings in grams-force (gf) for each axis of each load cell.
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

#include <MCP356x3axis.h>

// Pin Definitions
#define SDI_PIN  11
#define SDO_PIN  12
#define SCK_PIN  13
#define CS_PIN0  7
#define IRQ_PIN0 6
#define CS_PIN1  4
#define IRQ_PIN1 5
#define CS_PIN2  2
#define IRQ_PIN2 3

#define TOTAL_NUM_CELLS 12

// Global variables
MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell1 = nullptr;
MCP356x3axis* loadCell2 = nullptr;
MCP356x3axis* loadCell3 = nullptr;
MCP356x3axis* loadCell4 = nullptr;

uint32_t lastPrintTime = 0;

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, the MCP356xScale instance,
 * and the MCP356x3axis objects. Applies calibration and tare operations.
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
        mcpScale->tare(i);
    }

    // Initialize the 3-axis load cell with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 0, 1, 2);
    loadCell2 = new MCP356x3axis(mcpScale, 3, 4, 5);
    loadCell3 = new MCP356x3axis(mcpScale, 6, 7, 8);
    loadCell4 = new MCP356x3axis(mcpScale, 9, 10, 11);

    // Calibration matrix for Load Cell 1
    BLA::Matrix<3, 3> calibMatrix1 = {
       -5.982118e-04,  -6.398772e-06, 1.493637e-05,
       -1.610696e-05,   5.980800e-04, 1.154983e-05,
        5.832123e-06,  -5.437381e-06, 6.200539e-04 };

    // For Load Cell 2
    BLA::Matrix<3, 3> calibMatrix2 = {
         5.981983e-04, -1.070736e-05, 7.760431e-06,
         2.589967e-06,  5.869149e-04, 6.217067e-06,
         5.954714e-06,  1.355531e-05, 5.982035e-04 };

    // For Load Cell 3
    BLA::Matrix<3, 3> calibMatrix3 = {
        -5.967505e-04, -1.410111e-05, -1.031143e-05,
        -1.276690e-05,  5.953475e-04, -4.933627e-06,
         8.278846e-06, -1.064470e-05, -5.746296e-04 };

    // For Load Cell 4
    BLA::Matrix<3, 3> calibMatrix4 = {
        -6.007485e-04, -1.092638e-05, -2.390023e-06,
        -1.328377e-05,  5.968730e-04,  5.299463e-06,
         3.958772e-06, -7.053156e-06, 6.141198e-04 };
    
    // Apply calibration matrices to load cells
    loadCell1->setCalibrationMatrix(calibMatrix1);
    loadCell2->setCalibrationMatrix(calibMatrix2);
    loadCell3->setCalibrationMatrix(calibMatrix3);
    loadCell4->setCalibrationMatrix(calibMatrix4);

    Serial.println("Load cells initialized and tared.");
}

/**
 * @brief Main program loop
 * 
 * Continuously updates ADC readings and outputs calibrated grams-force 
 * readings for all three axes of each of the four load cells.
 */
void loop() {
    // Update the ADC readings from all connected load cells
    mcpScale->updatedAdcReadings();

    // Check if time has passed to print the readings
    if (millis() - lastPrintTime >= 500) {
        lastPrintTime = millis();

        // Assuming a fixed width for each value
        const int valueWidth = 10; // 10 for padding
        
        StringBuilder output;
        output.concat("GF Readings:\n");

        // Read and print grams-force for each 3-axis load cell with fixed width
        for (int i = 1; i <= 4; i++) {
            MCP356x3axis* currentLoadCell = nullptr;
            switch(i) {
                case 1: currentLoadCell = loadCell1; break;
                case 2: currentLoadCell = loadCell2; break;
                case 3: currentLoadCell = loadCell3; break;
                case 4: currentLoadCell = loadCell4; break;
            }
            
            Matrix<3, 1> gfReading = currentLoadCell->getGfReading();
            output.concatf("Load Cell %d: ", i);
            
            for (int j = 0; j < 3; j++) {
                // Ensure the buffer is large enough for the formatted string
                char formattedNumber[valueWidth + 4]; // +4 for '.', sign, and two decimal places
                snprintf(formattedNumber, sizeof(formattedNumber), "%*.*f", valueWidth, 2, gfReading(j));
                output.concat(formattedNumber);
                if (j < 2) {
                    output.concat(" gf\t"); // Separator between values
                } else {
                    output.concat(" gf\n"); // New line after the last value
                }
            }
        }

        // Print the combined grams-force readings for all load cells
        Serial.println((char*)output.string());
    }
}