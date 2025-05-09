/**
 * @file mcp356x_triaxial_load_cell_calibration.cpp
 * @brief Demonstrates the usage of the MCP356x3axis library to read and calibrate 3-axis load cell data.
 *
 * This program initializes instances of the MCP356xScale and MCP356x3axis classes to manage
 * multiple load cells and a 3-axis load cell setup. It performs a tare operation for all connected
 * load cells and then continuously retrieves and compensates the ADC readings based on a predefined
 * calibration matrix. The compensated readings, representing forces in grams, are printed every
 * 500 milliseconds.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - CS_PIN0 (7): Connect to MCP356x CS (Chip Select) for the first load cell
 * - IRQ_PIN0 (6): Connect to MCP356x IRQ (Interrupt Request) for the first load cell
 */

#include <MCP356x3axis.h>

// Pin Definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN0 7
#define IRQ_PIN0 6

#define TOTAL_NUM_CELLS 3

// Global variables
MCP356xScale *mcpScale = nullptr;
MCP356x3axis *loadCell1 = nullptr;

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, the MCP356xScale instance,
 * the MCP356x3axis object, and applies a calibration matrix.
 */
void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0);

    // Tare operation for all load cells to set the current weight as the zero point
    for (int i = 0; i < TOTAL_NUM_CELLS; i++)
    {
        mcpScale->tare(i);
    }

    // Initialize the 3-axis load cell with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 0, 1, 2);

    // Define calibration matrix
    BLA::Matrix<3, 3> calibMatrix = {
        5.98198338e-04, -1.07073615e-05, 7.76043063e-06,
        2.58996659e-06, 5.86914897e-04, 6.21706670e-06,
        5.95471435e-06, 1.35553078e-05, 5.98203473e-04};

    // Apply calibration matrix to the load cell
    loadCell1->setCalibrationMatrix(calibMatrix);

    Serial.println("\nLoad cells initialized and tared.\n");

    // Print the calibration matrix for reference
    loadCell1->printCalibrationMatrix();
}

/**
 * @brief Main program loop
 *
 * Continuously updates ADC readings and outputs calibrated grams-force
 * readings for all three axes of the load cell.
 */
void loop()
{
    static uint32_t lastPrintTime = millis();

    // Update the ADC readings from all connected load cells
    mcpScale->updatedAdcReadings();

    if (millis() - lastPrintTime >= 500)
    {
        lastPrintTime = millis();

        // Get the grams-force reading from the 3-axis load cell
        Matrix<3, 1> gfReading = loadCell1->getGfReading();

        // Use StringBuilder to create the output string for grams-force readings
        StringBuilder outputGF;
        outputGF.concat("GF Reading:\n");
        outputGF.concatf("3axisX %.2f\n", gfReading(0)); // X in grams-force
        outputGF.concatf("3axisY %.2f\n", gfReading(1)); // Y in grams-force
        outputGF.concatf("3axisZ %.2f\n", gfReading(2)); // Z in grams-force
        Serial.print((char *)outputGF.string());

        // Directly print the grams force readings using the built-in method
        loadCell1->printGfReadings();

        // Print a newline for better readability in the serial monitor
        Serial.println();
    }
}