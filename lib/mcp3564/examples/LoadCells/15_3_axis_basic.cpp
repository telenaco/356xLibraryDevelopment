/**
 * @file 14_3_axis_basic.cpp
 * @brief Demonstrates basic usage of the MCP356x3axis library to read 3-axis load cell data.
 *
 * This example initializes an instance of the MCP356xScale class for managing multiple load cells and an MCP356x3axis object for a 3-axis load cell. It performs a tare operation for all load cells and continuously updates the ADC readings. The program prints the digital readings from the 3-axis load cell and the tare-corrected readings from each load cell every 500 milliseconds.
 *
 * Hardware Connections:
 * - SDI_PIN : Connect to MCP356x SDI (Serial Data Input).
 * - SDO_PIN : Connect to MCP356x SDO (Serial Data Output).
 * - SCK_PIN : Connect to MCP356x SCK (Serial Clock Input).
 * - CS_PIN0 : Connect to MCP356x CS (Chip Select) for the first load cell.
 * - IRQ_PIN0: Connect to MCP356x IRQ (Interrupt Request) for the first load cell.
 */

#include <MCP356x3axis.h>

// Pin Definitions
#define SDI_PIN  11
#define SDO_PIN  12
#define SCK_PIN  13
#define CS_PIN0  7
#define IRQ_PIN0 6

#define TOTAL_NUM_CELLS 3

MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell1 = nullptr;

uint32_t lastPrintTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0);

    // Tare operation for all load cells to set the current weight as the zero point
    for (int i = 0; i < TOTAL_NUM_CELLS; i++) {
        mcpScale->tare(i);
    }

    // Initialize the 3-axis load cell with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 0, 1, 2);

    Serial.println("Load cells initialized and tared.");
}

void loop() {
    uint32_t currentTime = millis();

    // Update the ADC readings from all connected load cells
    mcpScale->updatedAdcReadings();

    // Check if it's time to print the readings
    if (currentTime - lastPrintTime >= 500) {
        lastPrintTime = currentTime;

        // Get the digital reading from the 3-axis load cell
        Matrix<3, 1> DigRead = loadCell1->getDigitalReading();

        // Print the digital readings from the 3-axis load cell on one line
        StringBuilder outputForce1;
        outputForce1.concat("DigRed 1:\t");
        outputForce1.concatf("3axisX\t%.0f\t", DigRead(0)); // X
        outputForce1.concatf("3axisY\t%.0f\t", DigRead(1)); // Y
        outputForce1.concatf("3axisZ\t%.0f\n", DigRead(2));   // Z
        Serial.print((char*)outputForce1.string());

        // Prepare the output for individual load cells' readings on the next line
        StringBuilder outputLoadCells;
        outputLoadCells.concat("Load Cells:\t");
        for (int i = 0; i < TOTAL_NUM_CELLS; i++) {
            double reading = mcpScale->getReading(i);
            outputLoadCells.concatf("loadCell%d\t%.0f\t", i + 1, reading);
        }
        outputLoadCells.concat("\n"); // End the line for Load Cells readings

        // Print the tare-corrected readings from each load cell
        Serial.print((char*)outputLoadCells.string());
    }
}
