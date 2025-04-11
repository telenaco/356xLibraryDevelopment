/**
 * @file mcp356x_four_triaxial_sensor_plate.cpp
 * @brief Reads and combines data from four 3-axis load cells for a force sensor plate.
 *
 * This program reads data from 12 load cell channels (4 triaxial load cells) using the 
 * MCP356x3axis library and combines the readings into a force plate configuration. It applies
 * appropriate calibration, calculates force vectors, and streams the output data via serial.
 * Quaternion representations are included to visualize force directions.
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
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN0 7
#define IRQ_PIN0 6
#define CS_PIN1 4
#define IRQ_PIN1 5
#define CS_PIN2 2
#define IRQ_PIN2 3

#define TOTAL_NUM_CELLS 12
#define SAMPLE_INTERVAL 10 // Sample interval in milliseconds

// Global variables
MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell1 = nullptr;
MCP356x3axis* loadCell2 = nullptr;
MCP356x3axis* loadCell3 = nullptr;
MCP356x3axis* loadCell4 = nullptr;

// Variables for averaging
unsigned int sampleCount = 0;
Matrix<3, 1> totalReadings1 = { 0, 0, 0 };
Matrix<3, 1> totalReadings2 = { 0, 0, 0 };
Matrix<3, 1> totalReadings3 = { 0, 0, 0 };
Matrix<3, 1> totalReadings4 = { 0, 0, 0 };

/**
 * @brief Initialize matrix calibration for all load cells.
 * 
 * Sets up calibration matrices for the four 3-axis load cells.
 */
void initializeCalibrationWithMatrices() {
    // For Load Cell 1
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
}

/**
 * @brief Initialize polynomial calibration for all load cells.
 * 
 * Sets up polynomial calibration coefficients for the four 3-axis load cells.
 */
void initializeCalibrationWithPolynomial() {
    // Set Polynomial Calibration for Load Cell 1
    loadCell1->setCalibrationPolynomial(
        { -2.828916e+02, -6.139373e-04, -4.779867e-12 }, // X Axis
        { 3.592542e+02, 5.951776e-04, 1.063187e-11 },    // Y Axis
        { -5.722137e+02, 6.244786e-04, -9.493906e-12 }   // Z Axis
    );

    // Set Polynomial Calibration for Load Cell 2
    loadCell2->setCalibrationPolynomial(
        { -1.517346e+02, 6.002052e-04, -7.213967e-13 }, // X Axis
        { 3.217731e+02, 5.941050e-04, -7.526178e-12 },  // Y Axis
        { -6.883740e+02, 5.964100e-04, 4.769434e-12 }   // Z Axis
    );

    // Set Polynomial Calibration for Load Cell 3
    loadCell3->setCalibrationPolynomial(
        { 4.924497e+02, -6.000414e-04, -5.842038e-12 }, // X Axis
        { 4.065573e+02, 5.873697e-04, -3.809444e-12 },  // Y Axis
        { -3.722284e+02, -5.611561e-04, 4.556780e-12 }  // Z Axis
    );

    // Set Polynomial Calibration for Load Cell 4
    loadCell4->setCalibrationPolynomial(
        { -1.973620e+01, -6.040589e-04, -8.463602e-13 }, // X Axis
        { 5.351286e+02, 5.982422e-04, 4.519417e-12 },    // Y Axis
        { -2.636953e+02, 5.996329e-04, 5.869018e-12 }    // Z Axis
    );
    
    // Tare all load cells
    loadCell1->tare(100);
    loadCell2->tare(100);
    loadCell3->tare(100);
    loadCell4->tare(100);
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, the MCP356xScale instance,
 * the MCP356x3axis objects, and applies calibration.
 */
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);
    mcpScale->tare(100000);

    // Initialize the 3-axis load cell with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 9, 10, 11);
    loadCell2 = new MCP356x3axis(mcpScale, 0, 1, 2);
    loadCell3 = new MCP356x3axis(mcpScale, 3, 4, 5);
    loadCell4 = new MCP356x3axis(mcpScale, 6, 7, 8);

    // Choose calibration method
    // Uncomment desired method:
    initializeCalibrationWithMatrices();
    // initializeCalibrationWithPolynomial();

    // Set axis inversion for each load cell to align coordinate systems
    loadCell1->setAxisInversion(1, 1, 0);
    loadCell2->setAxisInversion(0, 1, 1);
    loadCell3->setAxisInversion(0, 0, 0);
    loadCell4->setAxisInversion(1, 1, 1);
}

/**
 * @brief Main program loop
 * 
 * Continuously updates ADC readings, averages sample readings over intervals,
 * and outputs the force data along with quaternion representations.
 */
void loop() {
    static unsigned long lastSampleTime = 0;

    // Process tare command if available
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command.startsWith("T,")) {
            int tareValue = command.substring(2).toInt(); // Extract number after "T,"
            if (tareValue > 0) {
                mcpScale->tare(tareValue); // Execute tare with specified iterations
            }
        }
    }

    // Update the ADC readings from all connected load cells
    if (mcpScale->updatedAdcReadings()) {
        sampleCount++;

        // Accumulate the readings for each load cell
        totalReadings1 += loadCell1->getGfReading();
        totalReadings2 += loadCell2->getGfReading();
        totalReadings3 += loadCell3->getGfReading();
        totalReadings4 += loadCell4->getGfReading();

        unsigned long currentTime = millis();

        if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
            lastSampleTime = currentTime;

            // Calculate the average readings for each load cell
            Matrix<3, 1> averageReadings1;
            Matrix<3, 1> averageReadings2;
            Matrix<3, 1> averageReadings3;
            Matrix<3, 1> averageReadings4;

            for (int i = 0; i < 3; i++) {
                averageReadings1(i) = totalReadings1(i) / sampleCount;
                averageReadings2(i) = totalReadings2(i) / sampleCount;
                averageReadings3(i) = totalReadings3(i) / sampleCount;
                averageReadings4(i) = totalReadings4(i) / sampleCount;
            }

            // Reset the total readings and sample count for the next interval
            totalReadings1 = { 0, 0, 0 };
            totalReadings2 = { 0, 0, 0 };
            totalReadings3 = { 0, 0, 0 };
            totalReadings4 = { 0, 0, 0 };
            sampleCount = 0;

            // Convert force vectors to quaternions for visualization
            Quaternion q1 = loadCell1->forceVectorToQuaternion(averageReadings1);
            Quaternion q2 = loadCell2->forceVectorToQuaternion(averageReadings2);
            Quaternion q3 = loadCell3->forceVectorToQuaternion(averageReadings3);
            Quaternion q4 = loadCell4->forceVectorToQuaternion(averageReadings4);

            // Format and send output data
            char text[512]; // Adjusted size to ensure ample space for the formatted string
            snprintf(text, sizeof(text),
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                (int)round(averageReadings1(0)), (int)round(averageReadings1(1)), (int)round(averageReadings1(2)),
                (int)round(averageReadings2(0)), (int)round(averageReadings2(1)), (int)round(averageReadings2(2)),
                (int)round(averageReadings3(0)), (int)round(averageReadings3(1)), (int)round(averageReadings3(2)),
                (int)round(averageReadings4(0)), (int)round(averageReadings4(1)), (int)round(averageReadings4(2)),
                // Quaternion components for each load cell to show force direction
                q1.w, q1.x, q1.y, q1.z,
                q2.w, q2.x, q2.y, q2.z,
                q3.w, q3.x, q3.y, q3.z,
                q4.w, q4.x, q4.y, q4.z
            );

            Serial.println(text);
        }
    }
}