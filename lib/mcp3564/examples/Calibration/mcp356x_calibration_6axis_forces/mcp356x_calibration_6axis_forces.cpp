/**
 * @file mcp356x_calibration_6axis_forces.cpp
 * @brief Six-axis force/torque sensor calibration program using MCP356x ADCs.
 *
 * This program reads data from 12 load cells arranged in a six-axis force/torque
 * configuration using the MCP356x3axis and MCP356x6axis libraries. It calibrates and
 * combines the readings from four 3-axis load cells to provide force and torque
 * measurements in all six degrees of freedom. The calibrated readings are transmitted
 * via serial communication for further analysis or display.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to all MCP356x SDI pins
 * - SDO_PIN (12): Connect to all MCP356x SDO pins
 * - SCK_PIN (13): Connect to all MCP356x SCK pins
 * - CS_PIN0 (7): Connect to first MCP356x CS pin
 * - IRQ_PIN0 (6): Connect to first MCP356x IRQ pin
 * - CS_PIN1 (4): Connect to second MCP356x CS pin
 * - IRQ_PIN1 (5): Connect to second MCP356x IRQ pin
 * - CS_PIN2 (2): Connect to third MCP356x CS pin
 * - IRQ_PIN2 (3): Connect to third MCP356x IRQ pin
 */

#include <MCP356x6axis.h>

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

// ADC and Load Cell Objects
MCP356xScale *mcpScale = nullptr;
MCP356x3axis *loadCell1 = nullptr;
MCP356x3axis *loadCell2 = nullptr;
MCP356x3axis *loadCell3 = nullptr;
MCP356x3axis *loadCell4 = nullptr;
MCP356x6axis *sixAxisLoadCell = nullptr;

// Averaging Variables
unsigned int sampleCount = 0;
Matrix<3, 1> totalReadings1 = {0, 0, 0};
Matrix<3, 1> totalReadings2 = {0, 0, 0};
Matrix<3, 1> totalReadings3 = {0, 0, 0};
Matrix<3, 1> totalReadings4 = {0, 0, 0};

/**
 * @brief Initialize calibration matrices for all load cells
 *
 * Sets up the calibration matrices for each of the four 3-axis load cells.
 * These matrices transform raw ADC readings into calibrated force measurements.
 */
void initializeCalibrationWithMatrices()
{
    // Calibration matrix for Load Cell 1
    BLA::Matrix<3, 3> calibMatrix1 = {
        -5.982118e-04, -6.398772e-06, 1.493637e-05,
        -1.610696e-05, 5.980800e-04, 1.154983e-05,
        5.832123e-06, -5.437381e-06, 6.200539e-04};

    // Calibration matrix for Load Cell 2
    BLA::Matrix<3, 3> calibMatrix2 = {
        5.981983e-04, -1.070736e-05, 7.760431e-06,
        2.589967e-06, 5.869149e-04, 6.217067e-06,
        5.954714e-06, 1.355531e-05, 5.982035e-04};

    // Calibration matrix for Load Cell 3
    BLA::Matrix<3, 3> calibMatrix3 = {
        -5.967505e-04, -1.410111e-05, -1.031143e-05,
        -1.276690e-05, 5.953475e-04, -4.933627e-06,
        8.278846e-06, -1.064470e-05, -5.746296e-04};

    // Calibration matrix for Load Cell 4
    BLA::Matrix<3, 3> calibMatrix4 = {
        -6.007485e-04, -1.092638e-05, -2.390023e-06,
        -1.328377e-05, 5.968730e-04, 5.299463e-06,
        3.958772e-06, -7.053156e-06, 6.141198e-04};

    // Apply calibration matrices to load cells
    loadCell1->setCalibrationMatrix(calibMatrix1);
    loadCell2->setCalibrationMatrix(calibMatrix2);
    loadCell3->setCalibrationMatrix(calibMatrix3);
    loadCell4->setCalibrationMatrix(calibMatrix4);
}

/**
 * @brief Initialize polynomial calibration for all load cells
 *
 * Sets up polynomial calibration parameters for each axis of each load cell.
 * This provides an alternative calibration method to matrix calibration.
 */
void initializeCalibrationWithPolynomial()
{
    // Set Polynomial Calibration for Load Cell 1
    loadCell1->setCalibrationPolynomial(
        {-2.828916e+02, -6.139373e-04, -4.779867e-12}, // X Axis
        {3.592542e+02, 5.951776e-04, 1.063187e-11},    // Y Axis
        {-5.722137e+02, 6.244786e-04, -9.493906e-12}   // Z Axis
    );

    // Set Polynomial Calibration for Load Cell 2
    loadCell2->setCalibrationPolynomial(
        {-1.517346e+02, 6.002052e-04, -7.213967e-13}, // X Axis
        {3.217731e+02, 5.941050e-04, -7.526178e-12},  // Y Axis
        {-6.883740e+02, 5.964100e-04, 4.769434e-12}   // Z Axis
    );

    // Set Polynomial Calibration for Load Cell 3
    loadCell3->setCalibrationPolynomial(
        {4.924497e+02, -6.000414e-04, -5.842038e-12}, // X Axis
        {4.065573e+02, 5.873697e-04, -3.809444e-12},  // Y Axis
        {-3.722284e+02, -5.611561e-04, 4.556780e-12}  // Z Axis
    );

    // Set Polynomial Calibration for Load Cell 4
    loadCell4->setCalibrationPolynomial(
        {-1.973620e+01, -6.040589e-04, -8.463602e-13}, // X Axis
        {5.351286e+02, 5.982422e-04, 4.519417e-12},    // Y Axis
        {-2.636953e+02, 5.996329e-04, 5.869018e-12}    // Z Axis
    );

    // Tare all load cells to establish baseline readings
    loadCell1->tare(100);
    loadCell2->tare(100);
    loadCell3->tare(100);
    loadCell4->tare(100);
}

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, the MCP356xScale instance,
 * and the four 3-axis load cells. Applies calibration and configures
 * the six-axis force/torque sensor.
 */
void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN,
                                IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);
    mcpScale->tare(100000);

    // Initialize the 3-axis load cells with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 9, 10, 11);
    loadCell2 = new MCP356x3axis(mcpScale, 0, 1, 2);
    loadCell3 = new MCP356x3axis(mcpScale, 3, 4, 5);
    loadCell4 = new MCP356x3axis(mcpScale, 6, 7, 8);

    // Choose calibration method (uncomment desired method)
    initializeCalibrationWithMatrices();
    // initializeCalibrationWithPolynomial();

    // Set axis inversion flags for each load cell to align coordinate systems
    loadCell1->setAxisInversion(1, 1, 0);
    loadCell2->setAxisInversion(0, 1, 1);
    loadCell3->setAxisInversion(0, 0, 0);
    loadCell4->setAxisInversion(1, 1, 1);

    // Initialize the 6-axis load cell using the four 3-axis load cells
    float plateWidth = 0.125;  // Plate width in meters
    float plateLength = 0.125; // Plate length in meters
    sixAxisLoadCell = new MCP356x6axis(loadCell1, loadCell2, loadCell3, loadCell4,
                                       plateWidth, plateLength);

    // Define the 6-axis calibration matrix
    Matrix<6, 6> calibMatrix = {
        0.99962, 0.010247, 0.0125986, -0.000360691, -0.0217137, 0.00507714,
        -0.0302539, 0.994631, 0.0150176, 0.025775, 0.000985963, 0.00494589,
        -0.0090635, -0.0158627, 0.994607, 0.00113301, -0.00941599, -0.0000305863,
        -0.113791, -0.111746, -0.116821, 1.14718, 0.0195911, -0.0137317,
        -0.401589, -0.0569284, 0.588532, 0.0120469, 1.15636, 0.0465735,
        -0.0173116, 0.00683699, 0.106079, 0.0170052, -0.00432406, 1.00594};

    // Apply the calibration matrix to the 6-axis sensor
    sixAxisLoadCell->setCalibrationMatrix(calibMatrix);
}

/**
 * @brief Main program loop
 *
 * Continuously reads from the ADCs, processes the 6-axis force/torque data,
 * and sends the readings over serial communication.
 */
void loop()
{
    // Check for updated ADC readings
    if (mcpScale->updatedAdcReadings())
    {
        // Read calibrated force and torque values from the 6-axis sensor
        BLA::Matrix<6, 1> forceAndTorque = sixAxisLoadCell->readCalibratedForceAndTorque();

        // Prepare and send the data string including microsecond timestamp and force/torque readings
        StringBuilder output;
        output.concatf("%lu,", micros()); // Add microsecond timestamp

        // Add the six force and torque readings
        for (int i = 0; i < 6; i++)
        {
            output.concatf("%.3f,", forceAndTorque(i));
        }

        // Output the formatted string
        Serial.println((char *)output.string());
    }
}