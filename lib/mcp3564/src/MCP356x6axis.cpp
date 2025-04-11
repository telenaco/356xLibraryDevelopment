/**
 * @file MCP356x6axis.cpp
 * @brief Implementation of the MCP356x6axis class for interfacing with 6-axis force/torque sensors.
 *
 * This file contains the implementation of the MCP356x6axis class, which uses four 3-axis
 * load cells to create a 6-axis force/torque sensor (measuring forces Fx, Fy, Fz and
 * torques Mx, My, Mz).
 *
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#include "MCP356x6axis.h"

/**
 * @brief Constructor for the MCP356x6axis class.
 *
 * Initializes a 6-axis force/torque sensor using four 3-axis load cells.
 * The load cells should be arranged in a rectangular pattern.
 *
 * @param loadCellA Pointer to the first load cell (typically front-left)
 * @param loadCellB Pointer to the second load cell (typically front-right)
 * @param loadCellC Pointer to the third load cell (typically back-right)
 * @param loadCellD Pointer to the fourth load cell (typically back-left)
 * @param plateWidth Width of the mounting plate in meters
 * @param plateLength Length of the mounting plate in meters
 */
MCP356x6axis::MCP356x6axis(MCP356x3axis *loadCellA, MCP356x3axis *loadCellB,
                           MCP356x3axis *loadCellC, MCP356x3axis *loadCellD,
                           float plateWidth, float plateLength)
    : _loadCellA(loadCellA), _loadCellB(loadCellB),
      _loadCellC(loadCellC), _loadCellD(loadCellD),
      _plateWidth(plateWidth), _plateLength(plateLength),
      _isCalibrated(false)
{
    // Initialize calibration matrix to identity matrix
    _calibrationMatrix = BLA::Identity<6, 6>();
}

/**
 * @brief Destructor for the MCP356x6axis class.
 *
 * As no dynamic memory allocation is performed by this class,
 * no specific cleanup is needed.
 */
MCP356x6axis::~MCP356x6axis()
{
    // No dynamic memory allocation, so no need for explicit cleanup
}

/**
 * @brief Sets the calibration matrix for the 6-axis force/torque sensor.
 *
 * The calibration matrix is a 6x6 matrix that transforms raw sensor readings
 * into calibrated force and torque values.
 *
 * @param calibMatrix The 6x6 calibration matrix
 */
void MCP356x6axis::setCalibrationMatrix(const BLA::Matrix<6, 6> &calibMatrix)
{
    _calibrationMatrix = calibMatrix;
    _isCalibrated = true;
}

/**
 * @brief Reads the raw force and torque values from the load cells.
 *
 * Combines the readings from all four 3-axis load cells to produce
 * a 6-component vector containing the raw forces (Fx, Fy, Fz) and
 * torques (Mx, My, Mz).
 *
 * @return BLA::Matrix<6, 1> 6D vector of raw force and torque readings
 */
BLA::Matrix<6, 1> MCP356x6axis::readRawForceAndTorque()
{
    BLA::Matrix<6, 1> rawForceAndTorque;

    // Get force readings from each load cell in Newtons
    BLA::Matrix<3, 1> forceA = _loadCellA->getNewtonReading();
    BLA::Matrix<3, 1> forceB = _loadCellB->getNewtonReading();
    BLA::Matrix<3, 1> forceC = _loadCellC->getNewtonReading();
    BLA::Matrix<3, 1> forceD = _loadCellD->getNewtonReading();

    // Calculate total forces along each axis
    float totalFx = forceA(0) + forceB(0) + forceC(0) + forceD(0);
    float totalFy = forceA(1) + forceB(1) + forceC(1) + forceD(1);
    float totalFz = forceA(2) + forceB(2) + forceC(2) + forceD(2);

    // Calculate moments around each axis
    // Assume the following load cell positions:
    // A: (-width/2, -length/2)  B: (width/2, -length/2)
    // D: (-width/2, length/2)   C: (width/2, length/2)
    float halfWidth = _plateWidth / 2.0f;
    float halfLength = _plateLength / 2.0f;

    // Moments calculation
    float Mx = halfLength * (forceA(2) + forceB(2) - forceC(2) - forceD(2));
    float My = halfWidth * (-forceA(2) + forceB(2) + forceC(2) - forceD(2));
    float Mz = halfWidth * (forceA(1) - forceB(1) + forceC(1) - forceD(1)) +
               halfLength * (forceA(0) + forceB(0) - forceC(0) - forceD(0));

    // Populate the force/torque vector
    rawForceAndTorque(0) = totalFx;
    rawForceAndTorque(1) = totalFy;
    rawForceAndTorque(2) = totalFz;
    rawForceAndTorque(3) = Mx;
    rawForceAndTorque(4) = My;
    rawForceAndTorque(5) = Mz;

    return rawForceAndTorque;
}

/**
 * @brief Reads the calibrated force and torque values.
 *
 * Applies the calibration matrix to the raw force and torque readings
 * to get calibrated values.
 *
 * @return BLA::Matrix<6, 1> 6D vector of calibrated force and torque readings
 */
BLA::Matrix<6, 1> MCP356x6axis::readCalibratedForceAndTorque()
{
    BLA::Matrix<6, 1> rawForceAndTorque = readRawForceAndTorque();

    if (_isCalibrated)
    {
        return _calibrationMatrix * rawForceAndTorque;
    }
    else
    {
        return rawForceAndTorque;
    }
}

/**
 * @brief Performs a tare operation on all load cells.
 *
 * Sets the current reading as the zero point for all load cells.
 *
 * @param numReadings Number of readings to average for the tare operation
 */
void MCP356x6axis::tare(int numReadings)
{
    _loadCellA->tare(numReadings);
    _loadCellB->tare(numReadings);
    _loadCellC->tare(numReadings);
    _loadCellD->tare(numReadings);
}

/**
 * @brief Resets the calibration matrix to identity.
 *
 * Clears any previously set calibration data.
 */
void MCP356x6axis::reset()
{
    _calibrationMatrix = BLA::Identity<6, 6>();
    _isCalibrated = false;
}

/**
 * @brief Prints the current calibration matrix.
 *
 * Outputs the 6x6 calibration matrix to the serial port.
 */
void MCP356x6axis::printCalibrationMatrix()
{
    StringBuilder output;
    output.concat("6-Axis Calibration Matrix:\n");

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            output.concatf("%.4f\t", _calibrationMatrix(row, col));
        }
        output.concat("\n");
    }

    Serial.println((char *)output.string());
}

/**
 * @brief Prints the current force and torque readings.
 *
 * Outputs the current calibrated force and torque values to the serial port.
 */
void MCP356x6axis::printForceAndTorque()
{
    BLA::Matrix<6, 1> forceAndTorque = readCalibratedForceAndTorque();

    StringBuilder output;
    output.concat("Force and Torque Readings:\n");
    output.concatf("Fx: %.4f N\n", forceAndTorque(0));
    output.concatf("Fy: %.4f N\n", forceAndTorque(1));
    output.concatf("Fz: %.4f N\n", forceAndTorque(2));
    output.concatf("Mx: %.4f Nm\n", forceAndTorque(3));
    output.concatf("My: %.4f Nm\n", forceAndTorque(4));
    output.concatf("Mz: %.4f Nm\n", forceAndTorque(5));

    Serial.println((char *)output.string());
}

/**
 * @brief Prints the load cell configuration.
 *
 * Outputs configuration information for the 6-axis sensor including
 * the plate dimensions and calibration status.
 */
void MCP356x6axis::printLoadCellConfiguration()
{
    StringBuilder output;
    output.concat("6-Axis Load Cell Configuration:\n");
    output.concatf("Plate Width: %.4f m\n", _plateWidth);
    output.concatf("Plate Length: %.4f m\n", _plateLength);
    output.concat("Is Calibrated: ");
    output.concat(_isCalibrated ? "Yes\n" : "No\n");

    Serial.println((char *)output.string());

    output.clear();
    output.concat("\nIndividual Load Cell Configurations:\n");
    output.concat("Load Cell A (Front-Left):\n");
    Serial.println((char *)output.string());
    _loadCellA->printLoadCellConfiguration();

    output.clear();
    output.concat("Load Cell B (Front-Right):\n");
    Serial.println((char *)output.string());
    _loadCellB->printLoadCellConfiguration();

    output.clear();
    output.concat("Load Cell C (Back-Right):\n");
    Serial.println((char *)output.string());
    _loadCellC->printLoadCellConfiguration();

    output.clear();
    output.concat("Load Cell D (Back-Left):\n");
    Serial.println((char *)output.string());
    _loadCellD->printLoadCellConfiguration();
}