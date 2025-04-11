/**
 * @file MCP356x6axis.h
 * @brief MCP356x6axis class for interfacing with 6-axis force/torque sensors via MCP356x ADCs.
 *
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#ifndef MCP356X_6AXIS_H
#define MCP356X_6AXIS_H

#include "MCP356x3axis.h"

/**
 * @class MCP356x6axis
 * @brief Class for creating a 6-axis force/torque sensor using four 3-axis load cells.
 *
 * This class combines readings from four 3-axis load cells to measure forces (Fx, Fy, Fz)
 * and torques (Mx, My, Mz) in all six degrees of freedom.
 */
class MCP356x6axis
{
public:
    /**
     * @brief Constructor for the MCP356x6axis class.
     *
     * @param loadCellA Pointer to the first load cell (typically front-left)
     * @param loadCellB Pointer to the second load cell (typically front-right)
     * @param loadCellC Pointer to the third load cell (typically back-right)
     * @param loadCellD Pointer to the fourth load cell (typically back-left)
     * @param plateWidth Width of the mounting plate in meters
     * @param plateLength Length of the mounting plate in meters
     */
    MCP356x6axis(MCP356x3axis *loadCellA, MCP356x3axis *loadCellB,
                 MCP356x3axis *loadCellC, MCP356x3axis *loadCellD,
                 float plateWidth, float plateLength);

    /**
     * @brief Destructor for the MCP356x6axis class.
     */
    ~MCP356x6axis();

    void setCalibrationMatrix(const BLA::Matrix<6, 6> &calibMatrix);
    BLA::Matrix<6, 1> readRawForceAndTorque();
    BLA::Matrix<6, 1> readCalibratedForceAndTorque();
    void tare(int numReadings = 100);
    void reset();

    void printCalibrationMatrix();
    void printForceAndTorque();
    void printLoadCellConfiguration();

private:
    MCP356x3axis *_loadCellA; // Pointer to the first load cell (front-left)
    MCP356x3axis *_loadCellB; // Pointer to the second load cell (front-right)
    MCP356x3axis *_loadCellC; // Pointer to the third load cell (back-right)
    MCP356x3axis *_loadCellD; // Pointer to the fourth load cell (back-left)

    float _plateWidth;  // Width of the mounting plate in meters
    float _plateLength; // Length of the mounting plate in meters
    bool _isCalibrated; // Whether the sensor is calibrated

    BLA::Matrix<6, 6> _calibrationMatrix; // 6x6 calibration matrix
};

#endif // MCP356X_6AXIS_H