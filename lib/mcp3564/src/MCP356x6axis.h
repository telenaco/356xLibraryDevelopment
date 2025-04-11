/**
 * @file MCP356x6axis.h
 * @brief Class declaration for interfacing with 6-axis force/torque sensors using MCP356x ADCs.
 *
 * This header file contains the declaration for the MCP356x6axis class, which provides
 * functionality for reading and calibrating 6-axis force/torque sensors built using
 * four 3-axis load cells connected to MCP356x ADCs.
 *
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#ifndef __MCP356X_6AXIS_H__
#define __MCP356X_6AXIS_H__

#include "MCP356x3axis.h"

/**
 * @class MCP356x6axis
 * @brief Class for interfacing with 6-axis force/torque sensors using MCP356x ADCs.
 *
 * This class provides functionality for reading and calibrating 6-axis force/torque
 * sensors built using four 3-axis load cells. It combines readings from the individual
 * load cells and applies a calibration matrix to obtain force and torque readings in
 * all six degrees of freedom.
 */
class MCP356x6axis
{
public:
    MCP356x6axis(MCP356x3axis *loadCellA, MCP356x3axis *loadCellB,
                 MCP356x3axis *loadCellC, MCP356x3axis *loadCellD,
                 float plateWidth, float plateLength);
    ~MCP356x6axis();

    void setCalibrationMatrix(const BLA::Matrix<6, 6> &calibMatrix);
    BLA::Matrix<6, 1> readRawForceAndTorque();
    BLA::Matrix<6, 1> readCalibratedForceAndTorque();
    void tare(int numReadings = 100);
    void reset();

    // Additional methods implemented in the .cpp file
    void printCalibrationMatrix();
    void printForceAndTorque();
    void printLoadCellConfiguration();

private:
    MCP356x3axis *_loadCellA; // Load cell at corner A (front-left)
    MCP356x3axis *_loadCellB; // Load cell at corner B (front-right)
    MCP356x3axis *_loadCellC; // Load cell at corner C (back-right)
    MCP356x3axis *_loadCellD; // Load cell at corner D (back-left)

    float _plateWidth;  // Width of the sensor plate in meters
    float _plateLength; // Length of the sensor plate in meters

    BLA::Matrix<6, 6> _calibrationMatrix; // 6x6 calibration matrix
    bool _isCalibrated;                   // Whether the sensor is calibrated
};

#endif // __MCP356X_6AXIS_H__