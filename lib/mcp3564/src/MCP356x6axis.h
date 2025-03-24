/**
 * @file MCP356x6axis.h
 * @brief MCP356x6axis class for implementing a 6-axis force/torque sensor using multiple 3-axis load cells.
 * 
 * This class provides functionality for integrating multiple 3-axis load cells into
 * a 6-axis force/torque sensor. It handles calibration, reading forces and torques,
 * and coordinate transformations.
 * 
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#ifndef SIX_AXIS_LOAD_CELL_H
#define SIX_AXIS_LOAD_CELL_H

#include "MCP356x3axis.h"

/**
 * @class MCP356x6axis
 * @brief Class for implementing a 6-axis force/torque sensor using multiple 3-axis load cells.
 * 
 * This class combines readings from four 3-axis load cells to calculate forces and torques
 * in all six degrees of freedom (Fx, Fy, Fz, Mx, My, Mz). It also provides calibration
 * capabilities for the combined sensor.
 */
class MCP356x6axis {
public:
    /**
     * @brief Constructor for the MCP356x6axis class.
     * 
     * Initializes a 6-axis force/torque sensor using four 3-axis load cells.
     * 
     * @param loadCellA Pointer to the first 3-axis load cell
     * @param loadCellB Pointer to the second 3-axis load cell
     * @param loadCellC Pointer to the third 3-axis load cell
     * @param loadCellD Pointer to the fourth 3-axis load cell
     * @param plateWidth Width of the mounting plate in meters
     * @param plateLength Length of the mounting plate in meters
     */
    MCP356x6axis(MCP356x3axis* loadCellA, MCP356x3axis* loadCellB, 
                MCP356x3axis* loadCellC, MCP356x3axis* loadCellD, 
                float plateWidth, float plateLength);
    
    /**
     * @brief Destructor for the MCP356x6axis class.
     */
    ~MCP356x6axis();

    /**
     * @brief Sets the calibration matrix for the 6-axis sensor.
     * 
     * The calibration matrix is a 6x6 matrix that transforms raw 6-axis readings
     * into calibrated force and torque readings.
     * 
     * @param calibMatrix The 6x6 calibration matrix
     */
    void setCalibrationMatrix(const BLA::Matrix<6, 6>& calibMatrix);
    
    /**
     * @brief Gets the current calibration matrix.
     * 
     * @return BLA::Matrix<6, 6> The current calibration matrix
     */
    BLA::Matrix<6, 6> getCalibrationMatrix();

    /**
     * @brief Reads raw force and torque values from the load cells.
     * 
     * Combines the readings from the four 3-axis load cells to calculate
     * the raw forces and torques in all six axes.
     * 
     * @return BLA::Matrix<6, 1> Vector of raw force and torque values
     *                          [Fx, Fy, Fz, Mx, My, Mz]
     */
    BLA::Matrix<6, 1> readRawForceAndTorque();
    
    /**
     * @brief Reads calibrated force and torque values from the load cells.
     * 
     * Applies the calibration matrix to the raw force and torque readings
     * to obtain calibrated values.
     * 
     * @return BLA::Matrix<6, 1> Vector of calibrated force and torque values
     *                          [Fx, Fy, Fz, Mx, My, Mz]
     */
    BLA::Matrix<6, 1> readCalibratedForceAndTorque();

    /**
     * @brief Performs a tare operation on all load cells.
     * 
     * Sets the current readings as the zero point for all load cells.
     */
    void tare();
    
    /**
     * @brief Resets the calibration for all load cells and the 6-axis sensor.
     * 
     * Clears all calibration data and returns to uncalibrated state.
     */
    void reset();

private:
    MCP356x3axis* _loadCellA;  ///< Pointer to the first 3-axis load cell
    MCP356x3axis* _loadCellB;  ///< Pointer to the second 3-axis load cell
    MCP356x3axis* _loadCellC;  ///< Pointer to the third 3-axis load cell
    MCP356x3axis* _loadCellD;  ///< Pointer to the fourth 3-axis load cell

    float _plateWidth;         ///< Width of the mounting plate in meters
    float _plateLength;        ///< Length of the mounting plate in meters

    BLA::Matrix<6, 6> _calibrationMatrix;  ///< Calibration matrix for the 6-axis sensor
};

#endif // SIX_AXIS_LOAD_CELL_H
