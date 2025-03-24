/**
 * @file MCP356x3axis.h
 * @brief MCP356x3axis class for interfacing with 3-axis load cells via MCP356x ADCs.
 * 
 * This class provides functionality for working with 3-axis load cells, including
 * calibration, reading forces, and converting between different measurement units.
 * 
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#ifndef MCP356X_3AXIS_H
#define MCP356X_3AXIS_H

#include "MCP356xScale.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

/**
 * @enum CalibrationType
 * @brief Defines different calibration types for 3-axis load cells.
 */
enum CalibrationType { 
    NONE,       ///< No calibration
    MATRIX,     ///< Matrix-based calibration
    POLYNOMIAL  ///< Polynomial calibration
};

/**
 * @struct PolynomialCoefficients
 * @brief Structure holding coefficients for polynomial calibration.
 * 
 * Stores the coefficients for a polynomial: a0 + a1*x + a2*x^2
 */
struct PolynomialCoefficients {
    float a0;  ///< Constant term
    float a1;  ///< Coefficient of x
    float a2;  ///< Coefficient of x^2
};

/**
 * @struct Quaternion
 * @brief Structure representing a quaternion for orientation representation.
 */
struct Quaternion {
    float w;  ///< Scalar component
    float x;  ///< First vector component
    float y;  ///< Second vector component
    float z;  ///< Third vector component
};

/**
 * @class MCP356x3axis
 * @brief Class for interfacing with 3-axis load cells using MCP356x ADCs.
 * 
 * This class provides calibration and reading functionality for 3-axis load cells.
 * It supports both matrix-based and polynomial calibration methods.
 */
class MCP356x3axis {
public:
    /**
     * @brief Constructor for the MCP356x3axis class.
     * 
     * @param scale Pointer to the MCP356xScale instance
     * @param xIndex Index of the X-axis channel
     * @param yIndex Index of the Y-axis channel
     * @param zIndex Index of the Z-axis channel
     */
    MCP356x3axis(MCP356xScale* scale, int xIndex, int yIndex, int zIndex);
    
    /**
     * @brief Destructor for the MCP356x3axis class.
     */
    ~MCP356x3axis();

    /**
     * @brief Sets the calibration matrix for the 3-axis load cell.
     * 
     * The calibration matrix is a 3x3 matrix that transforms raw ADC readings
     * into calibrated force readings.
     * 
     * @param calibMatrix The 3x3 calibration matrix
     */
    void setCalibrationMatrix(const Matrix<3, 3>& calibMatrix);
    
    /**
     * @brief Sets polynomial calibration coefficients for each axis.
     * 
     * Configures the load cell to use polynomial equations for calibration on each axis.
     * 
     * @param xCoeffs Polynomial coefficients for X-axis
     * @param yCoeffs Polynomial coefficients for Y-axis
     * @param zCoeffs Polynomial coefficients for Z-axis
     */
    void setCalibrationPolynomial(const PolynomialCoefficients& xCoeffs, 
                                 const PolynomialCoefficients& yCoeffs, 
                                 const PolynomialCoefficients& zCoeffs);
    
    /**
     * @brief Sets axis inversion flags for each axis.
     * 
     * Enables or disables inversion for each axis. Inverted axes will have their
     * readings multiplied by -1.
     * 
     * @param invertX Whether to invert the X-axis
     * @param invertY Whether to invert the Y-axis
     * @param invertZ Whether to invert the Z-axis
     */
    void setAxisInversion(bool invertX, bool invertY, bool invertZ);

    /**
     * @brief Performs a tare operation on the load cell.
     * 
     * Sets the current reading as the zero point for all axes.
     * 
     * @param numReadings Number of readings to average for the tare operation
     */
    void tare(int numReadings = 100);

    /**
     * @brief Gets the current calibration matrix.
     * 
     * @return Matrix<3, 3> The current calibration matrix
     */
    Matrix<3, 3> getCalibrationMatrix();
    
    /**
     * @brief Converts a force vector to a quaternion representation.
     * 
     * @param forceReading Force vector to convert
     * @return Quaternion Quaternion representing the orientation of the force vector
     */
    Quaternion forceVectorToQuaternion(Matrix<3, 1> forceReading);

    /**
     * @brief Gets raw digital readings from the load cell.
     * 
     * @return Matrix<3, 1> 3D vector of raw digital readings
     */
    Matrix<3, 1> getDigitalReading();
    
    /**
     * @brief Gets calibrated force readings in grams-force (gf).
     * 
     * @return Matrix<3, 1> 3D vector of calibrated force readings in gf
     */
    Matrix<3, 1> getGfReading();
    
    /**
     * @brief Gets calibrated force readings in Newtons (N).
     * 
     * @return Matrix<3, 1> 3D vector of force readings in Newtons
     */
    Matrix<3, 1> getNewtonReading();

    /**
     * @brief Prints the load cell configuration.
     * 
     * Outputs the current configuration of the load cell to the serial port.
     */
    void printLoadCellConfiguration();
    
    /**
     * @brief Prints raw ADC readings for all axes.
     * 
     * Outputs the raw ADC readings for each axis to the serial port.
     */
    void printADCReadings();
    
    /**
     * @brief Prints the current calibration matrix.
     * 
     * Outputs the calibration matrix to the serial port.
     */
    void printCalibrationMatrix();
    
    /**
     * @brief Prints calibrated force readings in grams-force.
     * 
     * Outputs the calibrated force readings for each axis to the serial port.
     */
    void printGfReadings();
    
    /**
     * @brief Prints debug information about calibration.
     * 
     * Outputs detailed calibration information including both matrix and
     * polynomial calibration results for comparison.
     */
    void printCalibrationDebugInfo();

private:
    static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; ///< Standard gravity in m/s²
    
    MCP356xScale* scale;     ///< Pointer to the scale instance
    int xIndex;              ///< Index of the X-axis channel
    int yIndex;              ///< Index of the Y-axis channel
    int zIndex;              ///< Index of the Z-axis channel

    bool isCalibrated;       ///< Whether the load cell is calibrated
    bool invertX;            ///< Whether to invert the X-axis
    bool invertY;            ///< Whether to invert the Y-axis
    bool invertZ;            ///< Whether to invert the Z-axis
    CalibrationType calibrationType; ///< Current calibration type

    Matrix<3, 1> tareOffsets;       ///< Tare offsets for each axis
    Matrix<3, 3> calibrationMatrix; ///< Calibration matrix
    
    PolynomialCoefficients xPolyCoeffs; ///< Polynomial coefficients for X-axis
    PolynomialCoefficients yPolyCoeffs; ///< Polynomial coefficients for Y-axis
    PolynomialCoefficients zPolyCoeffs; ///< Polynomial coefficients for Z-axis
};

#endif // MCP356X_3AXIS_H
