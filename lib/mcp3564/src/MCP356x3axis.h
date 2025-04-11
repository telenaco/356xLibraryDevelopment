/**
 * @file MCP356x3axis.h
 * @brief MCP356x3axis class for interfacing with 3-axis load cells via MCP356x ADCs.
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
     NONE,       // No calibration
     MATRIX,     // Matrix-based calibration
     POLYNOMIAL  // Polynomial calibration
 };
 
 /**
  * @struct PolynomialCoefficients
  * @brief Structure holding coefficients for polynomial calibration.
  * 
  * Stores the coefficients for a polynomial: a0 + a1*x + a2*x^2
  */
 struct PolynomialCoefficients {
     float a0;  // Constant term
     float a1;  // Coefficient of x
     float a2;  // Coefficient of x^2
 };
 
 /**
  * @struct Quaternion
  * @brief Structure representing a quaternion for orientation representation.
  */
 struct Quaternion {
     float w;  // Scalar component
     float x;  // First vector component
     float y;  // Second vector component
     float z;  // Third vector component
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
 
     void setCalibrationMatrix(const Matrix<3, 3>& calibMatrix);
     void setCalibrationPolynomial(const PolynomialCoefficients& xCoeffs, 
                                  const PolynomialCoefficients& yCoeffs, 
                                  const PolynomialCoefficients& zCoeffs);
     void setAxisInversion(bool invertX, bool invertY, bool invertZ);
     void tare(int numReadings = 100);
 
     Matrix<3, 3> getCalibrationMatrix();
     Quaternion forceVectorToQuaternion(Matrix<3, 1> forceReading);
     Matrix<3, 1> getDigitalReading();
     Matrix<3, 1> getGfReading();
     Matrix<3, 1> getNewtonReading();
 
     void printLoadCellConfiguration();
     void printADCReadings();
     void printCalibrationMatrix();
     void printGfReadings();
     void printCalibrationDebugInfo();
 
 private:
     static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; // Standard gravity in m/s²
     
     MCP356xScale* scale;     // Pointer to the scale instance
     int xIndex;              // Index of the X-axis channel
     int yIndex;              // Index of the Y-axis channel
     int zIndex;              // Index of the Z-axis channel
 
     bool isCalibrated;       // Whether the load cell is calibrated
     bool invertX;            // Whether to invert the X-axis
     bool invertY;            // Whether to invert the Y-axis
     bool invertZ;            // Whether to invert the Z-axis
     CalibrationType calibrationType; // Current calibration type
 
     Matrix<3, 1> tareOffsets;       // Tare offsets for each axis
     Matrix<3, 3> calibrationMatrix; // Calibration matrix
     
     PolynomialCoefficients xPolyCoeffs; // Polynomial coefficients for X-axis
     PolynomialCoefficients yPolyCoeffs; // Polynomial coefficients for Y-axis
     PolynomialCoefficients zPolyCoeffs; // Polynomial coefficients for Z-axis
 };
 
 #endif // MCP356X_3AXIS_H