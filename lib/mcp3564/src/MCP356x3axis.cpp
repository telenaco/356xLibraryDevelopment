/**
 * @file MCP356x3axis.cpp
 * @brief Implementation of the MCP356x3axis class for interfacing with 3-axis load cells via MCP356x ADCs.
 * 
 * This file contains the implementation of the MCP356x3axis class, which provides
 * functionality for calibrating and reading data from 3-axis load cells connected
 * to MCP356x ADCs.
 * 
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

 #include "MCP356x3axis.h"

 /**
  * @brief Constructor for the MCP356x3axis class.
  * 
  * Initializes a 3-axis load cell with the specified scale and axis indices.
  * Sets default values for calibration and axis inversion flags.
  * 
  * @param scale Pointer to the MCP356xScale instance
  * @param xIndex Index of the X-axis channel
  * @param yIndex Index of the Y-axis channel
  * @param zIndex Index of the Z-axis channel
  */
 MCP356x3axis::MCP356x3axis(MCP356xScale* scale, int xIndex, int yIndex, int zIndex)
     : _invertX(false), _invertY(false), _invertZ(false),
     _scale(scale), _xIndex(xIndex), _yIndex(yIndex), _zIndex(zIndex),
     _isCalibrated(false), _calibrationType(NONE) {
     
     // Initialize calibration matrix to identity matrix
     _calibrationMatrix = Identity<3, 3>();
     
     // Initialize tare offsets to zero
     _tareOffsets = {0, 0, 0};
 }
 
 /**
  * @brief Destructor for the MCP356x3axis class.
  * 
  * As no dynamic memory allocation is performed by this class,
  * no specific cleanup is needed.
  */
 MCP356x3axis::~MCP356x3axis()
 {
     // No dynamic memory allocation, so no need for explicit cleanup
 }
 
 /**
  * @brief Sets the calibration matrix for the 3-axis load cell.
  * 
  * The calibration matrix is a 3x3 matrix that transforms raw ADC readings
  * into calibrated force readings.
  * 
  * @param calibMatrix The 3x3 calibration matrix
  */
 void MCP356x3axis::setCalibrationMatrix(const Matrix<3, 3>& calibMatrix) {
     _calibrationMatrix = calibMatrix;
     _isCalibrated = true;
     _calibrationType = MATRIX;
 }
 
 /**
  * @brief Gets the current calibration matrix.
  * 
  * @return Matrix<3, 3> The current calibration matrix
  */
 Matrix<3, 3> MCP356x3axis::getCalibrationMatrix()
 {
     return _calibrationMatrix;
 }
 
 /**
  * @brief Gets raw digital readings from the load cell.
  * 
  * @return Matrix<3, 1> 3D vector of raw digital readings
  */
 Matrix<3, 1> MCP356x3axis::getDigitalReading() {
     Matrix<3, 1> reading;
     reading(0) = _scale->getReading(_xIndex);
     reading(1) = _scale->getReading(_yIndex);
     reading(2) = _scale->getReading(_zIndex);
     return reading;
 }
 
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
 void MCP356x3axis::setAxisInversion(bool invertX, bool invertY, bool invertZ) {
     _invertX = invertX;
     _invertY = invertY;
     _invertZ = invertZ;
 }
 
 /**
  * @brief Sets polynomial calibration coefficients for each axis.
  * 
  * Configures the load cell to use polynomial equations for calibration on each axis.
  * 
  * @param xCoeffs Polynomial coefficients for X-axis
  * @param yCoeffs Polynomial coefficients for Y-axis
  * @param zCoeffs Polynomial coefficients for Z-axis
  */
 void MCP356x3axis::setCalibrationPolynomial(const PolynomialCoefficients& xCoeffs, 
                                            const PolynomialCoefficients& yCoeffs, 
                                            const PolynomialCoefficients& zCoeffs) {
     _xPolyCoeffs = xCoeffs;
     _yPolyCoeffs = yCoeffs;
     _zPolyCoeffs = zCoeffs;
     _isCalibrated = true;
     _calibrationType = POLYNOMIAL;
 }
 
 /**
  * @brief Performs a tare operation on the load cell.
  * 
  * Sets the current reading as the zero point for all axes.
  * 
  * @param numReadings Number of readings to average for the tare operation
  */
 void MCP356x3axis::tare(int numReadings) {
     // Ensure the scale is zeroed out at the hardware level
     Matrix<3, 1> sumReadings = {0, 0, 0};
     _tareOffsets = {0, 0, 0};
 
     for (int i = 0; i < numReadings; ++i) {
         auto reading = this->getGfReading(); 
         sumReadings += reading;
     }
     
     // Calculate the average and store it in tareOffsets
     for (int i = 0; i < 3; i++) {
         _tareOffsets(i) = sumReadings(i) / numReadings;
     }
 }
 
 /**
  * @brief Gets calibrated force readings in grams-force (gf).
  * 
  * Applies calibration and tare offsets to raw readings to obtain
  * calibrated force values in grams-force.
  * 
  * @return Matrix<3, 1> 3D vector of calibrated force readings in gf
  */
 Matrix<3, 1> MCP356x3axis::getGfReading() {
     Matrix<3, 1> digitalReading = getDigitalReading();
     if (!_isCalibrated) return digitalReading;
 
     Matrix<3, 1> calibratedReading = Identity<3, 1>();
 
     if (_calibrationType == MATRIX) {
         // Apply matrix calibration
         calibratedReading = _calibrationMatrix * digitalReading;
     }
     else if (_calibrationType == POLYNOMIAL) {
         // Apply polynomial calibration directly for each axis using stored coefficients
         calibratedReading(0) = _xPolyCoeffs.a0 + _xPolyCoeffs.a1 * digitalReading(0) + _xPolyCoeffs.a2 * digitalReading(0) * digitalReading(0);
         calibratedReading(1) = _yPolyCoeffs.a0 + _yPolyCoeffs.a1 * digitalReading(1) + _yPolyCoeffs.a2 * digitalReading(1) * digitalReading(1);
         calibratedReading(2) = _zPolyCoeffs.a0 + _zPolyCoeffs.a1 * digitalReading(2) + _zPolyCoeffs.a2 * digitalReading(2) * digitalReading(2);
     }
 
     // Apply tare offsets
     calibratedReading -= _tareOffsets;
 
     // Apply axis inversion if set
     if (_invertX) calibratedReading(0) = -calibratedReading(0);
     if (_invertY) calibratedReading(1) = -calibratedReading(1);
     if (_invertZ) calibratedReading(2) = -calibratedReading(2);
 
     return calibratedReading;
 }
 
 /**
  * @brief Gets calibrated force readings in Newtons (N).
  * 
  * Converts calibrated grams-force readings to Newtons using the gravitational
  * constant.
  * 
  * @return Matrix<3, 1> 3D vector of force readings in Newtons
  */
 Matrix<3, 1> MCP356x3axis::getNewtonReading()
 {
     Matrix<3, 1> gfReading = getGfReading();
     return gfReading * (GRAVITATIONAL_ACCELERATION / 1000.0f);
 }
 
 /**
  * @brief Converts a force vector to a quaternion representation.
  * 
  * This function converts a force vector into a quaternion representation,
  * which can be used to visualize the direction of the force.
  * 
  * @param forceReading Force vector to convert
  * @return Quaternion Quaternion representing the orientation of the force vector
  */
 Quaternion MCP356x3axis::forceVectorToQuaternion(Matrix<3, 1> forceReading) {
     // Define the initial forward vector
     Matrix<3, 1> initialForward = {1, 0, 0}; 
     
     // Calculate the magnitude of the force vector
     float magnitude = sqrt(forceReading(0) * forceReading(0) + 
                           forceReading(1) * forceReading(1) + 
                           forceReading(2) * forceReading(2));
     
     // Define a low-force threshold to filter out noise (e.g., 20 grams)
     float noiseThreshold = 20.0;
     if (magnitude < noiseThreshold) {
         // Below the threshold, return a neutral quaternion indicating no significant rotation
         return Quaternion{1, 0, 0, 0}; // Represents no rotation
     }
     
     // Normalize the force vector
     Matrix<3, 1> normForce = {
         forceReading(0) / magnitude, 
         forceReading(1) / magnitude, 
         forceReading(2) / magnitude
     };
     
     // Compute cross product of initialForward and normForce
     Matrix<3, 1> cross = {
         initialForward(1) * normForce(2) - initialForward(2) * normForce(1),
         initialForward(2) * normForce(0) - initialForward(0) * normForce(2),
         initialForward(0) * normForce(1) - initialForward(1) * normForce(0)
     };
     
     // Compute dot product of initialForward and normForce
     float dot = initialForward(0) * normForce(0) + 
                initialForward(1) * normForce(1) + 
                initialForward(2) * normForce(2);
     
     // Compute the magnitude of the cross product vector (sin of angle between vectors)
     float sinAngleMagnitude = sqrt(cross(0) * cross(0) + 
                                   cross(1) * cross(1) + 
                                   cross(2) * cross(2));
     
     // Compute the quaternion components
     Quaternion q;
     q.x = cross(0);
     q.y = cross(1);
     q.z = cross(2);
     q.w = sqrt((1 + dot) * 2);
     
     float invNorm = 1.0f / sqrt(q.w * q.w + sinAngleMagnitude * sinAngleMagnitude);
     q.w *= invNorm;
     q.x *= invNorm;
     q.y *= invNorm;
     q.z *= invNorm;
     
     return q;
 }
 
 /**
  * @brief Prints the load cell configuration.
  * 
  * Outputs the current configuration of the load cell to the serial port.
  */
 void MCP356x3axis::printLoadCellConfiguration() {
     StringBuilder output;
     output.concat("Load Cell Configuration:\n");
     output.concatf("X Index: %d\n", _xIndex);
     output.concatf("Y Index: %d\n", _yIndex);
     output.concatf("Z Index: %d\n", _zIndex);
     output.concat("Is Calibrated: ");
     output.concat(_isCalibrated ? "Yes\n" : "No\n");
     Serial.println((char*)output.string());
 }
 
 /**
  * @brief Prints raw ADC readings for all axes.
  * 
  * Outputs the raw ADC readings for each axis to the serial port.
  */
 void MCP356x3axis::printADCReadings() {
     Matrix<3, 1> readings = getDigitalReading();
     StringBuilder output;
     output.concat("ADC Readings:\n");
     output.concatf("X: %.2f\n", readings(0));
     output.concatf("Y: %.2f\n", readings(1));
     output.concatf("Z: %.2f\n", readings(2));
     Serial.print((char*)output.string());
 }
 
 /**
  * @brief Prints calibrated force readings in grams-force.
  * 
  * Outputs the calibrated force readings for each axis to the serial port.
  */
 void MCP356x3axis::printGfReadings() {
     Matrix<3, 1> readings = getGfReading();
     StringBuilder output;
     output.concat("GF Readings:\n");
     output.concatf("X: %.2f\n", readings(0));
     output.concatf("Y: %.2f\n", readings(1));
     output.concatf("Z: %.2f\n", readings(2));
     Serial.print((char*)output.string());
 }
 
 /**
  * @brief Prints the current calibration matrix.
  * 
  * Outputs the calibration matrix to the serial port.
  */
 void MCP356x3axis::printCalibrationMatrix() {
     StringBuilder output;
     output.concat("Calibration Matrix:\n");
     for (int row = 0; row < 3; ++row) {
         for (int col = 0; col < 3; ++col) {
             output.concatf("%.2e\t", _calibrationMatrix(row, col));
         }
         output.concat("\n");
     }
     Serial.println((char*)output.string());
 }
 
 /**
  * @brief Prints debug information about calibration.
  * 
  * Outputs detailed calibration information including both matrix and
  * polynomial calibration results for comparison.
  */
 void MCP356x3axis::printCalibrationDebugInfo() {
     Matrix<3, 1> digitalReading = getDigitalReading();
 
     // Calculate using matrix calibration
     Matrix<3, 1> matrixCalibratedReading = _calibrationMatrix * digitalReading;
 
     // Calculate using polynomial calibration
     Matrix<3, 1> polynomialCalibratedReading;
     polynomialCalibratedReading(0) = _xPolyCoeffs.a0 + _xPolyCoeffs.a1 * digitalReading(0) + _xPolyCoeffs.a2 * digitalReading(0) * digitalReading(0);
     polynomialCalibratedReading(1) = _yPolyCoeffs.a0 + _yPolyCoeffs.a1 * digitalReading(1) + _yPolyCoeffs.a2 * digitalReading(1) * digitalReading(1);
     polynomialCalibratedReading(2) = _zPolyCoeffs.a0 + _zPolyCoeffs.a1 * digitalReading(2) + _zPolyCoeffs.a2 * digitalReading(2) * digitalReading(2);
 
     // Print formatted string
     char text[512];
     snprintf(text, sizeof(text),
              "%.0f,%.0f,%.0f,%.0f,%.0f,%.0f",
              matrixCalibratedReading(0), matrixCalibratedReading(1), matrixCalibratedReading(2),
              polynomialCalibratedReading(0), polynomialCalibratedReading(1), polynomialCalibratedReading(2));
 
     Serial.println(text);
 }