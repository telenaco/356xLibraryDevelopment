#include "MCP356x3axis.h"

MCP356x3axis::MCP356x3axis(MCP356xScale* scale, int xIndex, int yIndex, int zIndex)
    : invertX(false), invertY(false), invertZ(false),
    scale(scale), xIndex(xIndex), yIndex(yIndex), zIndex(zIndex),
    isCalibrated(false), calibrationType(NONE) {
    calibrationMatrix = Identity<3, 3>();
    tareOffsets = {0, 0, 0};
}

MCP356x3axis::~MCP356x3axis()
{
    // No dynamic memory allocation, so no need for explicit cleanup
}

// Set calibration matrix
void MCP356x3axis::setCalibrationMatrix(const Matrix<3, 3>& calibMatrix) {
    calibrationMatrix = calibMatrix;
    isCalibrated = true;
    calibrationType = MATRIX;
}

Matrix<3, 3> MCP356x3axis::getCalibrationMatrix()
{
    return calibrationMatrix;
}

Matrix<3, 1> MCP356x3axis::getDigitalReading() {
    Matrix<3, 1> reading;
    reading(0) = scale->getReading(xIndex);
    reading(1) = scale->getReading(yIndex);
    reading(2) = scale->getReading(zIndex);
    return reading;
}

void MCP356x3axis::setAxisInversion(bool invertX, bool invertY, bool invertZ) {
    this->invertX = invertX;
    this->invertY = invertY;
    this->invertZ = invertZ;
}

// Set polynomial calibration
void MCP356x3axis::setCalibrationPolynomial(const PolynomialCoefficients& xCoeffs, const PolynomialCoefficients& yCoeffs, const PolynomialCoefficients& zCoeffs) {
    xPolyCoeffs = xCoeffs;
    yPolyCoeffs = yCoeffs;
    zPolyCoeffs = zCoeffs;
    isCalibrated = true;
    calibrationType = POLYNOMIAL;
}

void MCP356x3axis::tare(int numReadings) {

    // Ensure the scale is zeroed out at the hardware level
    Matrix<3, 1> sumReadings = {0, 0, 0};
    tareOffsets = {0, 0, 0};

    for (int i = 0; i < numReadings; ++i) {
        auto reading = this->getGfReading(); 
        sumReadings += reading;
    }
    // to calculate the average and store it in tareOffsets
    for (int i = 0; i < 3; i++) {
        this->tareOffsets(i) = sumReadings(i) / numReadings;
    }
}

Matrix<3, 1> MCP356x3axis::getGfReading() {
    Matrix<3, 1> digitalReading = getDigitalReading();
    if (!isCalibrated) return digitalReading;

    Matrix<3, 1> calibratedReading = Identity<3, 1>();;

    if (calibrationType == MATRIX) {
        // Apply matrix calibration
        calibratedReading = calibrationMatrix * digitalReading;
    }
    else if (calibrationType == POLYNOMIAL) {
        // Apply polynomial calibration directly for each axis using stored coefficients
        calibratedReading(0) = xPolyCoeffs.a0 + xPolyCoeffs.a1 * digitalReading(0) + xPolyCoeffs.a2 * digitalReading(0) * digitalReading(0);
        calibratedReading(1) = yPolyCoeffs.a0 + yPolyCoeffs.a1 * digitalReading(1) + yPolyCoeffs.a2 * digitalReading(1) * digitalReading(1);
        calibratedReading(2) = zPolyCoeffs.a0 + zPolyCoeffs.a1 * digitalReading(2) + zPolyCoeffs.a2 * digitalReading(2) * digitalReading(2);
    }

    calibratedReading -= this->tareOffsets;

    // Apply axis inversion if set
    if (invertX) calibratedReading(0) = -calibratedReading(0);
    if (invertY) calibratedReading(1) = -calibratedReading(1);
    if (invertZ) calibratedReading(2) = -calibratedReading(2);


    return calibratedReading;
}

Matrix<3, 1> MCP356x3axis::getNewtonReading()
{
    Matrix<3, 1> gfReading = getGfReading();
    return gfReading * (GRAVITATIONAL_ACCELERATION / 1000.0f);
}

Quaternion MCP356x3axis::forceVectorToQuaternion(Matrix<3, 1> forceReading) {
    // not quite there will come back to this later on... 
    // Define the initial forward vector
    Matrix<3, 1> initialForward = {1, 0, 0}; 
    
    // Calculate the magnitude of the force vector
    float magnitude = sqrt(forceReading(0) * forceReading(0) + forceReading(1) * forceReading(1) + forceReading(2) * forceReading(2));
    
    // Define a low-force threshold to filter out noise (e.g., 20 grams)
    float noiseThreshold = 20.0;
    if (magnitude < noiseThreshold) {
        // Below the threshold, return a neutral quaternion indicating no significant rotation
        return Quaternion{1, 0, 0, 0}; // Represents no rotation
    }
    
    // Normalize the force vector
    Matrix<3, 1> normForce = {forceReading(0) / magnitude, forceReading(1) / magnitude, forceReading(2) / magnitude};
    
    // Compute cross product of initialForward and normForce
    Matrix<3, 1> cross = {
        initialForward(1) * normForce(2) - initialForward(2) * normForce(1),
        initialForward(2) * normForce(0) - initialForward(0) * normForce(2),
        initialForward(0) * normForce(1) - initialForward(1) * normForce(0)
    };
    
    // Compute dot product of initialForward and normForce
    float dot = initialForward(0) * normForce(0) + initialForward(1) * normForce(1) + initialForward(2) * normForce(2);
    
    // Compute the magnitude of the cross product vector (sin of angle between vectors)
    float sinAngleMagnitude = sqrt(cross(0) * cross(0) + cross(1) * cross(1) + cross(2) * cross(2));
    
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

void MCP356x3axis::printLoadCellConfiguration() {
    StringBuilder output;
    output.concat("Load Cell Configuration:\n");
    output.concatf("X Index: %d\n", xIndex);
    output.concatf("Y Index: %d\n", yIndex);
    output.concatf("Z Index: %d\n", zIndex);
    output.concat("Is Calibrated: ");
    output.concat(isCalibrated ? "Yes\n" : "No\n");
    Serial.println((char*)output.string());
}

void MCP356x3axis::printADCReadings() {
    Matrix<3, 1> readings = getDigitalReading();
    StringBuilder output;
    output.concat("ADC Readings:\n");
    output.concatf("X: %.2f\n", readings(0));
    output.concatf("Y: %.2f\n", readings(1));
    output.concatf("Z: %.2f\n", readings(2));
    Serial.print((char*)output.string());
}

void MCP356x3axis::printGfReadings() {
    Matrix<3, 1> readings = getGfReading();
    StringBuilder output;
    output.concat("GF Readings:\n");
    output.concatf("X: %.2f\n", readings(0));
    output.concatf("Y: %.2f\n", readings(1));
    output.concatf("Z: %.2f\n", readings(2));
    Serial.print((char*)output.string());
}

void MCP356x3axis::printCalibrationMatrix() {
    StringBuilder output;
    output.concat("Calibration Matrix:\n");
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            output.concatf("%.2e\t", calibrationMatrix(row, col));
        }
        output.concat("\n");
    }
    Serial.println((char*)output.string());
}

void MCP356x3axis::printCalibrationDebugInfo() {
    Matrix<3, 1> digitalReading = getDigitalReading();

    // Calculate using matrix calibration
    Matrix<3, 1> matrixCalibratedReading = calibrationMatrix * digitalReading;

    // Calculate using polynomial calibration
    Matrix<3, 1> polynomialCalibratedReading;
    polynomialCalibratedReading(0) = xPolyCoeffs.a0 + xPolyCoeffs.a1 * digitalReading(0) + xPolyCoeffs.a2 * digitalReading(0) * digitalReading(0);
    polynomialCalibratedReading(1) = yPolyCoeffs.a0 + yPolyCoeffs.a1 * digitalReading(1) + yPolyCoeffs.a2 * digitalReading(1) * digitalReading(1);
    polynomialCalibratedReading(2) = zPolyCoeffs.a0 + zPolyCoeffs.a1 * digitalReading(2) + zPolyCoeffs.a2 * digitalReading(2) * digitalReading(2);

    // Apply tare offsets
    //matrixCalibratedReading -= this->tareOffsets;
    //polynomialCalibratedReading -= this->tareOffsets;

    //     if (invertX) {
    //     matrixCalibratedReading(0) = -matrixCalibratedReading(0);
    //     polynomialCalibratedReading(0) = -polynomialCalibratedReading(0);
    // }
    // if (invertY) {
    //     matrixCalibratedReading(1) = -matrixCalibratedReading(1);
    //     polynomialCalibratedReading(1) = -polynomialCalibratedReading(1);
    // }
    // if (invertZ) {
    //     matrixCalibratedReading(2) = -matrixCalibratedReading(2);
    //     polynomialCalibratedReading(2) = -polynomialCalibratedReading(2);
    // }

    // Print formatted string
    char text[512];
    snprintf(text, sizeof(text),
             "%.0f,%.0f,%.0f,%.0f,%.0f,%.0f",
             matrixCalibratedReading(0), matrixCalibratedReading(1), matrixCalibratedReading(2),
             polynomialCalibratedReading(0), polynomialCalibratedReading(1), polynomialCalibratedReading(2));

    Serial.println(text);
}
