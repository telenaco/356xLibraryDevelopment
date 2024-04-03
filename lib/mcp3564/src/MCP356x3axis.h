#ifndef MCP356X_3AXIS_H
#define MCP356X_3AXIS_H

#include "MCP356xScale.h"
#include <BasicLinearAlgebra.h>

using namespace BLA;

enum CalibrationType { NONE, MATRIX, POLYNOMIAL };

struct PolynomialCoefficients {
    float a0, a1, a2; // Coefficients for the polynomial: a0 + a1*x + a2*x^2
};

struct Quaternion {
    float w, x, y, z;
};

class MCP356x3axis {
public:
    MCP356x3axis(MCP356xScale *scale, int xIndex, int yIndex, int zIndex);
    ~MCP356x3axis();

    // Calibration functions
    void setCalibrationMatrix(const Matrix<3, 3>& calibMatrix);
    void setCalibrationPolynomial(const PolynomialCoefficients& xCoeffs, const PolynomialCoefficients& yCoeffs, const PolynomialCoefficients& zCoeffs);
    void setAxisInversion(bool invertX, bool invertY, bool invertZ);

    void tare(int numReadings = 100); 


    Matrix<3, 3> getCalibrationMatrix();
    Quaternion forceVectorToQuaternion(Matrix<3, 1> forceReading);

    // Reading functions
    Matrix<3, 1> getDigitalReading();
    Matrix<3, 1> getGfReading();
    Matrix<3, 1> getNewtonReading();

    // Debug functions
    void printLoadCellConfiguration();
    void printADCReadings();
    void printCalibrationMatrix();
    void printGfReadings();
    void printCalibrationDebugInfo();

private:

    static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; // m/s^2
    
    MCP356xScale* scale;
    int xIndex;
    int yIndex;
    int zIndex;

    bool isCalibrated;
            // Axis inversion flags
    bool invertX, invertY, invertZ;
    CalibrationType calibrationType; // Added to distinguish between calibration types

    Matrix<3, 1> tareOffsets;

    Matrix<3, 3> calibrationMatrix;
    PolynomialCoefficients xPolyCoeffs, yPolyCoeffs, zPolyCoeffs;


    // Utility functions
    Matrix<3, 1> convertToNewtons(const Matrix<3, 1> &forceVector);
};

#endif // MCP356X_3AXIS_H