
#ifndef MCP356X_3AXIS_H
#define MCP356X_3AXIS_H

#include "MCP356xScale.h"

// Struct for 3D readings in Newtons
struct Newton3D {
    float x;
    float y;
    float z;
};

// Struct for 3D readings in grams-force
struct Force3D {
    float x;
    float y;
    float z;
};

// Struct for raw 3D digital readings
struct Reading3D {
    float x;
    float y;
    float z;
};

class MCP356x3axis {
public:
    MCP356x3axis(MCP356xScale *xScale, MCP356xScale *yScale, MCP356xScale *zScale);
    ~MCP356x3axis();

    // Calibration functions
    void setCalibrationCoefficients(const float coeffsX[3], const float coeffsY[3], const float coeffsZ[3]);
    void getCalibrationCoefficients(float coeffsX[3], float coeffsY[3], float coeffsZ[3]);
    void tareScales();

    // Reading functions
    Reading3D digitalReading3D();
    Force3D forceData3D();
    Newton3D newtonData3D();

private:
    MCP356xScale *xScale;
    MCP356xScale *yScale;
    MCP356xScale *zScale;

    bool isCalibrated;

    // Calibration coefficients
    float coeffsX[3];
    float coeffsY[3];
    float coeffsZ[3];

    // Utility functions
    float calculateForce(float reading, const float coeffs[3]);
    Newton3D convertToNewtons(const Force3D &force);
};

#endif // MCP356X_3AXIS_H

