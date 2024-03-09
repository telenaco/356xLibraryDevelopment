#ifndef THREE_AXIS_LOAD_CELL_H
#define THREE_AXIS_LOAD_CELL_H

#include "MCP356xScale.h"

// returns 3 readins on newton
struct Newton3D {
    float x;
    float y;
    float z;
};

// returns 3 readings on gramsForce
struct Force3D {
    float x;
    float y;
    float z;
};

// returns 3 readings as the raw digital reading from the load cell
struct Reading3D {
    float x;
    float y;
    float z;
};

class MCP356x3axis {
  public:
    MCP356x3axis(MCP356xScale *xScale, MCP356xScale *yScale, MCP356xScale *zScale);
    ~MCP356x3axis();

    Reading3D digitalReading3D(); // Get raw digital readings
    Force3D   ForceData3D();      // Get force data in grams
    Newton3D  NewtonData3D();     // Get force data in Newtons
    void      tareScales();       // Tare (zero) the scales

    // Set and Get Calibration Coefficients
    void setCalibrationCoefficients(const float coeffsX[3], const float coeffsY[3], const float coeffsZ[3]);
    void getCalibrationCoefficients(float coeffsX[3], float coeffsY[3], float coeffsZ[3]);

  private:
    MCP356xScale *xScale;
    MCP356xScale *yScale;
    MCP356xScale *zScale;

    bool _isCalibrated = false;


    // Utility functions
    float    calculateForce(float reading, const float coeffs[3]); // Calculate force using polynomial coefficients
    Newton3D convertToNewtons(const Force3D &force);               // Convert Force3D from grams to Newtons
};

#endif // THREE_AXIS_LOAD_CELL_H