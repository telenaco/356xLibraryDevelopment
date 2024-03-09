#ifndef MCP356X_SCALE_H
#define MCP356X_SCALE_H

#include "MCP356x.h"

// Enumerations
enum class ScaleConversionMode : uint8_t {
    UNDEFINED,
    SINGLE_VALUE,
    LINEAR,
    POLYNOMIAL
};

class MCP356xScale {

  static const int MAX_SCALES = 12; // Maximum number of scales supported
    MCP356x* adcDevices[MAX_SCALES]; // Array to hold pointers to MCP356x instances
    MCP356xChannel channels[MAX_SCALES]; // Array to hold channel assignments for each scale

  public:
    MCP356x       *adcDevice;   // Pointer to the MCP356x device
    MCP356xChannel usedChannel; // The channel used by this scale

    // Constructor
    MCP356xScale(MCP356x *adc, MCP356xChannel channel);

    // Calibration Functions
    void setConversionMode(ScaleConversionMode mode);
    void setScaleFactor(float scale);
    void setLinearCalibration(float slope, float intercept);
    void setPolynomialCalibration(float a, float b, float c);
    void tare();

    // Reading Functions
    int32_t getDigitalValue();
    int32_t getCalibratedValue();
    float   getGramsForce();
    float   getForce();
    int32_t getAverageValue(int samples);

    // Force Conversion Methods
    float convertToSingleValueForce();
    float convertToLinearForce();
    float convertToPolynomialForce();

    // Configuration and Setup
    void setReadingOffset(int32_t offset);

    // Calibration and reading methods
    int32_t getReadingOffset();
    float   getScaleFactor();

  private:
    static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; // m/s^2

    ScaleConversionMode _conversionMode = ScaleConversionMode::UNDEFINED;

    // Channel-specific settings
    float _scale  = 0.0f;
    int   _offset = 0;
    float _linearSlope;
    float _linearIntercept;
    float _polyA;
    float _polyB;
    float _polyC;
};

#endif // MCP356X_SCALE_H