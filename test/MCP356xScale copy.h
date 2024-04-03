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
  public:
    static const size_t MAX_ADC_COUNT = 3;
    static const int MAX_CHANNELS_PER_ADC = 4;
    static const int MAX_SCALES = 12;

    // Default ADC configuration to be edited. 
    static MCP356xConfig defaultConfig;

    // Revised constructor to take the total number of scales
    MCP356xScale(int totalScales, int SCK_PIN, int SDO_PIN, int SDI_PIN, int irq_0, int cs_0, int irq_1 = 0, int cs_1 = 0, int irq_2 = 0, int cs_2 = 0);
    ~MCP356xScale();

    void setupChannelMappings();
    int  updateAdcsReadings();

    // Calibration Functions
    void setConversionMode(ScaleConversionMode mode);
    void setScaleFactor(int globalChannelIndex, float scale);
    void setLinearCalibration(int scaleIndex, float slope, float intercept);
    void setPolynomialCalibration(int scaleIndex, float a, float b, float c);
    void tare(int scaleIndex);
    void tareAll();
    void setReadingOffset(int scaleIndex, int32_t offset);

    // Reading Functions
    double getDigitalValue(int globalChannelIndex);
    int32_t getCalibratedValue(int scaleIndex, int32_t offset);
    float   getGramsForce(int scaleIndex);
    float   getForce(int scaleIndex);
    int32_t getAverageValue(int scaleIndex, int times);

    // Force Conversion Methods
    float convertToSingleValueForce(int scaleIndex);
    float convertToLinearForce(int scaleIndex);
    float convertToPolynomialForce(int scaleIndex);

    // Configuration and Setup
    int32_t getReadingOffset(int scaleIndex);

    // Calibration and reading methods
    int32_t getReadingOffset();
    float   getScaleFactor();

    struct ChannelMapping {
    int adcIndex;
    MCP356xChannel localChannel;
    };


  private:

    // method for dynamic ADC setup
    void setupADCs(int totalScales);
    static MCP356x* adcDevices[MAX_ADC_COUNT]; // Static allocation of ADC instances
    static bool isInitialized[MAX_ADC_COUNT]; // Tracks initialization status of each ADC
    static ChannelMapping channelMappings[MAX_SCALES];


    static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; // m/s^2

    ScaleConversionMode _conversionModes[MAX_SCALES];

    int adcCount = 0;
    int totalChannels = 0; 

    // Channel-specific settings
    float scaleFactors[MAX_SCALES];
    float scales[MAX_SCALES]  ;
    int32_t   offsets[MAX_SCALES];
    float linearSlopes[MAX_SCALES];
    float linearIntercepts[MAX_SCALES];
    float polyA[MAX_SCALES];
    float polyB[MAX_SCALES];
    float polyC[MAX_SCALES];
};


#endif // MCP356X_SCALE_H