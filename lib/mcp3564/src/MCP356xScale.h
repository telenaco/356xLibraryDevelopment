#ifndef MCP356X_SCALE_H
#define MCP356X_SCALE_H

#include "MCP356x.h"

class MCP356xScale : public MCP356x {

  public:
    static const MCP356xChannel A = MCP356xChannel::DIFF_A;
    static const MCP356xChannel B = MCP356xChannel::DIFF_B;
    static const MCP356xChannel C = MCP356xChannel::DIFF_C;
    static const MCP356xChannel D = MCP356xChannel::DIFF_D;
    
    // Constructor
    MCP356xScale(const MCP356xConfig &config) : MCP356x(config){};
    // Enumerations
    enum class ScaleConversionMode : uint8_t {
        UNDEFINED,
        SINGLE_VALUE,
        LINEAR,
        POLYNOMIAL
    };
    // Initialization
    void setupADC(SPIClass *spiInterface, int numChannels, MCP356xOversamplingRatio osr);
    void configureChannels(int numChannels);

    // Calibration Functions
    void setConversionMode(MCP356xChannel channel, ScaleConversionMode mode);
    void setScaleFactor(MCP356xChannel channel, float scale);
    void setLinearCalibration(MCP356xChannel channel, float slope, float intercept);
    void setPolynomialCalibration(MCP356xChannel channel, float a, float b, float c);
    void tare(MCP356xChannel channel);

    // Reading Functions
    int32_t getDigitalValue(MCP356xChannel channel);
    int32_t getCalibratedValue(MCP356xChannel channel);
    float   getGramsForce(MCP356xChannel channel);
    float   getForce(MCP356xChannel channel);
    int32_t getAverageValue(MCP356xChannel channel, int samples);
    void    getMultipleChannelAverage(int times, MCP356xChannel channels[], int32_t *averages, size_t num_channels);

    // Force Conversion Methods
    float convertToSingleValueForce(MCP356xChannel channel);
    float convertToLinearForce(MCP356xChannel channel);
    float convertToPolynomialForce(MCP356xChannel channel);

    // Configuration and Setup
    void setReadingOffset(MCP356xChannel channel, int32_t offset);

    // Calibration and reading methods
    int32_t getReadingOffset(MCP356xChannel channel);
    float   getScaleFactor(MCP356xChannel channel);

  private:
    static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; // m/s^2

    static const int    NUM_CHANNELS                  = 4;
    ScaleConversionMode _conversionMode[NUM_CHANNELS] = {
        ScaleConversionMode::UNDEFINED,
        ScaleConversionMode::UNDEFINED,
        ScaleConversionMode::UNDEFINED,
        ScaleConversionMode::UNDEFINED};

    // Channel-specific settings
    float _scale[NUM_CHANNELS]  = {1.0f, 1.0f, 1.0f, 1.0f};
    int   _offset[NUM_CHANNELS] = {0, 0, 0, 0};
    float _linearSlope[NUM_CHANNELS];
    float _linearIntercept[NUM_CHANNELS];
    float _polyA[NUM_CHANNELS];
    float _polyB[NUM_CHANNELS];
    float _polyC[NUM_CHANNELS];

    // Utility Methods
    int         channelToIndex(MCP356xChannel channel);
    const char *channelToString(MCP356xChannel channel) const;
};

#endif // MCP356X_SCALE_H