/**
 * @file MCP356xScale.h
 * @brief MCP356xScale class for managing multiple load cells connected to MCP356x ADCs.
 *
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#ifndef MCP356X_SCALE_H
#define MCP356X_SCALE_H

#include "MCP356x.h"

class MCP356xScale
{
public:
    /**
     * @enum ConversionMode
     * @brief Defines the different conversion modes for interpreting ADC readings.
     */
    enum class ConversionMode : uint8_t
    {
        UNDEFINED,    // Undefined conversion mode
        DIGITAL,      // Raw digital reading
        SINGLE_VALUE, // Single value scale factor conversion
        LINEAR,       // Linear equation conversion (ax + b)
        POLYNOMIAL    // Polynomial equation conversion (ax² + bx + c)
    };

    MCP356xScale(int totalScales, int sckPin, int sdoPin, int sdiPin,
                 int irqPin1, int csPin1, int irqPin2 = -1, int csPin2 = -1,
                 int irqPin3 = -1, int csPin3 = -1);
    ~MCP356xScale();

    void setupChannelMappings();
    int updatedAdcReadings();
    void printADCsDebug();
    void printAdcRawChannels();
    void printReadsPerSecond();
    void printChannelParameters();

    void setDigitalRead(int scaleIndex);
    void setScaleFactor(int scaleIndex, float scale);
    void setLinearCalibration(int scaleIndex, float slope, float intercept);
    void setPolynomialCalibration(int scaleIndex, float a, float b, float c);
    void tare(int scaleIndex, int times = 1000);
    float getReading(int scaleIndex);
    float getForce(int scaleIndex);
    int32_t getRawValue(int scaleIndex);
    uint32_t getReadCount(int adcIndex);
    uint16_t getReadsPerSecond(int adcIndex);

    static MCP356xConfig defaultConfig;

private:
    static const int MAX_SCALES = 12;      // Maximum number of scales supported
    static const int MAX_ADCS = 3;         // Maximum number of ADCs supported
    static const int CHANNELS_PER_ADC = 4; // Number of channels per ADC

    uint8_t _newDataFlags = 0;                  // Flags indicating new data availability
    int _totalScales;                           // Total number of scales being managed
    int _sckPin;                                // SPI clock pin
    int _sdoPin;                                // SPI data output pin
    int _sdiPin;                                // SPI data input pin
    MCP356x *_adcs[MAX_ADCS];                   // Array of MCP356x ADC instances
    uint16_t _readsPerSecond[MAX_ADCS] = {0};   // Readings per second for each ADC
    uint32_t _readAccumulator[MAX_ADCS] = {0};  // Read counter for rate calculation
    uint32_t _microsLastWindow[MAX_ADCS] = {0}; // Timestamp for rate calculation

    /**
     * @struct ScaleConfig
     * @brief Configuration structure for each scale.
     */
    struct ScaleConfig
    {
        ConversionMode mode;    // Conversion mode for this scale
        float param1;           // First calibration parameter (depends on mode)
        float param2;           // Second calibration parameter (depends on mode)
        float param3;           // Third calibration parameter (depends on mode)
        int32_t offset;         // Tare offset
        int32_t rawReading;     // Last raw reading
        int adcIndex;           // Index of the ADC this scale is connected to
        MCP356xChannel channel; // Channel on the ADC this scale is connected to
    };

    ScaleConfig _scaleConfigs[MAX_SCALES];

    int32_t convertToDigitalRead(int scaleIndex);
    float convertToSingleValueForce(int scaleIndex);
    float convertToLinearForce(int scaleIndex);
    float convertToPolynomialForce(int scaleIndex);

    static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f;
};

#endif // MCP356X_SCALE_H