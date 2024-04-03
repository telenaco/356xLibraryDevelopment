#ifndef MCP356X_SCALE_H
#define MCP356X_SCALE_H

#include "MCP356x.h"

class MCP356xScale {
public:
  // Enumerations
  enum class ConversionMode : uint8_t {
    UNDEFINED,
    DIGITAL, 
    SINGLE_VALUE,
    LINEAR,
    POLYNOMIAL
  };

  // Constructor and destructor
  MCP356xScale(int totalScales, int sckPin, int sdoPin, int sdiPin,
    int irqPin1, int csPin1, int irqPin2 = -1, int csPin2 = -1, int irqPin3 = -1, int csPin3 = -1);
  ~MCP356xScale();

  // Configuration and setup
  void   setupChannelMappings();
  static MCP356xConfig defaultConfig;

  // ADC operations
  int  updatedAdcReadings();

  void printADCsDebug();
  void printAdcRawChannels();
  void printReadsPerSecond();
  void printChannelParameters();


  // Calibration functions
  void setDigitalRead(int scaleIndex);
  void setScaleFactor(int scaleIndex, float scale);
  void setLinearCalibration(int scaleIndex, float slope, float intercept);
  void setPolynomialCalibration(int scaleIndex, float a, float b, float c);
  void tare(int scaleIndex);

  // Reading functions
  void     getAverageValues(int times);
  float    getReading(int scaleIndex);
  float    getForce(int scaleIndex);
  float    getAverageValue(int scaleIndex);
  int32_t  getRawValue(int scaleIndex);
  uint32_t getReadCount(int adcIndex);
  uint16_t getReadsPerSecond(int adcIndex);

  // Force conversion methods
  int32_t convertToDigitalRead(int scaleIndex);
  float convertToSingleValueForce(int scaleIndex);
  float convertToLinearForce(int scaleIndex);
  float convertToPolynomialForce(int scaleIndex);

private:
  // Constants
  static const int MAX_SCALES = 12;
  static const int MAX_ADCS = 3;
  static const int CHANNELS_PER_ADC = 4;

  // ADC and scale configuration
  uint8_t _newDataFlags = 0;

    int      _totalScales;
    int      _sckPin;
    int      _sdoPin;
    int      _sdiPin;
    int      _irqPins[MAX_ADCS];
    int      _csPins[MAX_ADCS];
    MCP356x* _adcs[MAX_ADCS];
    uint16_t _readsPerSecond[MAX_ADCS] = { 0 };
    uint32_t _readAccumulator[MAX_ADCS] = { 0 };
    uint32_t _microsLastWindow[MAX_ADCS] = { 0 };

  // Scale calibration parameters and conversion modes
    struct ScaleConfig {
      ConversionMode mode;
      float          param1;
      float          param2;
      float          param3;
      int32_t        offset;
      int32_t        rawReading;
      int            adcIndex;
      MCP356xChannel channel;
    };

  ScaleConfig _scaleConfigs[MAX_SCALES];

  static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; // m/s^2
};

#endif // MCP356X_SCALE_H


//   

//   int32_t getCalibratedValue(int scaleIndex, int32_t offset);
//   float   getScaleFactor();

