#include "MCP356xScale.h"

MCP356xConfig MCP356xScale::defaultConfig = {
  .irq_pin      = 0,
  .cs_pin       = 0,
  .mclk_pin     = 0,
  .addr         = 0x01,
  .spiInterface = nullptr,
  .numChannels  = 1,
  .osr          = MCP356xOversamplingRatio::OSR_32,
  .gain         = MCP356xGain::GAIN_1,
  .mode         = MCP356xADCMode::ADC_CONVERSION_MODE
};

MCP356xScale::MCP356xScale(int totalScales, int sckPin, int sdoPin, int sdiPin,
  int irqPin1, int csPin1, int irqPin2, int csPin2, int irqPin3, int csPin3)
  : _totalScales(totalScales), _sckPin(sckPin), _sdoPin(sdoPin), _sdiPin(sdiPin) {

  SPI.setSCK(sckPin);
  SPI.setMISO(sdoPin);
  SPI.setMOSI(sdiPin);
  SPI.begin();

  defaultConfig.spiInterface = &SPI;

  // Calculate the number of ADCs needed based on the total scales
  int adcCount = (_totalScales + CHANNELS_PER_ADC - 1) / CHANNELS_PER_ADC;

  // Initialize all ADCs to nullptr
  for (int i = 0; i < MAX_ADCS; ++i) {
    _adcs[i] = nullptr;
  }

  // Initialize the necessary ADCs
  for (int i = 0; i < adcCount; ++i) {

    int irqPin = (i == 0) ? irqPin1 : (i == 1) ? irqPin2 : irqPin3;
    int csPin  = (i == 0) ? csPin1 : (i == 1) ? csPin2 : csPin3;

    defaultConfig.irq_pin = irqPin;
    defaultConfig.cs_pin = csPin;

    
    // If it's the last ADC and the total scales is divisible by CHANNELS_PER_ADC, use CHANNELS_PER_ADC, otherwise calculate remainder
    defaultConfig.numChannels = (i == adcCount - 1) ? 
                                ((_totalScales % CHANNELS_PER_ADC) ? (_totalScales % CHANNELS_PER_ADC) : CHANNELS_PER_ADC) :
                                CHANNELS_PER_ADC;


    _adcs[i] = new MCP356x(defaultConfig);
    _adcs[i]->setSpecificChannels(defaultConfig.numChannels);

  }
  // initialize all the variables for the cells
  setupChannelMappings();
  // tare the sytem
  // tare(100000);
}

MCP356xScale::~MCP356xScale() {
  for (int i = 0; i < MAX_ADCS; i++) {
    if (_adcs[i] != nullptr) {
      delete _adcs[i];
      _adcs[i] = nullptr;
    }
  }
}

void MCP356xScale::setupChannelMappings() {
  int scaleIndex = 0;
  MCP356xChannel diffChannels[] = { MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D };
  int diffChannelsCount = sizeof(diffChannels) / sizeof(diffChannels[0]);

  for (int adcIndex = 0; adcIndex < MAX_ADCS; ++adcIndex) {
    for (int i = 0; i < diffChannelsCount; ++i) {
      if (scaleIndex < _totalScales) {
        _scaleConfigs[scaleIndex].adcIndex = adcIndex;
        _scaleConfigs[scaleIndex].channel = diffChannels[i];
        
        // Initialize default values for ScaleConfig to be used on 3axis library
        _scaleConfigs[scaleIndex].mode = ConversionMode::DIGITAL;
        _scaleConfigs[scaleIndex].param1 = 1.0f;
        _scaleConfigs[scaleIndex].param2 = 0.0f;
        _scaleConfigs[scaleIndex].param3 = 0.0f;
        _scaleConfigs[scaleIndex].offset = 0.0f;

        scaleIndex++;
      }
      else {
        break;
      }
    }
  }
}

int MCP356xScale::updatedAdcReadings() {

  uint32_t microsNow = micros();
  uint8_t initializedAdcsMask = 0;

  for (int i = 0; i < MAX_ADCS; ++i) {
    if (_adcs[i] != nullptr) {
      initializedAdcsMask |= (1 << i);
      if (_adcs[i]->updatedReadings()) {
        _newDataFlags |= (1 << i);
        _readAccumulator[i]++;

        if (microsNow - _microsLastWindow[i] >= 1000000) {
          _microsLastWindow[i] = microsNow;
          _readsPerSecond[i] = _readAccumulator[i];
          _readAccumulator[i] = 0;
                
        }
      }
    }
  }

  if ((_newDataFlags & initializedAdcsMask) == initializedAdcsMask) {
    _newDataFlags = 0;
    return 1;
  }

  return 0;
}

void MCP356xScale::setDigitalRead(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales) return;
  
  _scaleConfigs[scaleIndex].mode = ConversionMode::DIGITAL;
}

void MCP356xScale::setScaleFactor(int scaleIndex, float scale) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)  return;
  
  _scaleConfigs[scaleIndex].mode = ConversionMode::SINGLE_VALUE;
    _scaleConfigs[scaleIndex].param1 = scale;
}

void MCP356xScale::setLinearCalibration(int scaleIndex, float slope, float intercept) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)    return;
  
  _scaleConfigs[scaleIndex].mode = ConversionMode::LINEAR;
  _scaleConfigs[scaleIndex].param1 = slope;
  _scaleConfigs[scaleIndex].param2 = intercept;
}

void MCP356xScale::setPolynomialCalibration(int scaleIndex, float a, float b, float c) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)   return;
  
  _scaleConfigs[scaleIndex].mode = ConversionMode::POLYNOMIAL;
  _scaleConfigs[scaleIndex].param1 = a;
  _scaleConfigs[scaleIndex].param2 = b;
  _scaleConfigs[scaleIndex].param3 = c;
}

int32_t MCP356xScale::convertToDigitalRead(int scaleIndex){
  if (scaleIndex < 0 || scaleIndex >= _totalScales) return -2;

  ScaleConfig& config = _scaleConfigs[scaleIndex];
  MCP356x*     adc    = _adcs[config.adcIndex];

  if (adc == nullptr) return -2;

  config.rawReading = adc->value(config.channel);
  return config.rawReading - config.offset;
}

float MCP356xScale::convertToSingleValueForce(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)     return -2.0f;

  ScaleConfig& config = _scaleConfigs[scaleIndex];
  MCP356x* adc = _adcs[config.adcIndex];
  
  if (adc == nullptr)     return -2.0f;

  config.rawReading = adc->value(config.channel);
  int32_t reading = config.rawReading - config.offset;
  return reading * config.param1;
}

float MCP356xScale::convertToLinearForce(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)     return -2.0f;

  ScaleConfig& config = _scaleConfigs[scaleIndex];
  MCP356x*     adc    = _adcs[config.adcIndex];

  if (adc == nullptr)     return -2.0f;

  config.rawReading = adc->value(config.channel);
  return config.param1 * config.rawReading + config.param2;
}

float MCP356xScale::convertToPolynomialForce(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)     return -2.0f;

  ScaleConfig& config = _scaleConfigs[scaleIndex];
  MCP356x*     adc    = _adcs[config.adcIndex];

  if (adc == nullptr)     return -2.0f;

  config.rawReading = adc->value(config.channel);
  return config.param1 * config.rawReading * config.rawReading + config.param2 * config.rawReading + config.param3;
}

void MCP356xScale::tare(int times) {
  int32_t sums[MAX_SCALES] = { 0 };
  int readingCounts[MAX_SCALES] = { 0 };

  for (int i = 0; i < times;) {
    bool allUpdated = true;

    for (int j = 0; j < _totalScales; j++) {
      ScaleConfig& config = _scaleConfigs[j];
      MCP356x* adc = _adcs[config.adcIndex];

      if (adc != nullptr) {
        if (adc->updatedReadings()) {
          int32_t currentValue = adc->value(config.channel);

          if ((INT32_MAX - sums[j]) >= currentValue) {
            sums[j] += currentValue;
            readingCounts[j]++;
          }
        }
      }
      else {
        allUpdated = false;
      }
    }

    if (allUpdated) {
      i++;
    }
  }

  for (int j = 0; j < _totalScales; j++) {
    ScaleConfig& config = _scaleConfigs[j];
    if (readingCounts[j] > 0) {
      int32_t avgReading = static_cast<float>(sums[j]) / readingCounts[j];

      switch (config.mode) {
        case ConversionMode::DIGITAL:
          config.offset = avgReading;
          break;
        case ConversionMode::SINGLE_VALUE:
          config.offset = avgReading;
          break;
        case ConversionMode::LINEAR:
          config.param2 = -config.param1 * avgReading;
          break;
        case ConversionMode::POLYNOMIAL:
          config.param3 = -config.param1 * avgReading * avgReading - config.param2 * avgReading;
          break;
        default:
          break;
      }
    }
  }
}

int32_t MCP356xScale::getRawValue(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)     return -2;

  ScaleConfig config = _scaleConfigs[scaleIndex];

  if (_adcs[config.adcIndex] == nullptr)     return -1;

  return config.rawReading;;
}

float MCP356xScale::getReading(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)     return -2.0f;

  ScaleConfig& config = _scaleConfigs[scaleIndex];

  switch (config.mode) {
  case ConversionMode::DIGITAL:
    return convertToDigitalRead(scaleIndex);
  case ConversionMode::SINGLE_VALUE:
    return convertToSingleValueForce(scaleIndex);
  case ConversionMode::LINEAR:
    return convertToLinearForce(scaleIndex);
  case ConversionMode::POLYNOMIAL:
    return convertToPolynomialForce(scaleIndex);
  case ConversionMode::UNDEFINED:
  {
    MCP356x* adc = _adcs[config.adcIndex];
    if (adc == nullptr) {
      return -2.0f;
    }
    return static_cast<float>(adc->value(config.channel));
  }
  default:
    return -2.0f;
  }
}

float MCP356xScale::getForce(int scaleIndex) {
  if (scaleIndex < 0 || scaleIndex >= _totalScales)     return -2.0f;

  return getReading(scaleIndex) * GRAVITATIONAL_ACCELERATION / 1000.0f;
}
/*
void MCP356xScale::getAverageValues(int times) {
  int32_t sums[MAX_SCALES] = { 0 };
  int readingCounts[MAX_SCALES] = { 0 };

  for (int i = 0; i < times;) {
    bool allUpdated = true;

    for (int j = 0; j < _totalScales; j++) {
      ScaleConfig& config = _scaleConfigs[j];
      MCP356x* adc = _adcs[config.adcIndex];

      if (adc != nullptr) {
        if (adc->updatedReadings()) {
          int32_t currentValue = adc->value(config.channel);

          if ((INT32_MAX - sums[j]) >= currentValue) {
            sums[j] += currentValue;
            readingCounts[j]++;
          }
        }
      }
      else {
        allUpdated = false;
      }
    }

    if (allUpdated) {
      i++;
    }
  }

  for (int j = 0; j < _totalScales; j++) {
    if (readingCounts[j] > 0) {
      _scaleConfigs[j].offset = static_cast<float>(sums[j]) / readingCounts[j];
    }
  }
}

float MCP356xScale::getAverageValue(int scaleIndex) {
  if (scaleIndex >= 0 && scaleIndex < _totalScales) {
    return _scaleConfigs[scaleIndex].offset;
  }
  return 0.0f;
}
*/
uint16_t MCP356xScale::getReadsPerSecond(int adcIndex) {
    if (adcIndex >= 0 && adcIndex < MAX_ADCS)  return _readsPerSecond[adcIndex];
    return 0;
}

void MCP356xScale::printAdcRawChannels() {
  StringBuilder output;

    for (int scaleIndex = 0; scaleIndex < _totalScales; ++scaleIndex) {
        ScaleConfig config = _scaleConfigs[scaleIndex];
        if (_adcs[config.adcIndex] != nullptr) {
            double adcValue = _adcs[config.adcIndex]->value(config.channel);
            double voltage = _adcs[config.adcIndex]->valueAsVoltage(config.channel);

            output.concatf("Load Cell %d - ADC Value: %.0f, Voltage: %.6fV\n", scaleIndex + 1, adcValue, voltage);
        }
    }

    if (output.length() > 0) {
        Serial.print((char*)output.string());
    }
}

void MCP356xScale::printADCsDebug() {
    StringBuilder output;

    output.concatf("\n--- MCP356xScale Debug Information ---\n");
    output.concatf("Total Scales: %d\n", _totalScales);
    output.concatf("SCK Pin: %d\n", _sckPin);
    output.concatf("SDO Pin: %d\n", _sdoPin);
    output.concatf("SDI Pin: %d\n", _sdiPin);

    for (int i = 0; i < MAX_ADCS; ++i) {
      if (_adcs[i] != nullptr) {
          output.concatf("\n--- ADC %d Debug Information ---\n", i + 1);
            _adcs[i]->printRegs(&output);
            _adcs[i]->printPins(&output);
            _adcs[i]->printTimings(&output);
            _adcs[i]->printData(&output);
      }
      else {
            output.concatf("\n--- ADC %d is not initialized ---\n", i + 1);
        }
    }
    output.concatf("\n");
    Serial.print((char*)output.string());
}

void MCP356xScale::printReadsPerSecond() {
  StringBuilder output;
  int adcCount = 0;
  for (int i = 0; i < MAX_ADCS; ++i) {
    if (_adcs[i] != nullptr) {
      uint16_t readsPerSecond = getReadsPerSecond(i);
      output.concatf("ADC %d: %u readings per second\n", i + 1, readsPerSecond);
      adcCount++;
    }
  }
  if (adcCount == 0) {
    output.concat("No ADCs initialized.\n");
  }
  output.concatf("\n");
  Serial.print((char*)output.string());
}

void MCP356xScale::printChannelParameters() {
  StringBuilder output;

  output.concatf("\n--- Channel Parameters ---\n");

  for (int scaleIndex = 0; scaleIndex < _totalScales; ++scaleIndex) {
    ScaleConfig& config = _scaleConfigs[scaleIndex];

    output.concatf("Channel %d:\n", scaleIndex + 1);
    output.concatf("  Conversion Mode: ");

    switch (config.mode) {
      case ConversionMode::UNDEFINED:
        output.concatf("UNDEFINED\n");
        break;
      case ConversionMode::DIGITAL:
        output.concatf("DIGITAL_READ\n");
        break;
      case ConversionMode::SINGLE_VALUE:
        output.concatf("SINGLE_VALUE\n");
        output.concatf("  Scale Factor: %.2f\n", config.param1);
        break;
      case ConversionMode::LINEAR:
        output.concatf("LINEAR\n");
        output.concatf("  Slope: %.2f\n", config.param1);
        output.concatf("  Intercept: %.2f\n", config.param2);
        break;
      case ConversionMode::POLYNOMIAL:
        output.concatf("POLYNOMIAL\n");
        output.concatf("  A: %.2f\n", config.param1);
        output.concatf("  B: %.2f\n", config.param2);
        output.concatf("  C: %.2f\n", config.param3);
        break;
    }

    output.concatf("  Offset: %ld\n", config.offset);
    output.concatf("  Raw Reading: %ld\n", config.rawReading);
    output.concatf("  ADC Index: %d\n", config.adcIndex);
    output.concatf("  Channel: ");

    switch (config.channel) {
      case MCP356xChannel::DIFF_A:
        output.concatf("DIFF_A\n");
        break;
      case MCP356xChannel::DIFF_B:
        output.concatf("DIFF_B\n");
        break;
      case MCP356xChannel::DIFF_C:
        output.concatf("DIFF_C\n");
        break;
      case MCP356xChannel::DIFF_D:
        output.concatf("DIFF_D\n");
        break;
      default:
        output.concatf("UNKNOWN\n");
        break;
    }

    output.concatf("\n");
  }

  if (output.length() > 0) {
    Serial.print((char*)output.string());
  }
}