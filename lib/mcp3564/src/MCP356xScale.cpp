/**
 * @file MCP356xScale.cpp
 * @brief Implementation of the MCP356xScale class for load cell management with MCP356x ADCs.
 *
 * This file contains the implementation of the MCP356xScale class, which provides
 * a higher level of abstraction for interfacing with load cells connected to MCP356x ADCs.
 *
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#include "MCP356xScale.h"

// Initialize the default configuration
MCP356xConfig MCP356xScale::defaultConfig = {
    .irq_pin = 0,
    .cs_pin = 0,
    .mclk_pin = 0,
    .addr = 0x01,
    .spiInterface = nullptr,
    .numChannels = 1,
    .osr = MCP356xOversamplingRatio::OSR_32,
    .gain = MCP356xGain::GAIN_1,
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE};

/**
 * @brief Constructor for the MCP356xScale class.
 *
 * Initializes the MCP356xScale with the specified number of scales and pin configurations.
 * Sets up SPI communication, creates ADC instances, and configures channel mappings.
 *
 * @param totalScales Total number of scales (load cells) to manage
 * @param sckPin SPI clock pin
 * @param sdoPin SPI data output pin
 * @param sdiPin SPI data input pin
 * @param irqPin1 Interrupt pin for first ADC
 * @param csPin1 Chip select pin for first ADC
 * @param irqPin2 Interrupt pin for second ADC (optional)
 * @param csPin2 Chip select pin for second ADC (optional)
 * @param irqPin3 Interrupt pin for third ADC (optional)
 * @param csPin3 Chip select pin for third ADC (optional)
 */
MCP356xScale::MCP356xScale(int totalScales, int sckPin, int sdoPin, int sdiPin,
                           int irqPin1, int csPin1, int irqPin2, int csPin2,
                           int irqPin3, int csPin3)
    : _totalScales(totalScales), _sckPin(sckPin), _sdoPin(sdoPin), _sdiPin(sdiPin)
{
    // Initialize SPI
    SPI.setSCK(sckPin);
    SPI.setMISO(sdoPin);
    SPI.setMOSI(sdiPin);
    SPI.begin();

    defaultConfig.spiInterface = &SPI;

    // Calculate the number of ADCs needed based on the total scales
    int adcCount = (_totalScales + CHANNELS_PER_ADC - 1) / CHANNELS_PER_ADC;

    // Initialize all ADCs to nullptr
    for (int i = 0; i < MAX_ADCS; ++i)
    {
        _adcs[i] = nullptr;
    }

    // Initialize the necessary ADCs
    for (int i = 0; i < adcCount; ++i)
    {
        int irqPin = (i == 0) ? irqPin1 : (i == 1) ? irqPin2
                                                   : irqPin3;
        int csPin = (i == 0) ? csPin1 : (i == 1) ? csPin2
                                                 : csPin3;

        defaultConfig.irq_pin = irqPin;
        defaultConfig.cs_pin = csPin;

        // If it's the last ADC and the total scales is not divisible by CHANNELS_PER_ADC,
        // use the remainder as the number of channels, otherwise use CHANNELS_PER_ADC
        defaultConfig.numChannels = (i == adcCount - 1) ? ((_totalScales % CHANNELS_PER_ADC) ? (_totalScales % CHANNELS_PER_ADC) : CHANNELS_PER_ADC) : CHANNELS_PER_ADC;

        _adcs[i] = new MCP356x(defaultConfig);
        _adcs[i]->setSpecificChannels(defaultConfig.numChannels);
    }

    // Initialize channel mappings for all scales
    setupChannelMappings();
}

/**
 * @brief Destructor for the MCP356xScale class.
 *
 * Cleans up resources by deleting all ADC instances.
 */
MCP356xScale::~MCP356xScale()
{
    for (int i = 0; i < MAX_ADCS; i++)
    {
        if (_adcs[i] != nullptr)
        {
            delete _adcs[i];
            _adcs[i] = nullptr;
        }
    }
}

/**
 * @brief Sets up channel mappings for the scales.
 *
 * Maps each scale to a specific ADC and channel based on the total number of scales.
 * Each scale is assigned to a differential channel on one of the ADCs.
 */
void MCP356xScale::setupChannelMappings()
{
    int scaleIndex = 0;
    MCP356xChannel diffChannels[] = {
        MCP356xChannel::DIFF_A,
        MCP356xChannel::DIFF_B,
        MCP356xChannel::DIFF_C,
        MCP356xChannel::DIFF_D};
    int diffChannelsCount = sizeof(diffChannels) / sizeof(diffChannels[0]);

    for (int adcIndex = 0; adcIndex < MAX_ADCS; ++adcIndex)
    {
        for (int i = 0; i < diffChannelsCount; ++i)
        {
            if (scaleIndex < _totalScales)
            {
                _scaleConfigs[scaleIndex].adcIndex = adcIndex;
                _scaleConfigs[scaleIndex].channel = diffChannels[i];

                // Initialize default values for ScaleConfig
                _scaleConfigs[scaleIndex].mode = ConversionMode::DIGITAL;
                _scaleConfigs[scaleIndex].param1 = 1.0f;
                _scaleConfigs[scaleIndex].param2 = 0.0f;
                _scaleConfigs[scaleIndex].param3 = 0.0f;
                _scaleConfigs[scaleIndex].offset = 0.0f;

                scaleIndex++;
            }
            else
            {
                break;
            }
        }
    }
}

/**
 * @brief Updates ADC readings from all connected ADCs.
 *
 * Checks each ADC for new data and updates the _newDataFlags accordingly.
 * Also tracks the readings per second for each ADC.
 *
 * @return int Returns 1 if all ADCs have updated data, 0 otherwise.
 */
int MCP356xScale::updatedAdcReadings()
{
    uint32_t microsNow = micros();
    uint8_t initializedAdcsMask = 0;

    for (int i = 0; i < MAX_ADCS; ++i)
    {
        if (_adcs[i] != nullptr)
        {
            initializedAdcsMask |= (1 << i);
            if (_adcs[i]->updatedReadings())
            {
                _newDataFlags |= (1 << i);
                _readAccumulator[i]++;

                if (microsNow - _microsLastWindow[i] >= 1000000)
                {
                    _microsLastWindow[i] = microsNow;
                    _readsPerSecond[i] = _readAccumulator[i];
                    _readAccumulator[i] = 0;
                }
            }
        }
    }

    // Check if all initialized ADCs have new data
    if ((_newDataFlags & initializedAdcsMask) == initializedAdcsMask)
    {
        _newDataFlags = 0;
        return 1;
    }

    return 0;
}

/**
 * @brief Sets the scale to use digital reading mode.
 *
 * Configures the specified scale to use raw digital readings without calibration.
 *
 * @param scaleIndex Index of the scale to configure
 */
void MCP356xScale::setDigitalRead(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return;

    _scaleConfigs[scaleIndex].mode = ConversionMode::DIGITAL;
}

/**
 * @brief Sets a simple scale factor for the specified scale.
 *
 * Configures the scale to use a single scale factor for conversion.
 *
 * @param scaleIndex Index of the scale to configure
 * @param scale Scale factor to apply to raw readings
 */
void MCP356xScale::setScaleFactor(int scaleIndex, float scale)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return;

    _scaleConfigs[scaleIndex].mode = ConversionMode::SINGLE_VALUE;
    _scaleConfigs[scaleIndex].param1 = scale;
}

/**
 * @brief Sets linear calibration parameters for the specified scale.
 *
 * Configures the scale to use a linear equation (y = ax + b) for conversion.
 *
 * @param scaleIndex Index of the scale to configure
 * @param slope Slope (a) of the linear equation
 * @param intercept Y-intercept (b) of the linear equation
 */
void MCP356xScale::setLinearCalibration(int scaleIndex, float slope, float intercept)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return;

    _scaleConfigs[scaleIndex].mode = ConversionMode::LINEAR;
    _scaleConfigs[scaleIndex].param1 = slope;
    _scaleConfigs[scaleIndex].param2 = intercept;
}

/**
 * @brief Sets polynomial calibration parameters for the specified scale.
 *
 * Configures the scale to use a polynomial equation (y = ax² + bx + c) for conversion.
 *
 * @param scaleIndex Index of the scale to configure
 * @param a Coefficient of x²
 * @param b Coefficient of x
 * @param c Constant term
 */
void MCP356xScale::setPolynomialCalibration(int scaleIndex, float a, float b, float c)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return;

    _scaleConfigs[scaleIndex].mode = ConversionMode::POLYNOMIAL;
    _scaleConfigs[scaleIndex].param1 = a;
    _scaleConfigs[scaleIndex].param2 = b;
    _scaleConfigs[scaleIndex].param3 = c;
}

/**
 * @brief Converts a raw reading to a digital reading.
 *
 * Retrieves the raw reading from the ADC and applies the tare offset.
 *
 * @param scaleIndex Index of the scale
 * @return int32_t Digital reading with tare offset applied
 */
int32_t MCP356xScale::convertToDigitalRead(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return -2;

    ScaleConfig &config = _scaleConfigs[scaleIndex];
    MCP356x *adc = _adcs[config.adcIndex];

    if (adc == nullptr)
        return -2;

    config.rawReading = adc->value(config.channel);
    return config.rawReading - config.offset;
}

/**
 * @brief Converts a raw reading using a single scale factor.
 *
 * Retrieves the raw reading from the ADC, applies the tare offset,
 * and multiplies by the scale factor.
 *
 * @param scaleIndex Index of the scale
 * @return float Scaled reading
 */
float MCP356xScale::convertToSingleValueForce(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return -2.0f;

    ScaleConfig &config = _scaleConfigs[scaleIndex];
    MCP356x *adc = _adcs[config.adcIndex];

    if (adc == nullptr)
        return -2.0f;

    config.rawReading = adc->value(config.channel);
    int32_t reading = config.rawReading - config.offset;
    return reading * config.param1;
}

/**
 * @brief Converts a raw reading using a linear equation.
 *
 * Retrieves the raw reading from the ADC and applies a linear equation (y = ax + b).
 *
 * @param scaleIndex Index of the scale
 * @return float Calibrated reading
 */
float MCP356xScale::convertToLinearForce(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return -2.0f;

    ScaleConfig &config = _scaleConfigs[scaleIndex];
    MCP356x *adc = _adcs[config.adcIndex];

    if (adc == nullptr)
        return -2.0f;

    config.rawReading = adc->value(config.channel);
    return config.param1 * config.rawReading + config.param2;
}

/**
 * @brief Converts a raw reading using a polynomial equation.
 *
 * Retrieves the raw reading from the ADC and applies a polynomial equation (y = ax² + bx + c).
 *
 * @param scaleIndex Index of the scale
 * @return float Calibrated reading
 */
float MCP356xScale::convertToPolynomialForce(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return -2.0f;

    ScaleConfig &config = _scaleConfigs[scaleIndex];
    MCP356x *adc = _adcs[config.adcIndex];

    if (adc == nullptr)
        return -2.0f;

    config.rawReading = adc->value(config.channel);
    return config.param1 * config.rawReading * config.rawReading +
           config.param2 * config.rawReading + config.param3;
}

/**
 * @brief Performs a tare operation on the specified scale.
 *
 * Takes multiple readings and calculates the average to use as the zero offset.
 * If scaleIndex is -1, it tares all scales.
 *
 * @param scaleIndex Index of the scale to tare, or -1 to tare all scales
 * @param times Number of readings to average for the tare operation
 */
void MCP356xScale::tare(int scaleIndex, int times)
{
    // Create arrays to store the sum and count of readings for each scale
    int32_t sums[MAX_SCALES] = {0};
    int readingCounts[MAX_SCALES] = {0};

    // Choose which scales to tare: just the one specified, or all
    int startScale = (scaleIndex >= 0 && scaleIndex < _totalScales) ? scaleIndex : 0;
    int endScale = (scaleIndex >= 0 && scaleIndex < _totalScales) ? scaleIndex + 1 : _totalScales;

    // Take 'times' readings
    for (int i = 0; i < times;)
    {
        bool allUpdated = true;

        // For each scale to be tared
        for (int j = startScale; j < endScale; j++)
        {
            ScaleConfig &config = _scaleConfigs[j];
            MCP356x *adc = _adcs[config.adcIndex];

            if (adc != nullptr)
            {
                if (adc->updatedReadings())
                {
                    int32_t currentValue = adc->value(config.channel);

                    // Ensure no integer overflow when summing
                    if ((INT32_MAX - sums[j]) >= currentValue)
                    {
                        sums[j] += currentValue;
                        readingCounts[j]++;
                    }
                }
            }
            else
            {
                allUpdated = false;
            }
        }

        // Only increment the reading counter if we got readings from all scales
        if (allUpdated)
        {
            i++;
        }
    }

    // Calculate averages and apply tare offsets based on conversion mode
    for (int j = startScale; j < endScale; j++)
    {
        ScaleConfig &config = _scaleConfigs[j];
        if (readingCounts[j] > 0)
        {
            int32_t avgReading = static_cast<float>(sums[j]) / readingCounts[j];

            switch (config.mode)
            {
            case ConversionMode::DIGITAL:
            case ConversionMode::SINGLE_VALUE:
                config.offset = avgReading;
                break;
            case ConversionMode::LINEAR:
                config.param2 = -config.param1 * avgReading;
                break;
            case ConversionMode::POLYNOMIAL:
                config.param3 = -config.param1 * avgReading * avgReading -
                                config.param2 * avgReading;
                break;
            default:
                break;
            }
        }
    }
}

/**
 * @brief Gets the raw uncalibrated value from the specified scale.
 *
 * Returns the last raw ADC reading for the specified scale.
 *
 * @param scaleIndex Index of the scale to read
 * @return int32_t Raw ADC reading
 */
int32_t MCP356xScale::getRawValue(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return -2;

    ScaleConfig config = _scaleConfigs[scaleIndex];

    if (_adcs[config.adcIndex] == nullptr)
        return -1;

    return config.rawReading;
}

/**
 * @brief Gets the calibrated reading from the specified scale.
 *
 * Returns the reading after applying the configured calibration method.
 *
 * @param scaleIndex Index of the scale to read
 * @return float Calibrated reading in the configured units
 */
float MCP356xScale::getReading(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return -2.0f;

    ScaleConfig &config = _scaleConfigs[scaleIndex];

    switch (config.mode)
    {
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
        MCP356x *adc = _adcs[config.adcIndex];
        if (adc == nullptr)
        {
            return -2.0f;
        }
        return static_cast<float>(adc->value(config.channel));
    }
    default:
        return -2.0f;
    }
}

/**
 * @brief Gets the force reading from the specified scale in Newtons.
 *
 * Converts the calibrated reading to Newtons using gravitational acceleration.
 * Force (N) = mass (g) * 9.81 / 1000
 *
 * @param scaleIndex Index of the scale to read
 * @return float Force reading in Newtons
 */
float MCP356xScale::getForce(int scaleIndex)
{
    if (scaleIndex < 0 || scaleIndex >= _totalScales)
        return -2.0f;

    return getReading(scaleIndex) * GRAVITATIONAL_ACCELERATION / 1000.0f;
}

/**
 * @brief Gets the readings per second for the specified ADC.
 *
 * Returns the number of readings per second calculated for the ADC.
 *
 * @param adcIndex Index of the ADC
 * @return uint16_t Number of readings per second
 */
uint16_t MCP356xScale::getReadsPerSecond(int adcIndex)
{
    if (adcIndex >= 0 && adcIndex < MAX_ADCS)
        return _readsPerSecond[adcIndex];
    return 0;
}

/**
 * @brief Gets the read count for the specified ADC.
 *
 * Returns the total number of readings taken by the specified ADC.
 *
 * @param adcIndex Index of the ADC
 * @return uint32_t Number of readings taken
 */
uint32_t MCP356xScale::getReadCount(int adcIndex)
{
    if (adcIndex >= 0 && adcIndex < MAX_ADCS && _adcs[adcIndex] != nullptr)
        return _adcs[adcIndex]->readCount();
    return 0;
}

/**
 * @brief Prints raw ADC channel values.
 *
 * Outputs detailed information about each scale's ADC values and voltage readings.
 */
void MCP356xScale::printAdcRawChannels()
{
    StringBuilder output;

    for (int scaleIndex = 0; scaleIndex < _totalScales; ++scaleIndex)
    {
        ScaleConfig config = _scaleConfigs[scaleIndex];
        if (_adcs[config.adcIndex] != nullptr)
        {
            double adcValue = _adcs[config.adcIndex]->value(config.channel);
            double voltage = _adcs[config.adcIndex]->valueAsVoltage(config.channel);

            output.concatf("Load Cell %d - ADC Value: %.0f, Voltage: %.6fV\n",
                           scaleIndex + 1, adcValue, voltage);
        }
    }

    if (output.length() > 0)
    {
        Serial.print((char *)output.string());
    }
}

/**
 * @brief Prints debug information about all ADCs.
 *
 * Outputs detailed debug information about all initialized ADCs.
 */
void MCP356xScale::printADCsDebug()
{
    StringBuilder output;

    output.concatf("\n--- MCP356xScale Debug Information ---\n");
    output.concatf("Total Scales: %d\n", _totalScales);
    output.concatf("SCK Pin: %d\n", _sckPin);
    output.concatf("SDO Pin: %d\n", _sdoPin);
    output.concatf("SDI Pin: %d\n", _sdiPin);

    for (int i = 0; i < MAX_ADCS; ++i)
    {
        if (_adcs[i] != nullptr)
        {
            output.concatf("\n--- ADC %d Debug Information ---\n", i + 1);
            _adcs[i]->printRegs(&output);
            _adcs[i]->printPins(&output);
            _adcs[i]->printTimings(&output);
            _adcs[i]->printData(&output);
        }
        else
        {
            output.concatf("\n--- ADC %d is not initialized ---\n", i + 1);
        }
    }
    output.concatf("\n");
    Serial.print((char *)output.string());
}

/**
 * @brief Prints the update rate for each ADC in readings per second.
 *
 * Outputs the number of readings per second for each initialized ADC.
 */
void MCP356xScale::printReadsPerSecond()
{
    StringBuilder output;
    int adcCount = 0;
    for (int i = 0; i < MAX_ADCS; ++i)
    {
        if (_adcs[i] != nullptr)
        {
            uint16_t readsPerSecond = getReadsPerSecond(i);
            output.concatf("ADC %d: %u readings per second\n", i + 1, readsPerSecond);
            adcCount++;
        }
    }
    if (adcCount == 0)
    {
        output.concat("No ADCs initialized.\n");
    }
    output.concatf("\n");
    Serial.print((char *)output.string());
}

/**
 * @brief Prints the calibration parameters for each channel.
 *
 * Outputs detailed calibration parameters for all scales.
 */
void MCP356xScale::printChannelParameters()
{
    StringBuilder output;

    output.concatf("\n--- Channel Parameters ---\n");

    for (int scaleIndex = 0; scaleIndex < _totalScales; ++scaleIndex)
    {
        ScaleConfig &config = _scaleConfigs[scaleIndex];

        output.concatf("Channel %d:\n", scaleIndex + 1);
        output.concatf("  Conversion Mode: ");

        switch (config.mode)
        {
        case ConversionMode::UNDEFINED:
            output.concatf("UNDEFINED\n");
            break;
        case ConversionMode::DIGITAL:
            output.concatf("DIGITAL_READ\n");
            break;
        case ConversionMode::SINGLE_VALUE:
            output.concatf("SINGLE_VALUE\n");
            output.concatf("  Scale Factor: %.6f\n", config.param1);
            break;
        case ConversionMode::LINEAR:
            output.concatf("LINEAR\n");
            output.concatf("  Slope: %.6f\n", config.param1);
            output.concatf("  Intercept: %.6f\n", config.param2);
            break;
        case ConversionMode::POLYNOMIAL:
            output.concatf("POLYNOMIAL\n");
            output.concatf("  A: %.6e\n", config.param1);
            output.concatf("  B: %.6f\n", config.param2);
            output.concatf("  C: %.6f\n", config.param3);
            break;
        }

        output.concatf("  Offset: %ld\n", config.offset);
        output.concatf("  Raw Reading: %ld\n", config.rawReading);
        output.concatf("  ADC Index: %d\n", config.adcIndex);
        output.concatf("  Channel: ");

        switch (config.channel)
        {
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

    if (output.length() > 0)
    {
        Serial.print((char *)output.string());
    }
}