#include "_MCP356xScale.h"

MCP356xConfig MCP356xScale::defaultConfig = {
    .irq_pin = 0,  // no pins are asigned by default, must be configured. Default interrupt pin on the microcontroller
    .cs_pin = 0,   // no pins are asigned by default, must be configured. Default CS pin
    .mclk_pin = 0, // MCLK pin, 0 if indicates internal clock
    .addr = 0x01,  // Default device address
    .spiInterface = nullptr, // set in the constructor null pointer to reserve the memory
    .numChannels = 1, // Default uses a single ADC channel
    .osr = MCP356xOversamplingRatio::OSR_32, 
    .gain = MCP356xGain::GAIN_1, 
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE 
};

MCP356x* MCP356xScale::adcDevices[MCP356X_MAX_INSTANCES] = {nullptr};
bool MCP356xScale::isInitialized[MCP356X_MAX_INSTANCES] = { false };
MCP356xScale::ChannelMapping MCP356xScale::channelMappings[MCP356xScale::MAX_SCALES];


/** 
 * the sck and sdo pins are common for the 3 adc what changes and is unique are the interrupt pins and 
 * chip select pins. 
 * the construnctor will start with adc 1 and assign Scales based on the number of total scales. 
 * e.g. if totalScales == 6 it will create a scale on adc 1 channels diff a, b, c and d then, adc 2 channel a and b. 
 * 
*/

MCP356xScale::MCP356xScale(int totalScales, int SCK_PIN, int SDO_PIN, int SDI_PIN, int irq_0, int cs_0, int irq_1, int cs_1, int irq_2, int cs_2) {

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    totalChannels = totalScales; 

    defaultConfig.spiInterface = &SPI;

    adcCount = (totalChannels + MAX_CHANNELS_PER_ADC - 1) / MAX_CHANNELS_PER_ADC;

    for (int i = 0; i < adcCount; ++i) {
        int irq_pin = (i == 0) ? irq_0 : (i == 1) ? irq_1 : irq_2;
        int cs_pin = (i == 0) ? cs_0 : (i == 1) ? cs_1 : cs_2;


        MCP356xConfig config = defaultConfig;
        config.irq_pin = irq_pin;
        config.cs_pin = cs_pin;
        config.numChannels = (i < adcCount - 1) ? MAX_CHANNELS_PER_ADC : 
        (totalChannels % MAX_CHANNELS_PER_ADC == 0 ? MAX_CHANNELS_PER_ADC : totalChannels % MAX_CHANNELS_PER_ADC);
        
        adcDevices[i] = new MCP356x(config);
        isInitialized[i] = true;
    }

    setupChannelMappings();
}

MCP356xScale::~MCP356xScale() {
    for (auto& device : adcDevices) {
        if (device != nullptr) {
            delete device;
            device = nullptr;
        }
    }
}

// update the adc's register values if new data is available
int MCP356xScale::updateAdcsReadings() {
    int status = 0;

    for (int i = 0; i < MAX_ADC_COUNT; ++i) {
        if (isInitialized[i] && adcDevices[i] != nullptr) {
            int updateResult = adcDevices[i]->updatedReadings();
            // If the ADC was not updated, set the corresponding bit in 'status'
            if (updateResult == 0) {
                status |= (1 << i);
            }
        } 
    }
    return status;
}


void MCP356xScale::setupChannelMappings() {
    int globalChannelIndex = 0;
    MCP356xChannel diffChannels[] = {MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D};
    int diffChannelsCount = sizeof(diffChannels) / sizeof(diffChannels[0]); // Number of DIFF channels

    // Assuming adcCount and other relevant members are already set up by the constructor
    for (int adcIndex = 0; adcIndex < adcCount; ++adcIndex) {
        for (int i = 0; i < diffChannelsCount; ++i) {
            if (globalChannelIndex < totalChannels) {
                channelMappings[globalChannelIndex] = {adcIndex, diffChannels[i]};
                globalChannelIndex++;
            } else {
                // Exit the loop if exceeding the total number of scales
                break;
            }
        }
    }
}



double MCP356xScale::getDigitalValue(int globalChannelIndex) {

    if (globalChannelIndex < 0 || globalChannelIndex >= totalChannels) {
        return -2; // Error code for invalid channel index
    }

    ChannelMapping mapping = channelMappings[globalChannelIndex];
    
    if (mapping.adcIndex < 0 || mapping.adcIndex >= adcCount || !isInitialized[mapping.adcIndex]) {
        return -1; // Error code for invalid ADC index or not initialized
    }

    double value = adcDevices[mapping.adcIndex]->value(mapping.localChannel);

    return value;
}

/**
 * @brief Get the average value for a specified channel over a certain number of readings.
 *
 * This function reads a specified channel a number of times and returns the average value.
 * It uses a 32-bit integer to accumulate the sum of readings to avoid potential overflow.
 * If an overflow is about to occur, it breaks out of the averaging loop. The function then
 * divides the sum by the number of readings to compute the average and returns the result 
 * as an int32_t.
 * 
 * @param channel The channel for which the average value is to be computed.
 * @param times The number of readings to be taken for averaging.
 * @return The average value of the specified channel. Returns 0 if no readings were taken.
 */
int32_t MCP356xScale::getAverageValue(int scaleIndex, int times) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return -1; // Handle invalid scale index
    }

    ChannelMapping mapping = channelMappings[scaleIndex];
    if (!isInitialized[mapping.adcIndex]) {
        return -1; // Handle uninitialized ADC device
    }

    int32_t sum = 0; 
    int readingCount = 0;

    for (int i = 0; i < times;) {
        if (adcDevices[mapping.adcIndex]->isr_fired) {
            if (adcDevices[mapping.adcIndex]->updateReadings() == 1) { // Assuming updateReadings returns 1 on new data
                int32_t currentValue = adcDevices[mapping.adcIndex]->value(mapping.localChannel);
                // Check for potential overflow before performing addition
                if ((INT32_MAX - sum) < currentValue) {
                    break; // Break out of the loop on potential overflow
                }
                sum += currentValue;
                readingCount++;
                i++;
            }
        }
    }
    if (readingCount == 0) {
        return 0;
    }
    return sum / readingCount; // Return the average value
}

/**
 * @brief Set the conversion mode for a specific channel and tares the channel.
 * 
 * This method determines how ADC readings from the specified channel will be converted to force. 
 * 
 * @param channel The ADC channel to set the conversion mode for.
 * @param mode The desired conversion mode: SINGLE_VALUE, LINEAR, or POLYNOMIAL.
 */
void MCP356xScale::setConversionMode(ScaleConversionMode mode) {
    _conversionMode = mode;
}

/**
 * @brief Set the scale factor for calibration of a specific ADC channel.
 *
 * This method is useful when calibrating the ADC readings with a single scale factor.
 * 
 * @param channel The ADC channel for which the scale factor is set.
 * @param scale The scale factor to apply for the channel.
 */
void MCP356xScale::setScaleFactor(int globalChannelIndex, float scale) {
    if (globalChannelIndex >= 0 && globalChannelIndex < totalChannels) {
        _conversionModes[globalChannelIndex] = ScaleConversionMode::SINGLE_VALUE;  // Assuming a global conversion mode is still relevant
        scaleFactors[globalChannelIndex] = scale;
    }
}

/**
 * @brief Set the linear calibration parameters for a specific ADC channel.
 *
 * This method applies a linear calibration given by the equation: y = mx + c.
 * Where 'm' is the slope and 'c' is the intercept.
 * 
 * @param channel The ADC channel for which the linear calibration parameters are set.
 * @param slope The slope (m) of the linear calibration.
 * @param intercept The intercept (c) of the linear calibration.
 */
void MCP356xScale::setLinearCalibration(int scaleIndex, float slope, float intercept) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        // Handle invalid scale index, perhaps with an error message or exception
        return;
    }

    // Set the conversion mode specifically for this scale. This may require adjustments
    // to how you track conversion modes if they can differ between scales.
    _conversionModes[scaleIndex] = ScaleConversionMode::LINEAR;

    // Store calibration parameters for the specified scale
    linearSlopes[scaleIndex] = slope;
    linearIntercepts[scaleIndex] = intercept;
}


/**
 * @brief Set the polynomial calibration parameters for a specific ADC channel.
 *
 * This method applies a polynomial calibration given by the equation: y = ax^2 + bx + c.
 * 
 * @param channel The ADC channel for which the polynomial calibration parameters are set.
 * @param a The coefficient for x^2 in the polynomial.
 * @param b The coefficient for x in the polynomial.
 * @param c The constant term in the polynomial.
 */
void MCP356xScale::setPolynomialCalibration(int scaleIndex, float a, float b, float c) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return; // Optionally handle invalid scale index
    }

    // Assuming arrays exist to store polynomial calibration parameters for each scale
    polyA[scaleIndex] = a;
    polyB[scaleIndex] = b;
    polyC[scaleIndex] = c;
}


/**
 * @brief Convert the ADC reading of a specific channel to force using a single scale value.
 *
 * The resulting force is computed by multiplying the calibrated reading with the scale value.
 * 
 * @param channel The ADC channel from which the reading is taken.
 * @return The computed force in grams.
 */
float MCP356xScale::convertToSingleValueForce(int scaleIndex) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return -1.0f; // Optionally handle invalid scale index
    }

    ChannelMapping mapping = channelMappings[scaleIndex];
    if (!isInitialized[mapping.adcIndex]) {
        return -1.0f; // Optionally handle uninitialized ADC device
    }

    int32_t reading = adcDevices[mapping.adcIndex]->value(mapping.localChannel) - offsets[scaleIndex]; // Apply the calibration offset
    return reading * scales[scaleIndex];
}


/**
 * @brief Convert the ADC reading of a specific channel to force using linear regression.
 *
 * This method is used when the ADC has been calibrated with linear parameters (slope and intercept). 
 * The resulting force is computed using the equation: force = slope * reading + intercept.
 * 
 * @param channel The ADC channel from which the reading is taken.
 * @return The computed force in grams-Force.
 */
float MCP356xScale::convertToLinearForce(int scaleIndex) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return -1.0f; // Optionally handle invalid scale index
    }

    ChannelMapping mapping = channelMappings[scaleIndex];
    if (!isInitialized[mapping.adcIndex]) {
        return -1.0f; // Optionally handle uninitialized ADC device
    }

    int32_t reading = adcDevices[mapping.adcIndex]->value(mapping.localChannel);
    return (linearSlopes[scaleIndex] * reading + linearIntercepts[scaleIndex]);
}


/**
 * @brief Convert the ADC reading of a specific channel to force using polynomial regression.
 *
 * This method is used when the ADC has been calibrated using a polynomial equation derived from calibration data. 
 * The resulting force is computed using the equation: force = a * reading^2 + b * reading + c.
 * 
 * @param channel The ADC channel from which the reading is taken.
 * @return The computed force in grams-Force.
 */
float MCP356xScale::convertToPolynomialForce(int scaleIndex) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return -1.0f; // Optionally handle invalid scale index
    }

    ChannelMapping mapping = channelMappings[scaleIndex];
    if (!isInitialized[mapping.adcIndex]) {
        return -1.0f; // Optionally handle uninitialized ADC device
    }

    int32_t reading = adcDevices[mapping.adcIndex]->value(mapping.localChannel);

    float quadraticTerm = polyA[scaleIndex];
    float linearTerm = polyB[scaleIndex];
    float constantTerm = polyC[scaleIndex];

    double term1 = quadraticTerm * reading * reading;
    double term2 = linearTerm * reading;

    return (term1 + term2 + constantTerm);
}


/**
 * @brief Retrieve the computed force for a specified channel based on the set conversion mode.
 * 
 * This method utilizes the conversion mode (SINGLE_VALUE, LINEAR, or POLYNOMIAL) set during initialization 
 * to determine how ADC readings from the specified channel are converted into force values.
 * 
 * @param channel The ADC channel for which the force value is to be computed.
 * @return Computed force value for the channel or NaN if an unknown conversion mode is encountered.
 */
float MCP356xScale::getGramsForce(int scaleIndex) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return NAN; // Handle invalid scale index
    }

    // Retrieve the conversion mode for the specific scale
    ScaleConversionMode mode = _conversionModes[scaleIndex];

    switch (mode) {
        case ScaleConversionMode::SINGLE_VALUE:
            return convertToSingleValueForce(scaleIndex);
        case ScaleConversionMode::LINEAR:
            return convertToLinearForce(scaleIndex);
        case ScaleConversionMode::POLYNOMIAL:
            return convertToPolynomialForce(scaleIndex);
        case ScaleConversionMode::UNDEFINED:
        {
            ChannelMapping mapping = channelMappings[scaleIndex];
            if (!isInitialized[mapping.adcIndex]) {
                return NAN; // Handle uninitialized ADC device
            }
            return static_cast<float>(adcDevices[mapping.adcIndex]->value(mapping.localChannel));
        }
        default:
            return NAN;
    }
}


/**
 * @brief Retrieves the force reading in Newtons for a given channel.
 *
 * This function internally calls getGramsForce() and converts the result 
 * from grams to Newtons using gravitational acceleration.
 *
 * @param channel The MCP356x channel for which the force is to be retrieved.
 * @return The force reading in Newtons.
 */
float MCP356xScale::getForce(int scaleIndex) {
    // Convert grams to kg and then multiply by gravitational acceleration to get the force in newtons
    return getGramsForce(scaleIndex) * GRAVITATIONAL_ACCELERATION / 1000.0f;
}


/**
 * @brief Tares the ADC reading for the specified channel.
 * 
 * This function adjusts the calibration values such that the current ADC reading becomes the zero reference.
 * The method of adjustment depends on the conversion mode currently set (single value, linear, or polynomial).
 * If no conversion mode is set (i.e., UNDEFINED), an error message is printed and the function returns.
 *
 * @param channel The channel for which the tare operation should be performed.
 */
void MCP356xScale::tare(int scaleIndex) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return; // Optionally handle invalid scale index
    }

    ChannelMapping mapping = channelMappings[scaleIndex];
    if (!isInitialized[mapping.adcIndex]) {
        return; // Optionally handle uninitialized ADC device
    }

    MCP356xOversamplingRatio osr = adcDevices[mapping.adcIndex]->getOversamplingRatio();
    int baseSamples = 1000;
    int divisor = static_cast<int>(osr);

    // Ensure we aren't dividing by zero
    divisor = (divisor == 0) ? 1 : divisor;

    int32_t avgReading = getAverageValue(scaleIndex, baseSamples / divisor);

    // Update calibration parameters specifically for this scale
    switch (_conversionModes[scaleIndex]) {
        case ScaleConversionMode::SINGLE_VALUE:
            offsets[scaleIndex] = avgReading; // Assuming offsets is an array storing offset for each scale
            break;
        case ScaleConversionMode::LINEAR:
            linearIntercepts[scaleIndex] = -linearSlopes[scaleIndex] * avgReading;
            break;
        case ScaleConversionMode::POLYNOMIAL:
            polyC[scaleIndex] = -polyA[scaleIndex] * avgReading * avgReading - polyB[scaleIndex] * avgReading;
            break;
        default:
            return; // Handle UNDEFINED mode or any other unexpected values
    }
}


/// private methods

/**
 * @brief Sets the reading offset for a specific ADC channel.
 *
 * @param channel The ADC channel to set the offset for.
 * @param offset The offset value to be set.
 */
void MCP356xScale::setReadingOffset(int scaleIndex, int32_t offset) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return; // Optionally handle invalid scale index
    }
    offsets[scaleIndex] = offset;
}


/**
 * @brief Gets the reading offset for a specific ADC channel.
 *
 * @param channel The ADC channel to retrieve the offset for.
 * @return The offset value of the specified channel.
 */
int32_t MCP356xScale::getReadingOffset(int scaleIndex) {
    if (scaleIndex < 0 || scaleIndex >= totalChannels) {
        return 0; // Optionally handle invalid scale index and return a default offset
    }
    return offsets[scaleIndex];
}




