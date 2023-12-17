#include "MCP356xScale.h"


void MCP356xScale::setupADC(SPIClass* spiInterface, int numChannels, MCP356xOversamplingRatio osr) {
        setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
        init(spiInterface);
        configureChannels(numChannels);
        setOversamplingRatio(osr);
        setGain(MCP356xGain::GAIN_1);
        setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
    }

void MCP356xScale::configureChannels(int numChannels) {
        // Call setScanChannels with the correct channels based on numChannels
        switch (numChannels) {
            case 1:
                setScanChannels(1, MCP356xChannel::DIFF_A);
                break;
            case 2:
                setScanChannels(2, MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B);
                break;
            case 3:
                setScanChannels(3, MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C);
                break;
            case 4:
                setScanChannels(4, MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D);
                break;
            default:
                // Handle invalid input
                break;
        }
}

/**
 * @brief Get the average value for a specified channel over a certain number of readings.
 *
 * This function reads a specified channel a number of times and returns the average value.
 * It uses a 64-bit integer to accumulate the sum of readings to avoid potential overflow.
 * If an overflow is about to occur, it breaks out of the averaging loop. The function then
 * divides the sum by the number of readings to compute the average and returns the result 
 * as an int32_t.
 * 
 * @param channel The channel for which the average value is to be computed.
 * @param times The number of readings to be taken for averaging.
 * @return The average value of the specified channel. Returns 0 if no readings were taken.
 */
int32_t MCP356xScale::getAverageValue(MCP356xChannel channel, int times) {
    int64_t sum          = 0; // Use 64-bit integer for the sum
    int     readingCount = 0;

    for (int i = 0; i < times;) {
        if (isr_fired && read() == 2) {
            int32_t currentValue = value(channel);
            // Check for potential overflow before performing addition
            if ((sum + currentValue) > INT32_MAX) {
                break; // Break out of the loop on potential overflow
            }
            sum += currentValue;
            readingCount++;
            i++;
        }
    }
    if (readingCount == 0) {
        return 0;
    }
    return static_cast<int32_t>(sum / readingCount); // Convert back to int32_t for the return value
}

/**
 * @brief Compute average values for multiple channels.
 *
 * This method calculates the average values for multiple ADC channels simultaneously.
 * This is efficient as ADC updates all channel values at once.
 * If an overflow or underflow is about to occur during summation, the addition is skipped.
 * 
 * @param times Number of readings to take for averaging.
 * @param channels Array of channels for which average values are needed.
 * @param averages Output array to store average values of channels.
 * @param num_channels Number of channels in the channels array.
 */
void MCP356xScale::getMultipleChannelAverage(int times, MCP356xChannel channels[], int32_t *averages, size_t num_channels) {
    int64_t sums[num_channels];
    int     readingCounts[num_channels];

    // Initialize sums and reading counts to zero
    memset(sums, 0, sizeof(sums));
    memset(readingCounts, 0, sizeof(readingCounts));

    for (int i = 0; i < times;) {
        // Check for new valid ADC reads
        if (isr_fired && read() == 2) {
            for (size_t j = 0; j < num_channels; j++) {
                int32_t currentValue = value(channels[j]);
                // Ensure the sum won't overflow or underflow
                if (((sums[j] + currentValue) <= INT32_MAX) && ((sums[j] + currentValue) >= INT32_MIN)) {
                    sums[j] += currentValue;
                    readingCounts[j]++;
                }
                // TODO error handling/notification for overflows
            }
            i++;
        }
    }

    // Compute and store average values in the provided output array
    for (size_t j = 0; j < num_channels; j++) {
        averages[j] = (readingCounts[j] != 0) ? static_cast<int32_t>(sums[j] / readingCounts[j]) : 0;
    }
}

/**
 * @brief Retrieve the digital value of a specified ADC channel from the last reading.
 * 
 * @param channel The ADC channel for which the digital value is needed.
 * @return The digital value of the specified channel from the last reading.
 */
int32_t MCP356xScale::getDigitalValue(MCP356xChannel channel) {
    return value(channel);
}

/**
 * @brief Set the conversion mode for a specific channel and tares the channel.
 * 
 * This method determines how ADC readings from the specified channel will be converted to force. 
 * 
 * @param channel The ADC channel to set the conversion mode for.
 * @param mode The desired conversion mode: SINGLE_VALUE, LINEAR, or POLYNOMIAL.
 */
void MCP356xScale::setConversionMode(MCP356xChannel channel, ScaleConversionMode mode) {
    int index = channelToIndex(channel);
    _conversionMode[index] = mode;
}

/**
 * @brief Set the scale factor for calibration of a specific ADC channel.
 *
 * This method is useful when calibrating the ADC readings with a single scale factor.
 * 
 * @param channel The ADC channel for which the scale factor is set.
 * @param scale The scale factor to apply for the channel.
 */
void MCP356xScale::setScaleFactor(MCP356xChannel channel, float scale) {
    int index     = channelToIndex(channel);
    _scale[index] = scale;
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
void MCP356xScale::setLinearCalibration(MCP356xChannel channel, float slope, float intercept) {
    int index               = channelToIndex(channel);
    _linearSlope[index]     = slope;
    _linearIntercept[index] = intercept;
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
void MCP356xScale::setPolynomialCalibration(MCP356xChannel channel, float a, float b, float c) {
    int index     = channelToIndex(channel);
    _polyA[index] = a;
    _polyB[index] = b;
    _polyC[index] = c;
}

/**
 * @brief Convert the ADC reading of a specific channel to force using a single scale value.
 *
 * The resulting force is computed by multiplying the calibrated reading with the scale value.
 * 
 * @param channel The ADC channel from which the reading is taken.
 * @return The computed force in grams.
 */
float MCP356xScale::convertToSingleValueForce(MCP356xChannel channel) {
    int     index   = channelToIndex(channel);
    int32_t reading = value(channel) - _offset[index]; // Apply the calibration offset
    return reading * _scale[index];
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
float MCP356xScale::convertToLinearForce(MCP356xChannel channel) {
    int32_t reading = value(channel);
    int     index   = channelToIndex(channel);

    return (_linearSlope[index] * reading + _linearIntercept[index]);
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
float MCP356xScale::convertToPolynomialForce(MCP356xChannel channel) {
    int32_t reading = value (channel);
    int     index   = channelToIndex(channel);

    float termA = _polyA[index];
    float termB = _polyB[index];
    float termC = _polyC[index];

    double term1 = termA * reading * reading;
    double term2 = termB * reading;

    return (term1 + term2 + termC);
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
float MCP356xScale::getGramsForce(MCP356xChannel channel) {
    int index = channelToIndex(channel);
    switch (_conversionMode[index]) {
        case ScaleConversionMode::SINGLE_VALUE:
            return convertToSingleValueForce(channel);
        case ScaleConversionMode::LINEAR:
            return convertToLinearForce(channel);
        case ScaleConversionMode::POLYNOMIAL:
            return convertToPolynomialForce(channel);
        case ScaleConversionMode::UNDEFINED:
            return static_cast<float>(value(channel));
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
float MCP356xScale::getForce(MCP356xChannel channel) {
    // Convert grams to kg and then multiply by gravitational acceleration
    return getGramsForce(channel) * GRAVITATIONAL_ACCELERATION / 1000.0f;
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
void MCP356xScale::tare(MCP356xChannel channel) {
    MCP356xOversamplingRatio osr = getOversamplingRatio();
    int baseSamples = 5000;
    int divisor = static_cast<int>(osr);
    
    // Ensure we aren't dividing by zero for OSR_32
    divisor = (divisor == 0) ? 1 : divisor;
    
    int32_t avgReading = getAverageValue(channel, baseSamples / divisor);
    int index = channelToIndex(channel);

    switch (_conversionMode[index]) {
        case ScaleConversionMode::SINGLE_VALUE:
            // For single value, just set the reading offset.
            _offset[index] = avgReading;
            break;

        case ScaleConversionMode::LINEAR:
            // For linear, adjust the intercept.
            _linearIntercept[index] = -_linearSlope[index] * avgReading;
            break;

        case ScaleConversionMode::POLYNOMIAL:
            // For polynomial, adjust the constant term.
            _polyC[index] = -_polyA[index] * avgReading * avgReading - _polyB[index] * avgReading;
            break;

        default:
            // UNDEFINED case and any other unexpected values.
            Serial.print("Please set the conversion mode for ");
            Serial.print(channelToString(channel));
            Serial.println(" before taring.");
            return;
    }
}


/// private methods

/**
 * @brief Sets the reading offset for a specific ADC channel.
 *
 * @param channel The ADC channel to set the offset for.
 * @param offset The offset value to be set.
 */
void MCP356xScale::setReadingOffset(MCP356xChannel channel, int32_t offset) {
    int index      = channelToIndex(channel);
    _offset[index] = offset;
}

/**
 * @brief Gets the reading offset for a specific ADC channel.
 *
 * @param channel The ADC channel to retrieve the offset for.
 * @return The offset value of the specified channel.
 */
int32_t MCP356xScale::getReadingOffset(MCP356xChannel channel) {
    int index = channelToIndex(channel);
    return _offset[index];
}

/**
 * @brief Converts an MCP356xChannel enumeration to its corresponding index.
 *
 * Converts a given MCP356xChannel enumeration to its array index, adjusting for the
 * channel configuration. Returns -1 for invalid channels.
 *
 * @param channel The MCP356x channel enumeration to be converted.
 * @return The corresponding array index or -1 for invalid channels.
 */
int MCP356xScale::channelToIndex(MCP356xChannel channel) {
    // Convert channel to index, adjusting for DIFF channels starting from DIFF_A
    int index = static_cast<int>(channel) - 8;

    // Ensure the calculated index is within the bounds of our calibration arrays.
    if (index < 0 || index >= NUM_CHANNELS) {
        return -1;
    }
    return index;
}

/**
 * @brief Converts the MCP356xChannel enum to a human-readable string.
 * 
 * @param channel The MCP356x channel enum value.
 * @return String representation of the MCP356x channel.
 */
const char* MCP356xScale::channelToString(MCP356xChannel channel) const {
    switch (channel) {
        case MCP356xChannel::DIFF_A: return "channel A";
        case MCP356xChannel::DIFF_B: return "channel B";
        case MCP356xChannel::DIFF_C: return "channel C";
        case MCP356xChannel::DIFF_D: return "channel D";
        default: return "unknown channel";
    }
}
