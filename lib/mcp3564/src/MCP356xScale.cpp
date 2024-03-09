#include "MCP356xScale.h"


MCP356xScale::MCP356xScale(MCP356x *adc, MCP356xChannel channel)
    : adcDevice(adc), usedChannel(channel) { }


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
int32_t MCP356xScale::getAverageValue(int times) {
    int32_t sum          = 0; 
    int     readingCount = 0;

    for (int i = 0; i < times;) {
        if (adcDevice->isr_fired && adcDevice->read() == 2) {
            int32_t currentValue = adcDevice->value(usedChannel);
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
// int32_t MCP356xScale::getAverageValue(int times) {
//     int32_t sum          = 0; 
//     int     readingCount = 0;

//     Serial.println("Starting getAverageValue");

//     for (int i = 0; i < times;) {
//         if (adcDevice->isr_fired) {
//             Serial.println("ISR fired");
//             if (adcDevice->read() == 2) {
//                 Serial.println("Data ready and processed");
//                 int32_t currentValue = adcDevice->value(usedChannel);
//                 Serial.print("Current value: "); Serial.println(currentValue);

//                 // Check for potential overflow before performing addition
//                 if ((sum + currentValue) > INT32_MAX) {
//                     Serial.println("Potential overflow detected, breaking loop");
//                     break; // Break out of the loop on potential overflow
//                 }
//                 sum += currentValue;
//                 readingCount++;
//                 i++;
//             } else {
//                 Serial.println("Data not ready or not fully processed");
//             }
//         } else {
//             //Serial.println("ISR not fired yet");
//         }
//     }

//     if (readingCount == 0) {
//         Serial.println("No readings taken, returning 0");
//         return 0;
//     }

//     int32_t average = static_cast<int32_t>(sum / readingCount);
//     Serial.print("Average value: "); Serial.println(average);
//     return average;
// }


/**
 * @brief Retrieve the digital value of a specified ADC channel from the last reading.
 * 
 * @param channel The ADC channel for which the digital value is needed.
 * @return The digital value of the specified channel from the last reading.
 */
int32_t MCP356xScale::getDigitalValue() {
    return adcDevice->value(usedChannel);
}

/**
 * @brief Set the conversion mode for a specific channel and tares the channel.
 * 
 * This method determines how ADC readings from the specified channel will be converted to force. 
 * 
 * @param channel The ADC channel to set the conversion mode for.
 * @param mode The desired conversion mode: SINGLE_VALUE, LINEAR, or POLYNOMIAL.
 */
// void MCP356xScale::setConversionMode(ScaleConversionMode mode) {
//     _conversionMode = mode;
// }

/**
 * @brief Set the scale factor for calibration of a specific ADC channel.
 *
 * This method is useful when calibrating the ADC readings with a single scale factor.
 * 
 * @param channel The ADC channel for which the scale factor is set.
 * @param scale The scale factor to apply for the channel.
 */
void MCP356xScale::setScaleFactor(float scale) {
    _conversionMode = ScaleConversionMode::SINGLE_VALUE;
    _scale = scale;
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
void MCP356xScale::setLinearCalibration(float slope, float intercept) {
    _conversionMode = ScaleConversionMode::LINEAR;
    _linearSlope     = slope;
    _linearIntercept = intercept;
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
void MCP356xScale::setPolynomialCalibration(float a, float b, float c) {
    _conversionMode = ScaleConversionMode::POLYNOMIAL;
    _polyA = a;
    _polyB = b;
    _polyC = c;
}

/**
 * @brief Convert the ADC reading of a specific channel to force using a single scale value.
 *
 * The resulting force is computed by multiplying the calibrated reading with the scale value.
 * 
 * @param channel The ADC channel from which the reading is taken.
 * @return The computed force in grams.
 */
float MCP356xScale::convertToSingleValueForce() {
    int32_t reading = adcDevice->value(usedChannel) - _offset; // Apply the calibration offset
    return reading * _scale;
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
float MCP356xScale::convertToLinearForce() {
    int32_t reading = adcDevice->value(usedChannel);

    return (_linearSlope * reading + _linearIntercept);
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
float MCP356xScale::convertToPolynomialForce() {
    int32_t reading = adcDevice->value (usedChannel);

    float quadraticTerm = _polyA;
    float linearTerm = _polyB;
    float constantTerm = _polyC;

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
float MCP356xScale::getGramsForce() {
    switch (_conversionMode) {
        case ScaleConversionMode::SINGLE_VALUE:
            return convertToSingleValueForce();
        case ScaleConversionMode::LINEAR:
            return convertToLinearForce();
        case ScaleConversionMode::POLYNOMIAL:
            return convertToPolynomialForce();
        case ScaleConversionMode::UNDEFINED:
            return static_cast<float>(adcDevice->value(usedChannel));
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
float MCP356xScale::getForce() {
    // Convert grams to kg and then multiply by gravitational acceleration
    return getGramsForce() * GRAVITATIONAL_ACCELERATION / 1000.0f;
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
void MCP356xScale::tare() {
    MCP356xOversamplingRatio osr = adcDevice->getOversamplingRatio();
    int baseSamples = 1000;
    int divisor = static_cast<int>(osr);
    
    // Ensure we aren't dividing by zero for OSR_32
    divisor = (divisor == 0) ? 1 : divisor;
    
    int32_t avgReading = getAverageValue( baseSamples / divisor);

    switch (_conversionMode) {
        case ScaleConversionMode::SINGLE_VALUE:
            // For single value, just set the reading offset.
            _offset = avgReading;
            break;

        case ScaleConversionMode::LINEAR:
            // For linear, adjust the intercept.
            _linearIntercept = -_linearSlope * avgReading;
            break;

        case ScaleConversionMode::POLYNOMIAL:
            // For polynomial, adjust the constant term.
            _polyC = -_polyA * avgReading * avgReading - _polyB * avgReading;
            break;

        default:
            // UNDEFINED case and any other unexpected values.
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
void MCP356xScale::setReadingOffset(int32_t offset) {
    _offset = offset;
}

/**
 * @brief Gets the reading offset for a specific ADC channel.
 *
 * @param channel The ADC channel to retrieve the offset for.
 * @return The offset value of the specified channel.
 */
int32_t MCP356xScale::getReadingOffset() {
    return _offset;
}



