#include "MCP356x.h"

// get the average value for a single channel 
int32_t MCP356x::getAverageValue(MCP356xChannel channel, int times) {
  int64_t sum = 0;  // Use 64-bit integer for the sum
  int readingCount = 0;

  for (int i = 0; i < times;) {
    if (isr_fired && read() == 2) {
      int32_t currentValue = value(channel);
      // Check for potential overflow before performing addition
      if ((sum + currentValue) > INT32_MAX) {
        break;  // Break out of the loop on potential overflow
      }
      sum += currentValue;
      readingCount++;
      i++;
    }
  }
  if (readingCount == 0) {
    return 0;
  }
  return static_cast<int32_t>(sum / readingCount);  // Convert back to int32_t for the return value
}

void MCP356x::getMultipleChannelAverage(int times, MCP356xChannel channels[], int32_t* averages, size_t num_channels) {
    int64_t sums[num_channels];
    int readingCounts[num_channels];

    // Initialize arrays
    memset(sums, 0, sizeof(sums));
    memset(readingCounts, 0, sizeof(readingCounts));

    for (int i = 0; i < times;) {
        if (isr_fired && read() == 2) {  // Ensure new reads
            for (size_t j = 0; j < num_channels; j++) {
                int32_t currentValue = value(channels[j]);
                // Check for potential overflow and underflow before performing addition
                if (((sums[j] + currentValue) <= INT32_MAX) && ((sums[j] + currentValue) >= INT32_MIN)) {
                    sums[j] += currentValue;
                    readingCounts[j]++;
                }
                // TODO: error handling or notification if there's an overflow.
            }
            i++;
        }
    }

    // Calculate averages for each channel
    for (size_t j = 0; j < num_channels; j++) {
        averages[j] = (readingCounts[j] != 0) ? static_cast<int32_t>(sums[j] / readingCounts[j]) : 0;
    }
}

int32_t MCP356x::getDigitalValue(MCP356xChannel channel) {
  return value(channel);
}

int32_t MCP356x::getCalibratedValue(MCP356xChannel channel) {
  int index = channelToIndex(channel);
  return getDigitalValue(channel) - _offset[index];
}

void MCP356x::setScaleFactor(MCP356xChannel channel, float scale) {
  int index = channelToIndex(channel);
  _scale[index] = scale;
}

void MCP356x::setLinearCalibration(MCP356xChannel channel, float slope,
                                   float intercept) {
  int index = channelToIndex(channel);
  _linearSlope[index] = slope;
  _linearIntercept[index] = intercept;
}

void MCP356x::setPolynomialCalibration(MCP356xChannel channel, float a, float b,
                                       float c) {
  int index = channelToIndex(channel);
  _polyA[index] = a;
  _polyB[index] = b;
  _polyC[index] = c;
}

// Compute force using a single scale value.
float MCP356x::convertToSingleValueForce(MCP356xChannel channel) {
  // int32_t reading = _tared ? getCalibratedValue(channel) :
  // getDigitalValue(channel);
  int32_t reading = getCalibratedValue(channel);
  int index = channelToIndex(channel);
  return reading * _scale[index];  //* GRAVITATIONAL_ACCELERATION;
}

// Compute force using linear regression
float MCP356x::convertToLinearForce(MCP356xChannel channel) {
  int32_t reading = value(channel);
  int index = channelToIndex(channel);

  return (_linearSlope[index] * reading +
          _linearIntercept[index]);  //* GRAVITATIONAL_ACCELERATION;
}

// Compute force using polynomial regression
float MCP356x::convertToPolynomialForce(MCP356xChannel channel) {
  int32_t reading = getDigitalValue(channel);
  int index = channelToIndex(channel);

  float termA = _polyA[index];
  float termB = _polyB[index];
  float termC = _polyC[index];

  double term1 = termA * reading * reading;
  double term2 = termB * reading;

  return (term1 + term2 + termC);  //* GRAVITATIONAL_ACCELERATION;
}

// Standard method to get force
float MCP356x::getForce(MCP356xChannel channel) {
  switch (_conversionMode) {
    case MCP356xConversionMode::SINGLE_VALUE: 
      return convertToSingleValueForce(channel);
    case MCP356xConversionMode::LINEAR: 
      return convertToLinearForce(channel);
    case MCP356xConversionMode::POLYNOMIAL: 
      return convertToPolynomialForce(channel);
    default: 
      return NAN;
  }
}

void MCP356x::tare(MCP356xChannel channel) {
  int32_t avgReading = getAverageValue(channel, 2000);
  int index = channelToIndex(channel);

  switch (_conversionMode) {
    case MCP356xConversionMode::SINGLE_VALUE: 
        // For single value, just set the reading offset.
      _offset[index] = avgReading;
      break;

    case MCP356xConversionMode::LINEAR: 
        // For linear, adjust the intercept.
      _linearIntercept[index] = -_linearSlope[index] * avgReading;
      break;

    case MCP356xConversionMode::POLYNOMIAL: 
        // For polynomial, adjust the constant term.
      _polyC[index] = 
          -_polyA[index] * avgReading * avgReading - _polyB[index] * avgReading;
      break;
  }
  _tared = true;
}

float MCP356x::getScaleFactor(MCP356xChannel channel) {
  int index = channelToIndex(channel);
  return _scale[index];
}

void MCP356x::setReadingOffset(MCP356xChannel channel, int32_t offset) {
  int index = channelToIndex(channel);
  _offset[index] = offset;
}

int32_t MCP356x::getReadingOffset(MCP356xChannel channel) {
  int index = channelToIndex(channel);
  return _offset[index];
}

void MCP356x::setConversionMode(MCP356xChannel channel,
                                MCP356xConversionMode mode) {
  if (mode == MCP356xConversionMode::SINGLE_VALUE) {
    tare(channel);
  }
  _conversionMode = mode;
}

/***********************helper functions******************************/

int MCP356x::channelToIndex(MCP356xChannel channel) {
  // Convert channel to index, adjusting for DIFF channels starting from DIFF_A
  // = 0x08.
  int index = static_cast<int>(channel) - 8;

  // Ensure the calculated index is within the bounds of our calibration arrays.
  if (index < 0 || index >= NUM_CHANNELS) {
    return -1;
  }
  return index;
}
