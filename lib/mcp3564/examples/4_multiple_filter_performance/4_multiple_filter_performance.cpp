#include "MCP356x.h"
#include <Filters.h>

#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/MedianFilter.hpp>        // MedianFilter
#include <Filters/Butterworth.hpp>
#include <Filters/Notch.hpp>
#include <Filters/SMA.hpp>                 // SMA (Simple Moving Average)
#include <AH/STL/cmath>                    


// MCP356x Constants
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2  // Chip select for ADC
#define ADC_IRQ_PIN 3 // Interrupt for ADC
#define MCLK_PIN 0    //

const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;  // Using DIFF_A channel

MCP356x scale(ADC_IRQ_PIN, ADC_CS_PIN, MCLK_PIN);  

void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(1, LOADCELL_CHANNEL);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

const int BUFFER_SIZE = 50;  // Adjust this to the desired size
int buffer[BUFFER_SIZE];
int head = 0;
bool bufferFull = false;
int runningSum = 0;

int computeRunningAverage(int value) {
    if (bufferFull) {
        // Subtract the oldest value from the sum
        runningSum -= buffer[head];
    }

    // Add the new value to the sum
    runningSum += value;

    // Store the new value in the buffer
    buffer[head] = value;
    head = (head + 1) % BUFFER_SIZE;

    // If the buffer is full
    if (head == 0) {
        bufferFull = true;
    }

    // Return the average value
    if (bufferFull) {
        return runningSum / BUFFER_SIZE;
    } else {
        return runningSum / head;
    }
}

void setup() {
  Serial.begin(2000000);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  setupADC(scale);

  scale.tare(LOADCELL_CHANNEL);
  delay(500);
}


// Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11111; // Hz
constexpr double CUT_OFF_FREQUENCY = 10; // Hz
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
constexpr double NOTCH_FREQUENCY = 50; // Hz
constexpr double NORMALIZED_NOTCH = 2 * NOTCH_FREQUENCY / SAMPLING_FREQUENCY;

Timer<micros> sampleTimer = std::round(1e6 / SAMPLING_FREQUENCY);
auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);
SMA<50, int32_t, int32_t> simpleMovingAvg = {0};
MedianFilter<50, int32_t> medianFilter = {0};
auto primaryNotchFilter = simpleNotchFIR(NOTCH_FREQUENCY);
auto secondaryNotchFilter = simpleNotchFIR(0.06360676);

const int serialBufferSize = 512;
char serialBuffer[serialBufferSize];
int bufferIndex = 0;

void sendBuffer() {
    if (bufferIndex > 0) {
        Serial.write(reinterpret_cast<char*>(serialBuffer), bufferIndex);
        bufferIndex = 0;
        Serial.flush();  // Ensure all data is transmitted
    }
}

void loop() {
    if (scale.isr_fired && scale.read() == 2) {
        int rawReading = scale.getDigitalValue(LOADCELL_CHANNEL);
        float butterworthFilteredValue = butterworthFilter(rawReading);
        int averagedValue = simpleMovingAvg(rawReading);
        int medianFilteredValue = medianFilter(rawReading);
        float notchedValue = secondaryNotchFilter(primaryNotchFilter(rawReading));
        int customRunningAverage = computeRunningAverage(rawReading);
        int currentMicros = micros();

        char output[60];  
        int len = snprintf(output, sizeof(output), "%d,%d,%d,%d,%d,%d,%d\r\n",
                           currentMicros,
                           rawReading, 
                           static_cast<int>(butterworthFilteredValue), 
                           averagedValue, 
                           medianFilteredValue,
                           static_cast<int>(notchedValue),
                           customRunningAverage);

        // Append the data to the buffer
        for (int i = 0; i < len; i++) {
            serialBuffer[bufferIndex++] = output[i];
            if (bufferIndex >= serialBufferSize) {
                sendBuffer();  // If buffer is full, send its content
            }
        }
    }
    delayMicroseconds(90);
}

