    /**
 * @file 4_multiple_filter_performance.cpp
 * @brief Demonstrates the performance of multiple filters using the MCP356x ADC.
 *
 * This program reads data from the MCP356x ADC and applies various filters to the raw readings.
 * The filters used include Butterworth, Simple Moving Average, Median, Notch, and a custom running average.
 * The filtered values, along with the raw reading and timestamp, are printed to the serial monitor.
 *
 * Hardware Connections: 
 * - SDI_PIN           : Connect to MCP356x SDI (Serial Data Input).
 * - SDO_PIN           : Connect to MCP356x SDO (Serial Data Output).
 * - SCK_PIN           : Connect to MCP356x SCK (Serial Clock Input).
 * - ADC_IRQ_PIN       : Connect to MCP356x IRQ (Interrupt Request).
 * - ADC_CS_PIN        : Connect to MCP356x CS (Chip Select).
 *
 *  use telemetry viewer for better displaying the results
 * http: //www.farrellf.com/TelemetryViewer/
 */

#include "MCP356x.h"
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/MedianFilter.hpp>
#include <Filters/Butterworth.hpp>
#include <Filters/Notch.hpp>
#include <Filters/SMA.hpp>
#include <AH/STL/cmath>

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_IRQ_PIN 6
#define ADC_CS_PIN  7

const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

MCP356xConfig config = {
    .irq_pin      = ADC_IRQ_PIN,
    .cs_pin       = ADC_CS_PIN,
    .mclk_pin     = 0,
    .addr         = 0x01,
    .spiInterface = &SPI,
    .numChannels  = 1,
    .osr          = MCP356xOversamplingRatio::OSR_32,
    .gain         = MCP356xGain::GAIN_1,
    .mode         = MCP356xADCMode::ADC_CONVERSION_MODE
};

MCP356x* scale = nullptr;

const int BUFFER_SIZE = 50;
int buffer[BUFFER_SIZE];
int  head       = 0;
bool bufferFull = false;
int  runningSum = 0;

    /**
 * @brief Computes the running average of the input values.
 *
 * This function maintains a circular buffer of size BUFFER_SIZE and calculates
 * the running average of the values in the buffer. It updates the buffer and
 * the running sum with each new value and returns the current average.
 *
 * @param value The new value to be added to the running average calculation.
 * @return The current running average of the values in the buffer.
 */
int computeRunningAverage(int value) {
    if (bufferFull) {
        runningSum -= buffer[head];
    }
    runningSum   += value;
    buffer[head]  = value;
    head          = (head + 1) % BUFFER_SIZE;
    if (head == 0) {
        bufferFull = true;
    }
    if (bufferFull) {
        return runningSum / BUFFER_SIZE;
    } else {
        return runningSum / head;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    scale = new MCP356x(config);
}

constexpr double SAMPLING_FREQUENCY = 11111;
constexpr double CUT_OFF_FREQUENCY  = 10;
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
constexpr double NOTCH_FREQUENCY    = 50;
constexpr double NORMALIZED_NOTCH   = 2 * NOTCH_FREQUENCY / SAMPLING_FREQUENCY;

Timer<micros>   sampleTimer            = std::round(1e6 / SAMPLING_FREQUENCY);
auto            butterworthFilter      = butter<1>(NORMALIZED_CUT_OFF);
SMA<50         , int32_t,               int32_t> simpleMovingAvg = {0};
MedianFilter<50, int32_t> medianFilter = {0};
auto primaryNotchFilter = simpleNotchFIR(NOTCH_FREQUENCY);
auto secondaryNotchFilter = simpleNotchFIR(0.06360676);

void loop() {
    if (scale->updatedReadings()) {
        int rawReading = scale->value(LOADCELL_CHANNEL);
        float butterworthFilteredValue = butterworthFilter(rawReading);
        int averagedValue = simpleMovingAvg(rawReading);
        int medianFilteredValue = medianFilter(rawReading);
        float notchedValue = secondaryNotchFilter(primaryNotchFilter(rawReading));
        int customRunningAverage = computeRunningAverage(rawReading);

        StringBuilder output;
        output.concatf("%u,%d,%d,%d,%d,%d,%d\r\n",
                       micros(),
                       rawReading,
                       static_cast<int>(butterworthFilteredValue),
                       averagedValue,
                       medianFilteredValue,
                       static_cast<int>(notchedValue),
                       customRunningAverage);

        Serial.print((char*)output.string());
    }
}