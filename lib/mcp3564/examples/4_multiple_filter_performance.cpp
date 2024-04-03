#include "MCP356x.h"
#include <Filters.h>

#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/MedianFilter.hpp>  // MedianFilter
#include <Filters/Butterworth.hpp>
#include <Filters/Notch.hpp>
#include <Filters/SMA.hpp>  // SMA (Simple Moving Average)
#include <AH/STL/cmath>                    


// MCP356x Constants
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2   // Chip select for ADC
#define ADC_IRQ_PIN 3  // Interrupt for ADC
#define MCLK_PIN 0     //

const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;  // Using DIFF_A channel

// Define the configuration for the MCP356x
MCP356xConfig config = {
    .irq_pin = ADC_IRQ_PIN,                               // IRQ pin
    .cs_pin = ADC_CS_PIN,                                 // Chip select pin
    .mclk_pin = MCLK_PIN,                                 // MCLK pin
    .addr = 0x01,                                         // Device address (GND in this case)
    .spiInterface = &SPI,                                 // SPI interface to use
    .numChannels = 1,                                     // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,              // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                          // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE           // Continuous conversion mode
};

MCP356x* scale = nullptr;

const int BUFFER_SIZE = 50;  // Adjust this to the desired size
int buffer[BUFFER_SIZE];
int  head       = 0;
bool bufferFull = false;
int  runningSum = 0;

int computeRunningAverage(int value) {
    if (bufferFull) runningSum -= buffer[head];
      // Add the new value to the sum
    runningSum += value;
      // Store the new value in the buffer
    buffer[head] = value;
    head         = (head + 1) % BUFFER_SIZE;
      // If the buffer is full
    if (head == 0)bufferFull = true;
      // Return the average value
    if (bufferFull) {
        return runningSum / BUFFER_SIZE;
    } else {
        return runningSum / head;
    }
}

void setup() {
  Serial.begin(115200);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  scale = new MCP356x(config);
  delay(500);
}

constexpr double SAMPLING_FREQUENCY = 11111;  // Hz
constexpr double CUT_OFF_FREQUENCY = 10;      // Hz
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
constexpr double NOTCH_FREQUENCY    = 50;                                          // Hz
constexpr double NORMALIZED_NOTCH   = 2 * NOTCH_FREQUENCY / SAMPLING_FREQUENCY;

Timer<micros>   sampleTimer            = std::round(1e6 / SAMPLING_FREQUENCY);
auto            butterworthFilter      = butter<1>(NORMALIZED_CUT_OFF);
SMA<50         , int32_t,               int32_t> simpleMovingAvg = {0};
MedianFilter<50, int32_t> medianFilter = {0};
auto            primaryNotchFilter     = simpleNotchFIR(NOTCH_FREQUENCY);
auto            secondaryNotchFilter   = simpleNotchFIR(0.06360676);

void loop() {
    if (scale->updatedReadings()) {
        int   rawReading               = scale->value(LOADCELL_CHANNEL);
        float butterworthFilteredValue = butterworthFilter(rawReading);
        int   averagedValue            = simpleMovingAvg(rawReading);
        int   medianFilteredValue      = medianFilter(rawReading);
        float notchedValue             = secondaryNotchFilter(primaryNotchFilter(rawReading));
        int   customRunningAverage     = computeRunningAverage(rawReading);

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
    delayMicroseconds(90);
}

