/*
This program demonstrates using the MCP356x ADC to make continuous
measurements on a single differential channel.

The ADC is configured for the DIFF_A channel. An interrupt is triggered
when each conversion is complete.

In the main loop, the number of conversions is counted over a 5 second
interval. The sample rate is then calculated and printed.
*/

#include "MCP356x.h"
#include <SPI.h>

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 7  // Chip select for ADC
#define ADC_IRQ_PIN 6 // Interrupt for ADC
#define MCLK_PIN 0    //

// Define the configuration for the MCP356x
MCP356xConfig config = {
    .irq_pin = 6,                                // IRQ pin
    .cs_pin = 7,                                 // Chip select pin
    .mclk_pin = 0,                               // MCLK pin
    .addr = 0x01,                                // Device address (GND in this case)
    .spiInterface = &SPI,                        // SPI interface to use
    .numChannels = 1,                            // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,     // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                 // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};

// Pointer for dynamic creation
MCP356x* adc = nullptr;

unsigned long startTime;
unsigned int sampleCount = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("\n\n");

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  adc = new MCP356x(config);

  adc->setScanChannels(1, MCP356xChannel::DIFF_A);
  //adc->setScanChannels(4, MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D);
  startTime = micros();
}

void loop() {

  if (adc->updatedReadings()) {
    sampleCount++;
  }

  if (micros() - startTime >= 5000000) {
    float samplesPerSecond = sampleCount / 5.0;
    float averageTimePerSample = 5000000.0 / sampleCount;

    StringBuilder output;
    output.concatf("updates per second: %.2f samples/s\n", samplesPerSecond);
    output.concatf("Average time per update: %.2f us/sample\n", averageTimePerSample);
    adc->printTimings(&output);

    Serial.print((char*)output.string());

    // Reset for the next interval
    startTime = micros();
    sampleCount = 0;
  }
}

