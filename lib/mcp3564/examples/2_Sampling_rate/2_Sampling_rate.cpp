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
#define ADC_CS_PIN 2  // Chip select for ADC
#define ADC_IRQ_PIN 3 // Interrupt for ADC
#define MCLK_PIN 0    //

// MCP356x Global Variables
MCP356x adc0(ADC_IRQ_PIN, ADC_CS_PIN, MCLK_PIN);
unsigned long startTime;
unsigned int sampleCount = 0;

void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(1, MCP356xChannel::DIFF_A);
  adc.setReferenceRange(-1.5f, 1.5f);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void setup() {
  Serial.begin(115200);
  Serial.print("\n\n");

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  setupADC(adc0);

  startTime = millis();
}

void loop() {
  StringBuilder output;

  if (adc0.isr_fired) {
    if (2 == adc0.read()) {
      sampleCount++;
    }
  }

  // Check if 5 seconds have passed
  if (millis() - startTime >= 5000) {
    float samplesPerSecond =
        sampleCount / 5.0 / 1000.0; // Convert to kilo samples per second
    Serial.print("Samples per second: ");
    Serial.print(samplesPerSecond, 2); // Print with 2 decimal places
    Serial.println(" ksps");

    // Print timings
    StringBuilder output;
    adc0.printTimings(&output);
    Serial.print((char *)output.string());

    // Reset for the next interval
    startTime = millis();
    sampleCount = 0;
  }
}