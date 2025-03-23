/**
* @file 3_multiple_ADC_sampling.cpp
* @brief Demonstrates continuous sampling using multiple MCP356x ADCs.
*
* This program configures three MCP356x ADCs for continuous sampling on differential channels.
* It triggers an interrupt when each conversion is complete. The main loop counts the number of conversions
* for each ADC over a 5-second interval and calculates the sample rates.
*
* Hardware Connections:
* - SDI_PIN           : Connect to all MCP356x SDI pins (Serial Data Input).
* - SDO_PIN           : Connect to all MCP356x SDO pins (Serial Data Output).
* - SCK_PIN           : Connect to all MCP356x SCK pins (Serial Clock Input).
* - ADC0_IRQ_PIN      : Connect to MCP356x ADC0 IRQ pin (Interrupt Request).
* - ADC0_CS_PIN       : Connect to MCP356x ADC0 CS pin (Chip Select).
* - ADC1_IRQ_PIN      : Connect to MCP356x ADC1 IRQ pin (Interrupt Request).
* - ADC1_CS_PIN       : Connect to MCP356x ADC1 CS pin (Chip Select).
* - ADC2_IRQ_PIN      : Connect to MCP356x ADC2 IRQ pin (Interrupt Request).
* - ADC2_CS_PIN       : Connect to MCP356x ADC2 CS pin (Chip Select).
*/

#include "MCP356x.h"

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Pins for the three ADCs
#define ADC0_CS_PIN 7
#define ADC1_CS_PIN 4
#define ADC2_CS_PIN 2

#define ADC0_IRQ_PIN 6
#define ADC1_IRQ_PIN 5
#define ADC2_IRQ_PIN 3

    // Define the configuration for the MCP356x
MCP356xConfig config = {
    .irq_pin = ADC0_IRQ_PIN,
    .cs_pin = ADC0_CS_PIN,
    .mclk_pin = 0,
    .addr = 0x01,
    .spiInterface = &SPI,
    .numChannels = 4,
    .osr = MCP356xOversamplingRatio::OSR_32,
    .gain = MCP356xGain::GAIN_1,
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE
};

MCP356x* adc0 = nullptr;
MCP356x* adc1 = nullptr;
MCP356x* adc2 = nullptr;

unsigned long startTime;
unsigned int sampleCount0 = 0;
unsigned int sampleCount1 = 0;
unsigned int sampleCount2 = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("\n\n");

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  adc0 = new MCP356x(config);
  config.cs_pin = ADC1_CS_PIN; config.irq_pin = ADC1_IRQ_PIN;
  adc1 = new MCP356x(config);
  config.cs_pin = ADC2_CS_PIN; config.irq_pin = ADC2_IRQ_PIN;
  adc2 = new MCP356x(config);

  adc0->setSpecificChannels(4);
  adc1->setSpecificChannels(4);
  adc2->setSpecificChannels(4);

  startTime = micros();
}

void loop() {
  if (adc0->updatedReadings()) {
    sampleCount0++;
  }
  if (adc1->updatedReadings()) {
    sampleCount1++;
  }
  if (adc2->updatedReadings()) {
    sampleCount2++;
  }

  // Check if 5 seconds have passed
  if (millis() - startTime >= 5000) {
    float ksps0 = sampleCount0 / 5.0 / 1000.0;
    float ksps1 = sampleCount1 / 5.0 / 1000.0;
    float ksps2 = sampleCount2 / 5.0 / 1000.0;
    float totalKsps = ksps0 + ksps1 + ksps2;

    StringBuilder output;
    output.concatf("ADC0 ksps: %.2f\n", ksps0);
    output.concatf("ADC1 ksps: %.2f\n", ksps1);
    output.concatf("ADC2 ksps: %.2f\n", ksps2);
    output.concatf("Total ksps: %.2f\n", totalKsps);
    Serial.print((char*)output.string());


    // Reset for the next interval
    startTime = millis();
    sampleCount0 = 0;
    sampleCount1 = 0;
    sampleCount2 = 0;
  }
}