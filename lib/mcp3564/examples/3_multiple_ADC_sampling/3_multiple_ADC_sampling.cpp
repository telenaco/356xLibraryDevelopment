/*
This program demonstrates using 3 MCP356x ADCs simultaneously to make 
continuous measurements on each one. 

The 3 ADCs are configured identically for 1 differential channel each.
Interrupts are triggered when conversions are complete.

In the main loop, the number of conversions on each ADC is counted
over a 5 second interval. The sample rate for each ADC is calculated 
and printed, along with the total combined sample rate.
*/

#include "MCP356x.h"
#include <SPI.h>

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Pins for the three ADCs
#define ADC0_CS_PIN 2
#define ADC1_CS_PIN 4
#define ADC2_CS_PIN 6

#define ADC0_IRQ_PIN 3
#define ADC1_IRQ_PIN 5
#define ADC2_IRQ_PIN 7

#define MCLK_PIN 0

MCP356x adc0(ADC0_IRQ_PIN, ADC0_CS_PIN, MCLK_PIN);
MCP356x adc1(ADC1_IRQ_PIN, ADC1_CS_PIN, MCLK_PIN);
MCP356x adc2(ADC2_IRQ_PIN, ADC2_CS_PIN, MCLK_PIN);

unsigned int sampleCount0 = 0;
unsigned int sampleCount1 = 0;
unsigned int sampleCount2 = 0;
unsigned long startTime;

void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(4, MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D );

  adc.setReferenceRange(-1.5f, 1.5f);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void readADC(MCP356x &adc, unsigned int &sampleCounter) {
  if (adc.read() == 2) {
    float voltageA = adc.valueAsVoltage(MCP356xChannel::DIFF_A);
    float voltageB = adc.valueAsVoltage(MCP356xChannel::DIFF_B);
    float voltageC = adc.valueAsVoltage(MCP356xChannel::DIFF_C);
    float voltageD = adc.valueAsVoltage(MCP356xChannel::DIFF_D);
    sampleCounter++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print("\n\n");

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  setupADC(adc0);
  setupADC(adc1);
  setupADC(adc2);
  startTime = millis();
}

void loop() {

  // reading all the channels only when the three chanels are ready. 
  // if (adc0.isrFired() && adc1.isrFired() && adc2.isrFired()) {
  //   // Read all ADCs
  //   readADC(adc0, sampleCount0);
  //   readADC(adc1, sampleCount1);
  //   readADC(adc2, sampleCount2);
  // }

  if (adc0.isrFired()) {
    readADC(adc0, sampleCount0);
  }
  if (adc1.isrFired()) {
    readADC(adc1, sampleCount1);
  }
  if (adc2.isrFired()) {
    readADC(adc2, sampleCount2);
  }

  // Check if 5 seconds have passed
  if (millis() - startTime >= 5000) {
    float ksps0 =
        sampleCount0 / 5.0 / 1000.0; // Convert to kilo samples per second
    float ksps1 = sampleCount1 / 5.0 / 1000.0;
    float ksps2 = sampleCount2 / 5.0 / 1000.0;
    float totalKsps = ksps0 + ksps1 + ksps2;

    Serial.print("ADC0 ksps: ");
    Serial.println(ksps0, 2);

    Serial.print("ADC1 ksps: ");
    Serial.println(ksps1, 2);

    Serial.print("ADC2 ksps: ");
    Serial.println(ksps2, 2);

    Serial.print("Total ksps: ");
    Serial.println(totalKsps, 2);

    // Reset for the next interval
    startTime = millis();
    sampleCount0 = 0;
    sampleCount1 = 0;
    sampleCount2 = 0;
  }
}
