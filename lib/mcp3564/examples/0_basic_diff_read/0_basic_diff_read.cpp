/*
This program demonstrates reading an analog voltage from an ADC using the 
MCP356x library. The ADC value is printed both as a voltage and raw digital value.

The ADC used is the MCP3564, which has 4 differential input channel. The DIFF_A 
channel is connected to the analog input voltage being to measure.
*/

#include "MCP356x.h"
#include <SPI.h>


#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2 // Chip select for ADC  
#define ADC_IRQ_PIN 3 // Interrupt pin for ADC

//MCP356x adc0(ADC_IRQ_PIN, ADC_CS_PIN, MCLK_PIN);

MCP356xConfig adc0Config = {ADC_IRQ_PIN, ADC_CS_PIN};
MCP356x       adc0(adc0Config);

void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(1, MCP356xChannel::DIFF_A);
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
}

void loop() {
  if (adc0.isr_fired) {
    if (2 == adc0.read()) {
      
      // Get voltage value
      float voltage = adc0.valueAsVoltage(MCP356xChannel::DIFF_A);
      
      // Get raw ADC value
      int32_t adc_value = adc0.value(MCP356xChannel::DIFF_A);

    char text[30];
    snprintf(text, 30, "%lu, %f", adc_value, voltage);
    Serial.println(text);
    }
  }
}