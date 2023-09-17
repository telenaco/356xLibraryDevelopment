#include <SPI.h>
#include "MCP356x.h"

// Define pins for the MCP356x
#define SDI_PIN             11
#define SDO_PIN             12
#define SCK_PIN             13
#define ADC_CS_PIN          2  // Chip select for ADC
#define ADC_IRQ_PIN         3  

// Create a configuration object for the MCP356x
MCP356xConfig adcConfig;

// Set up the configuration
void configureADC() {
  adcConfig.flags = MCP356X_FLAG_USE_INTERNAL_CLK;
  adcConfig.mode = MCP356xMode::CONTINUOUS;
  adcConfig.gain = MCP356xGain::GAIN_1;
  adcConfig.over = MCP356xOversamplingRatio::OSR_64;
  adcConfig.prescaler = MCP356xAMCLKPrescaler::OVER_1;  
}

// Create an MCP356x object
MCP356x adc(ADC_IRQ_PIN, ADC_CS_PIN, &adcConfig);

void setup() {
  Serial.begin(115200);
  
  // Initialize SPI and set pins
  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  // Configure and initialize the MCP356x ADC
  configureADC();
  adc.init(&SPI);

  Serial.println("MCP356x ADC initialized!");
}

void loop() {
  // Check if a scan is complete
  if (adc.scanComplete()) {
    // Read the values from the four DIFF channels
    Serial.print("DIFF_A: "); Serial.println(adc.value(MCP356xChannel::DIFF_A));
    Serial.print("DIFF_B: "); Serial.println(adc.value(MCP356xChannel::DIFF_B));
    Serial.print("DIFF_C: "); Serial.println(adc.value(MCP356xChannel::DIFF_C));
    Serial.print("DIFF_D: "); Serial.println(adc.value(MCP356xChannel::DIFF_D));
    Serial.println("------");
    delay(1000);
  }
}
