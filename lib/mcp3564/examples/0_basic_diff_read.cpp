
#include "Arduino.h"
#include "MCP356x.h"

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

unsigned long startTime;

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

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  // config.irq_pin = 3; config.cs_pin = 2; // to use the other available adc on the board. 
  // config.irq_pin = 5; config.cs_pin = 4;
  adc = new MCP356x(config);
  startTime = millis();
}

void loop() {
  // if (adc->updatedReadings()) {
  // }


  if (millis() - startTime >= 5000) {

      // Get voltage value
    float voltage = adc->valueAsVoltage(MCP356xChannel::DIFF_A);

      // Get raw ADC value
    int32_t adc_value = adc->value(MCP356xChannel::DIFF_A);

      // Format and print the ADC value, voltage, and elapsed time
    StringBuilder output;
    output.concatf("ADC Value: %ld, Voltage: %.2fV\n", adc_value, voltage);
    // Get the current time in microseconds

    adc->printTimings(&output);

    if (output.length() > 0) Serial.print((char*)output.string());
    // Reset for the next interval
    startTime = millis();

  }
}

