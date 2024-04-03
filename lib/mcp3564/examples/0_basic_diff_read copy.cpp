/**
* File       : 0_basic_diff_read.cpp
* Description: Demonstrates basic usage of the MCP356x ADC for differential readings.
* This example initializes the MCP356x ADC, configures it for differential input mode,
* and periodically reads the voltage and raw ADC values from a differential channel.
*
* Hardware Connections:
* - SDI_PIN: Connect to MCP356x SDI (Serial Data Input).
* - SDO_PIN: Connect to MCP356x SDO (Serial Data Output).
* - SCK_PIN: Connect to MCP356x SCK (Serial Clock Input).
* - IRQ_PIN: Connect to MCP356x IRQ (Interrupt Request, optional for this example).
* - CS_PIN: Connect to MCP356x CS (Chip Select).
*
* The example uses SPI for communication with the MCP356x and prints ADC values and
* voltages to the serial console every 2 seconds.
*/

#include "Arduino.h"
#include "MCP356x.h"

// SPI pin definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

unsigned long startTime;  // Tracks time for periodic ADC reading

// MCP356x configuration setup
MCP356xConfig config = {
    .irq_pin = 6,                                   // IRQ pin
    .cs_pin = 7,                                   // Chip select pin
    .mclk_pin = 0,                                   // MCLK pin (0 if using internal clock)
    .addr = 0x01,                                // Device address (GND in this case)
    .spiInterface = &SPI,                                // SPI interface
    .numChannels = 1,                                   // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,    // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                 // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE  // ADC conversion mode
};

// Pointer for dynamic ADC device creation
MCP356x* adc = nullptr;

void setup() {
  Serial.begin(115200);
  while (!Serial) { // Wait for Serial port to connect
    delay(10);
  }

  // SPI configuration
  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  adc = new MCP356x(config);
  startTime = millis(); // Initialize start time
}

void loop() {
  adc->updatedReadings();  // update readings

  if (millis() - startTime >= 2000) { // Every 2 seconds
    float voltage = adc->valueAsVoltage(MCP356xChannel::DIFF_A);  // Read voltage and raw ADC value from differential channel A

    int32_t adc_value = adc->value(MCP356xChannel::DIFF_A);

    StringBuilder output;  // Format and print the ADC value and voltage
    output.concatf("ADC Value: %ld, Voltage: %.2fV\n", adc_value, voltage);

    if (output.length() > 0) Serial.print((char*)output.string());

    startTime = millis(); // Reset timer for the next interval
  }
}