/**
 * @file single_channel_operations.cpp
 * @brief Demonstrates basic operations with a single channel of the MCP356x ADC.
 *
 * This example shows how to initialize and read data from a single differential channel
 * of the MCP356x ADC. It continuously monitors the ADC for new readings, retrieves both
 * raw ADC values and converted voltage values, and outputs them via serial communication.
 * This serves as a minimal implementation for applications requiring single-channel monitoring.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - IRQ_PIN (3): Connect to MCP356x IRQ (Interrupt Request)
 * - CS_PIN (2): Connect to MCP356x CS (Chip Select)
 */

 #include "Arduino.h"
 #include "MCP356x.h"
 
 // Pin definitions for SPI communication
 #define SDI_PIN 11
 #define SDO_PIN 12
 #define SCK_PIN 13
 #define IRQ_PIN 3
 #define CS_PIN  2
 
 /**
  * @brief MCP356x ADC configuration structure
  */
 MCP356xConfig config = {
     .irq_pin = IRQ_PIN,                            // Interrupt Request pin
     .cs_pin = CS_PIN,                              // Chip Select pin
     .mclk_pin = 0,                                 // Master Clock pin (0 for internal clock)
     .addr = 0x01,                                  // Device address (GND in this case)
     .spiInterface = &SPI,                          // SPI interface to use
     .numChannels = 1,                              // Number of channels to scan
     .osr = MCP356xOversamplingRatio::OSR_32,       // Oversampling ratio
     .gain = MCP356xGain::GAIN_1,                   // Gain setting (1x)
     .mode = MCP356xADCMode::ADC_CONVERSION_MODE    // Continuous conversion mode
 };
 
 // Pointer for dynamically created ADC object
 MCP356x* adc = nullptr;
 
 /**
  * @brief Setup function run once at startup
  * 
  * Initializes SPI interface and the MCP356x ADC for single-channel operation.
  */
 void setup() {
     // Initialize serial communication
     Serial.begin(115200);
 
     // Initialize SPI interface
     SPI.setSCK(SCK_PIN);
     SPI.setMISO(SDO_PIN);
     SPI.setMOSI(SDI_PIN);
     SPI.begin();
 
     // Create and initialize the ADC
     adc = new MCP356x(config);
 }
 
 /**
  * @brief Main program loop
  * 
  * Continuously checks for new ADC readings, and when available,
  * retrieves and outputs both voltage and raw ADC values.
  */
 void loop() {
     // Check if the ADC has new data available
     if (adc->updatedReadings()) {
         // Get voltage value from the differential channel A
         float voltage = adc->valueAsVoltage(MCP356xChannel::DIFF_A);
 
         // Get raw ADC value
         int32_t adc_value = adc->value(MCP356xChannel::DIFF_A);
 
         // Format and output the readings
         char text[50];
         snprintf(text, 50, "ADC Value: %ld, Voltage: %.4f V", adc_value, voltage);
         Serial.println(text);
     }
 }