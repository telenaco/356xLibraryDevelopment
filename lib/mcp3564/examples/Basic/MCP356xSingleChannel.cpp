#include "Arduino.h"
#include "MCP356x.h"

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13


// Define the configuration for the MCP356x
MCP356xConfig config = {
    .irq_pin = 3,          // IRQ pin
    .cs_pin = 2,          // Chip select pin
    .mclk_pin = 0,         // MCLK pin
    .addr = 0x01,  // Device address (GND in this case)
    .spiInterface = &SPI,  // SPI interface to use
    .numChannels = 1,      // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,  // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,  // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};

// Pointer for dynamic creation
MCP356x* adc = nullptr;

void setup() {

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    Serial.begin(115200);

    adc = new MCP356x(config);
}

void loop() {
    // Update readings from the ADC
    if (adc->updateReadings()) {
        // If new data has been updated, process it

        // Get voltage value
        float voltage = adc->valueAsVoltage(MCP356xChannel::DIFF_A);

        // Get raw ADC value
        int32_t adc_value = adc->value(MCP356xChannel::DIFF_A);

        // Format and print the ADC value and voltage
        char text[30];
        snprintf(text, 30, "%ld, %f", adc_value, voltage);
        Serial.println(text);
    }
}
