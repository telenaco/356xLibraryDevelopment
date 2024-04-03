#include "Arduino.h"
#include "MCP356x.h"

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Define the configuration for each MCP356x ADC
MCP356xConfig config1 = {
    .irq_pin = 3,          // IRQ pin
    .cs_pin = 2,          // Chip select pin
    .mclk_pin = 0,         // MCLK pin
    .addr = 0x01,  // Device address (GND in this case)
    .spiInterface = &SPI,  // SPI interface to use
    .numChannels = 4,      // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,  // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,  // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};

MCP356xConfig config2 = {
    .irq_pin = 5,          // IRQ pin
    .cs_pin = 4,          // Chip select pin
    .mclk_pin = 0,         // MCLK pin
    .addr = 0x01,  // Device address (GND in this case)
    .spiInterface = &SPI,  // SPI interface to use
    .numChannels = 4,      // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,  // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,  // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};

MCP356xConfig config3 = {
    .irq_pin = 6,          // IRQ pin
    .cs_pin = 7,          // Chip select pin
    .mclk_pin = 0,         // MCLK pin
    .addr = 0x01,  // Device address (GND in this case)
    .spiInterface = &SPI,  // SPI interface to use
    .numChannels = 4,      // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,  // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,  // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};


// Pointers for dynamic creation
MCP356x* adc1 = nullptr;
MCP356x* adc2 = nullptr;
MCP356x* adc3 = nullptr;

// void readAndPrintADC(MCP356x* adc, MCP356xChannel channel, const char* adcName) {
//     if (adc->updateReadings()) {
//         float voltage = adc->valueAsVoltage(channel);
//         int32_t adc_value = adc->value(channel);
//         char text[50]; 
//         snprintf(text, 50, "%s: %ld, %f", adcName, adc_value, voltage);
//         Serial.println(text);
//     }
// }

void readAndPrintADC(MCP356x* adc, const char* adcName) {
     if (adc->updateReadings()) {
        for (int channel = 0; channel < 4; ++channel) {  // Iterate through 4 channels
            int32_t adc_value = adc->value(static_cast<MCP356xChannel>(channel));
            char text[150]; 
            snprintf(text, 50, "%s Channel %d: %ld", adcName, channel, adc_value);
            Serial.println(text);
        }
    }
    Serial.print(adcName);
    Serial.print(" Total Time for 3 Adcs: ");

}

void setup() {
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    Serial.begin(115200);

    // Initialize each MCP356x ADC
    adc1 = new MCP356x(config1);
    adc2 = new MCP356x(config2);
    adc3 = new MCP356x(config3);
}

void loop() {
    unsigned long startTime = micros();

    readAndPrintADC(adc1, "ADC1");
    readAndPrintADC(adc2, "ADC2");
    readAndPrintADC(adc3, "ADC3");

    unsigned long timeTaken = micros() - startTime;
    Serial.print(timeTaken);
    Serial.println(" microseconds");
}
