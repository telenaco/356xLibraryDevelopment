  /**
 * Filename: loadcell_readings.cpp
 *
 * Description: 
 * This code captures readings from an MCP356x-based load cell. The readings
 * are filtered using a Butterworth filter to minimize noise and enhance precision.
 * The raw and filtered readings are then sent over the serial port for further processing.
 *
 * Requirements: 
 * - MCP356x library
 * - Filters library
 *
 * Author: [Your Name]
 * Date  : [Date of Creation]
 */
#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

  // MCP356x Pin Configuration
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2   // Chip select for ADC
#define ADC_IRQ_PIN 3  // Interrupt for ADC
#define MCLK_PIN 0     //

const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

  // Define the configuration for the MCP356x
MCP356xConfig config = {
    .irq_pin      = ADC_IRQ_PIN,                         // IRQ pin
    .cs_pin       = ADC_CS_PIN,                          // Chip select pin
    .mclk_pin     = MCLK_PIN,                            // MCLK pin
    .addr         = 0x01,                                // Device address (GND in this case)
    .spiInterface = &SPI,                                // SPI interface to use
    .numChannels  = 1,                                   // Number of channels to scan
    .osr          = MCP356xOversamplingRatio::OSR_32,    // Oversampling ratio
    .gain         = MCP356xGain::GAIN_1,                 // Gain setting
    .mode         = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};

MCP356x* scale = nullptr;

  // Function to set up the ADC
void setupADC() {
    scale->setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    scale->setScanChannels(1, LOADCELL_CHANNEL);
}

void setup() {
    Serial.begin(2000000);
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    scale = new MCP356x(config);
    setupADC();
}

// Butterworth Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11000;  // since are taking a sample every ~90 uSeconds freq is 1/T
constexpr double CUT_OFF_FREQUENCY = 48;
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);

void loop() {
    if (scale->updatedReadings()) {
        int32_t reading = scale->value(LOADCELL_CHANNEL);
        // Apply the Butterworth filter
        int32_t filteredValue = butterworthFilter(reading);

        // Print the raw and filtered readings to the serial port
        StringBuilder output;
        output.concatf("%d, %d", reading, filteredValue);
        Serial.println((char*)output.string());
        Serial.flush();
    }
}