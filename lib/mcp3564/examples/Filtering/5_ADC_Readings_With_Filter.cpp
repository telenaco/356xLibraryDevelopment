    /**
 * @file ADC_Readings_With_Filter.ino
 * @brief ADC Reading with Butterworth Filter
 *
 * This script reads data from an MCP356x ADC and applies a Butterworth filter
 * to the readings. The main goal is to capture data from the ADC, filter it,
 * and then send the filtered and raw data to the serial port.
 *
 * The script captures and prints the following details: 
 * - Current microsecond timestamp
 * - Raw ADC reading
 * - Butterworth-filtered value
 * - Loop counter (for tracking iterations)
 * - Elapsed time since the last successful ADC reading
 *
 * An explicit delay is added to ensure consistent serial print messages
 * and compensate for ADC read time.
 *
 * Note: Ensure that all required libraries, including Filters and MCP356x, are installed.
 *
 * @author [Your Name]
 * @date [Date of Creation]
 */
#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

    // MCP356x Constants
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2   // Chip select for ADC
#define ADC_IRQ_PIN 3  // Interrupt for ADC
#define MCLK_PIN 0     //

const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;  // Using DIFF_A channel

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

void setupADC() {
    scale->setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    scale->setScanChannels(1, LOADCELL_CHANNEL);
}

void setup() {
    Serial.begin(115200);
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    scale = new MCP356x(config);
    setupADC();
    delay(500);
}

    // Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11111;
constexpr double CUT_OFF_FREQUENCY  = 2;
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
auto      butterworthFilter         = butter<1>(NORMALIZED_CUT_OFF);

void loop() {
    static uint32_t lastSuccessfulReadTime = 0;  // Time of the last successful read
    static uint32_t loopCounter            = 0;  // Counter to keep track of loop iterations
           loopCounter++;
           
    if (scale->updatedReadings()) {
        int      rawReading               = scale->value(LOADCELL_CHANNEL);
        float    butterworthFilteredValue = butterworthFilter(rawReading);
        uint32_t currentMicros            = micros();
        uint32_t elapsedMicros            = currentMicros - lastSuccessfulReadTime;

        StringBuilder output;
        output.concatf("%lu,%d,%d,%lu,%lu",
                       currentMicros,
                       rawReading,
                       static_cast<int>(butterworthFilteredValue),
                       loopCounter,
                       elapsedMicros);

        Serial.println((char*)output.string());
        lastSuccessfulReadTime = currentMicros;
    }

}