/**
 * @file 5_ADC_Readings_With_Filter.cpp
 * @brief ADC Reading with Butterworth Filter
 *
 * This script reads data from an MCP356x ADC and applies a Butterworth filter
 * to the readings. The main goal is to capture data from the ADC, filter it,
 * and then send the filtered and raw data to the serial port.
 *
 * Hardware Connections:
 * - SDI_PIN: Connect to MCP356x SDI (Serial Data Input).
 * - SDO_PIN: Connect to MCP356x SDO (Serial Data Output).
 * - SCK_PIN: Connect to MCP356x SCK (Serial Clock Input).
 * - ADC_IRQ_PIN: Connect to MCP356x IRQ (Interrupt Request).
 * - ADC_CS_PIN: Connect to MCP356x CS (Chip Select).
 */

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_IRQ_PIN 6
#define ADC_CS_PIN  7
#define MCLK_PIN 0

const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

MCP356xConfig config = {
    .irq_pin = ADC_IRQ_PIN,
    .cs_pin = ADC_CS_PIN,
    .mclk_pin = 0,
    .addr = 0x01,
    .spiInterface = &SPI,
    .numChannels = 1,
    .osr = MCP356xOversamplingRatio::OSR_32,
    .gain = MCP356xGain::GAIN_1,
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE
};

MCP356x* scale = nullptr;



void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    scale = new MCP356x(config);
}

constexpr double SAMPLING_FREQUENCY = 11111;
constexpr double CUT_OFF_FREQUENCY = 2;
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);

void loop() {
    static uint32_t lastSuccessfulReadTime = 0;
    static uint32_t loopCounter = 0;
    loopCounter++;

    if (scale->updatedReadings()) {
        int rawReading = scale->value(LOADCELL_CHANNEL);
        float butterworthFilteredValue = butterworthFilter(rawReading);
        uint32_t currentMicros = micros();
        uint32_t elapsedMicros = currentMicros - lastSuccessfulReadTime;

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