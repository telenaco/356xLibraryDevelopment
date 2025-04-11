/**
 * @file mcp356x_comparison_hx711_vs_mcp356x_force.cpp
 * @brief Compares raw and filtered force readings from MCP356x and HX711 sensors.
 *
 * This program interfaces with MCP356x and HX711 sensors to measure load cell data.
 * It captures raw readings and processes them through Butterworth filters for both
 * sensor types. The primary purpose is to compare the readings from both sensors side 
 * by side, evaluating accuracy, noise levels, and the effectiveness of filtering
 * for each sensor type.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - MCP_CS_PIN (7): Connect to MCP356x CS (Chip Select)
 * - MCP_IRQ_PIN (6): Connect to MCP356x IRQ (Interrupt Request)
 * - HX711_DOUT_PIN (23): Connect to HX711 DOUT (Data Output)
 * - HX711_SCK_PIN (22): Connect to HX711 SCK (Serial Clock Input)
 */

#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "HX711.h"
#include "MCP356xScale.h"

// Pin definitions for SPI and ADC connections
#define SDI_PIN         11
#define SDO_PIN         12
#define SCK_PIN         13
#define MCP_CS_PIN      7
#define MCP_IRQ_PIN     6
#define HX711_DOUT_PIN  23
#define HX711_SCK_PIN   22

// Filter Configuration for MCP356x
constexpr double MCP_SAMPLING_FREQUENCY = 11000;
constexpr double MCP_CUT_OFF_FREQUENCY  = 48;
constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;

// Filter Configuration for HX711
constexpr double HX711_SAMPLING_FREQUENCY = 80;
constexpr double HX711_CUT_OFF_FREQUENCY  = 10;
constexpr double HX711_NORMALIZED_CUT_OFF = 2 * HX711_CUT_OFF_FREQUENCY / HX711_SAMPLING_FREQUENCY;

// ADC objects
MCP356xScale* mcpScale = nullptr;
HX711 hx711Scale;

// Filter instances
auto butterworthMcpFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
auto butterworthHxFilter  = butter<1>(HX711_NORMALIZED_CUT_OFF);

// Data ready flag for HX711
volatile boolean hx711DataReady = false;

// Reading and filtering variables
int32_t mcpReading           = 0;
int32_t hx711Reading         = 0;
float filteredMcpReading     = 0;
float filteredHx711Reading   = 0;

/**
 * @brief Interrupt service routine for HX711 data ready event
 * 
 * Sets the hx711DataReady flag when new data is available from the HX711 ADC.
 */
void hx711DataReadyISR() {
    if (hx711Scale.is_ready()) {
        hx711DataReady = true;
    }
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, the MCP356x ADC,
 * and the HX711 ADC with calibration and interrupt handling.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize MCP356x ADC
    mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, MCP_IRQ_PIN, MCP_CS_PIN);

    // Configure MCP356x calibration
    mcpScale->setScaleFactor(0, 0.0006093949987217794f);
    // Alternative calibration methods (uncomment to use)
    // mcpScale->setLinearCalibration(0, 0.0006025509378972841f, -538.4288848531679f);
    // mcpScale->setPolynomialCalibration(0, -9.500760302035329e-14f, 0.0006027208452100216f, -538.3644732675577f);
    
    // Tare the MCP356x ADC
    mcpScale->tare(0);

    // Initialize HX711 ADC
    hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    
    // Attach interrupt for HX711 data ready detection
    attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);

    // Configure HX711 calibration
    hx711Scale.set_scale(0.0037841729238619852f);
    hx711Scale.tare(10);
    
    // Print CSV header
    Serial.println("Timestamp (us), MCP Raw, MCP Filtered, HX711 Raw, HX711 Filtered");
}

/**
 * @brief Main program loop
 * 
 * Continuously reads from both ADCs, applies filtering, and outputs
 * comparison data for both raw and filtered readings.
 */
void loop() {
    bool printOutput = false;

    // Check for new MCP356x readings
    if (mcpScale->updatedAdcReadings()) {
        mcpReading = mcpScale->getReading(0);
        filteredMcpReading = butterworthMcpFilter(mcpReading);
        printOutput = true;
    }

    // Check for new HX711 readings
    if (hx711DataReady) {
        hx711Reading = hx711Scale.get_units();
        filteredHx711Reading = butterworthHxFilter(hx711Reading);
        hx711DataReady = false;
        printOutput = true;
    }

    // Output the readings if new data is available from either ADC
    if (printOutput) {
        unsigned long elapsedTime = micros();
        
        StringBuilder output;
        output.concatf("%lu, %ld, %.2f, %ld, %.2f",
                      elapsedTime,
                      mcpReading,
                      filteredMcpReading,
                      hx711Reading,
                      filteredHx711Reading);

        Serial.println((char*)output.string());
    }
}