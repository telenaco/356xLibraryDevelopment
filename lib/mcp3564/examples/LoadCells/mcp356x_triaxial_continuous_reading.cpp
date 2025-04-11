/**
 * @file mcp356x_triaxial_continuous_reading.cpp
 * @brief Captures and filters data from a 3-axis load cell using MCP356x ADCs.
 *
 * This program continuously captures data on three axes (X, Y, Z) and applies
 * filtering to smooth the readings for calibration and monitoring of a 3-axis load cell.
 * It outputs the filtered readings in a CSV format that can be used for calibration analysis
 * and real-time monitoring.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - MCP_ADC1_CS_PIN (2): Connect to first ADC's CS (Chip Select)
 * - MCP_ADC1_IRQ_PIN (3): Connect to first ADC's IRQ (Interrupt Request)
 * - MCP_ADC2_CS_PIN (5): Connect to second ADC's CS (Chip Select)
 * - MCP_ADC2_IRQ_PIN (4): Connect to second ADC's IRQ (Interrupt Request)
 * - MCP_ADC3_CS_PIN (7): Connect to third ADC's CS (Chip Select)
 * - MCP_ADC3_IRQ_PIN (6): Connect to third ADC's IRQ (Interrupt Request)
 */

#include "MCP356x.h"
#include "MCP356x3axis.h"
#include "MCP356xScale.h"

/** @brief Pins Configuration */
const uint8_t SDI_PIN = 11;
const uint8_t SDO_PIN = 12;
const uint8_t SCK_PIN = 13;

// Pins for ADC1, ADC2 and ADC3
const uint8_t MCP_ADC1_CS_PIN = 2;
const uint8_t MCP_ADC1_IRQ_PIN = 3;
const uint8_t MCP_ADC2_CS_PIN = 5;
const uint8_t MCP_ADC2_IRQ_PIN = 4;
const uint8_t MCP_ADC3_CS_PIN = 7;
const uint8_t MCP_ADC3_IRQ_PIN = 6;

// Configuration for ADC1, ADC2 & ADC3
MCP356xConfig adc1Config = {MCP_ADC1_IRQ_PIN, MCP_ADC1_CS_PIN};
MCP356x adc1(adc1Config);

// Three-axis Load Cell Instances
// Load cell one (using channels A (x), B(y), C(z) from ADC1)
MCP356xScale scaleA1(&adc1, MCP356xChannel::DIFF_A);
MCP356xScale scaleB1(&adc1, MCP356xChannel::DIFF_B);
MCP356xScale scaleC1(&adc1, MCP356xChannel::DIFF_C);

// Create ThreeAxisLoadCell instances, x, y & z
MCP356x3axis threeAxisLoadCell(&scaleA1, &scaleB1, &scaleC1);

/**
 * @brief Filter class for applying moving average to sensor readings.
 *
 * This class implements a moving average filter to smooth out noise from
 * load cell readings, improving measurement stability.
 */
class Filter
{
private:
    float *buffer;
    int index;
    int size;
    float sum;

public:
    /**
     * @brief Constructor for the Filter class.
     *
     * @param _size Size of the filter window
     */
    Filter(int _size) : index(0), size(_size), sum(0)
    {
        buffer = new float[size];
        for (int i = 0; i < size; ++i)
        {
            buffer[i] = 0.0;
        }
    }

    /**
     * @brief Destructor for the Filter class.
     */
    ~Filter()
    {
        delete[] buffer;
    }

    /**
     * @brief Adds a new value to the buffer and computes the moving average.
     *
     * @param value New value to add to the filter
     * @return Moving average of values in the filter window
     */
    float addValue(float value)
    {
        sum -= buffer[index];
        buffer[index] = value;
        sum += value;
        index = (index + 1) % size;
        return sum / size;
    }
};

// Instantiate filter with desired buffer size
Filter filterX(10000);
Filter filterY(10000);
Filter filterZ(10000);

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 * Configures ADC1 to scan 3 channels with OSR_64 oversampling ratio.
 */
void setup()
{
    Serial.begin(115200);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Scanning 3 channels with OSR_64
    adc1.setupADC(&SPI, 3, MCP356xOversamplingRatio::OSR_64);
}

/**
 * @brief Main program loop
 *
 * Continuously reads data from the ADC when available, applies filtering
 * to each axis, and outputs the filtered readings as CSV data.
 */
void loop()
{
    if (adc1.isr_fired && (2 == adc1.read()))
    {
        // Acquire the 3D force data
        Reading3D forceReadings = threeAxisLoadCell.digitalReading3D();

        // Filter the readings
        float filteredX = filterX.addValue(forceReadings.x);
        float filteredY = filterY.addValue(forceReadings.y);
        float filteredZ = filterZ.addValue(forceReadings.z);

        // Print the current time in microseconds and the filtered readings for each axis on a single line
        Serial.print(micros());
        Serial.print(",");
        Serial.print(filteredX, 1); // One decimal place for X-Axis
        Serial.print(",");
        Serial.print(filteredY, 1); // One decimal place for Y-Axis
        Serial.print(",");
        Serial.println(filteredZ, 1); // One decimal place for Z-Axis
    }
}