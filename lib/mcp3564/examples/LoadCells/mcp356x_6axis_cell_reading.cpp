/**
 * @file mcp356x_6axis_cell_reading.cpp
 * @brief Reads data from a 6-axis load cell system using four 3-axis load cells.
 *
 * This program demonstrates how to set up a 6-axis force/torque sensor using four 3-axis
 * load cells connected to MCP356x ADCs. It initializes the load cells, applies calibration,
 * and reads the combined force data in all six degrees of freedom.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to all MCP356x SDI pins
 * - SDO_PIN (12): Connect to all MCP356x SDO pins
 * - SCK_PIN (13): Connect to all MCP356x SCK pins
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

// Configuration for ADC1, ADC2, ADC3
MCP356xConfig adc1Config = {MCP_ADC1_IRQ_PIN, MCP_ADC1_CS_PIN};
MCP356xConfig adc2Config = {MCP_ADC2_IRQ_PIN, MCP_ADC2_CS_PIN};
MCP356xConfig adc3Config = {MCP_ADC3_IRQ_PIN, MCP_ADC3_CS_PIN};
MCP356x adc1(adc1Config);
MCP356x adc2(adc2Config);
MCP356x adc3(adc3Config);

// Three-axis Load Cell Instances for each load cell
MCP356xScale scaleX1(&adc1, MCP356xChannel::DIFF_A);
MCP356xScale scaleY1(&adc1, MCP356xChannel::DIFF_B);
MCP356xScale scaleZ1(&adc1, MCP356xChannel::DIFF_C);

MCP356xScale scaleX2(&adc1, MCP356xChannel::DIFF_D);
MCP356xScale scaleY2(&adc2, MCP356xChannel::DIFF_A);
MCP356xScale scaleZ2(&adc2, MCP356xChannel::DIFF_B);

MCP356xScale scaleX3(&adc2, MCP356xChannel::DIFF_C);
MCP356xScale scaleY3(&adc2, MCP356xChannel::DIFF_D);
MCP356xScale scaleZ3(&adc3, MCP356xChannel::DIFF_A);

MCP356xScale scaleX4(&adc3, MCP356xChannel::DIFF_B);
MCP356xScale scaleY4(&adc3, MCP356xChannel::DIFF_C);
MCP356xScale scaleZ4(&adc3, MCP356xChannel::DIFF_D);

// Create Four ThreeAxisLoadCell instances
MCP356x3axis threeAxisLoadCell1(&scaleX1, &scaleY1, &scaleZ1);
MCP356x3axis threeAxisLoadCell2(&scaleX2, &scaleY2, &scaleZ2);
MCP356x3axis threeAxisLoadCell3(&scaleX3, &scaleY3, &scaleZ3);
MCP356x3axis threeAxisLoadCell4(&scaleX4, &scaleY4, &scaleZ4);

// Calibration coefficients for each load cell
float coeffsX1[3] = {-282.891559, -0.000613937252, -4.77986709e-12};
float coeffsY1[3] = {359.254151, 0.000595177607, 1.06318667e-11};
float coeffsZ1[3] = {-572.213742, 0.000624478626, -9.49390560e-12};

float coeffsX2[3] = {-151.734577, 0.00060020515, -7.21396710e-13};
float coeffsY2[3] = {321.773149, 0.000594105046, -7.52617778e-12};
float coeffsZ2[3] = {-688.373969, 0.000596410043, 4.76943428e-12};

float coeffsX3[3] = {492.449655, -0.000600041428, -5.84203806e-12};
float coeffsY3[3] = {406.557270, 0.000587369689, -3.80944390e-12};
float coeffsZ3[3] = {-372.228370, -0.000561156115, 4.55678020e-12};

float coeffsX4[3] = {-19.7361969, -0.000604058912, -8.46360169e-13};
float coeffsY4[3] = {535.128578, 0.000598242204, 4.51941740e-12};
float coeffsZ4[3] = {-263.695282, 0.000599632857, 5.86901761e-12};

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, SPI interface, and the MCP356x ADCs.
 * Sets up calibration coefficients for all load cells and tares them.
 */
void setup()
{
    Serial.begin(115200);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Initialize ADCs and load cells
    adc1.setupADC(&SPI, 4, MCP356xOversamplingRatio::OSR_32);
    adc2.setupADC(&SPI, 4, MCP356xOversamplingRatio::OSR_32);
    adc3.setupADC(&SPI, 4, MCP356xOversamplingRatio::OSR_32);

    // Apply calibration coefficients to load cells
    threeAxisLoadCell1.setCalibrationCoefficients(coeffsX1, coeffsY1, coeffsZ1);
    threeAxisLoadCell2.setCalibrationCoefficients(coeffsX2, coeffsY2, coeffsZ2);
    threeAxisLoadCell3.setCalibrationCoefficients(coeffsX3, coeffsY3, coeffsZ3);
    threeAxisLoadCell4.setCalibrationCoefficients(coeffsX4, coeffsY4, coeffsZ4);

    // Tare the scales for all load cells
    Serial.println("Calibration start");
    threeAxisLoadCell1.tareScales();
    Serial.println("Calibration start2");
    threeAxisLoadCell2.tareScales();
    Serial.println("Calibration start3");
    threeAxisLoadCell3.tareScales();
    Serial.println("Calibration start4");
    threeAxisLoadCell4.tareScales();

    Serial.println("6-axis system initialized and calibrated.");
}

/**
 * @brief Main program loop
 *
 * Continuously reads force data from the first 3-axis load cell when available
 * and outputs the 3D force data along with timestamps.
 */
void loop()
{
    if (adc1.isr_fired && (2 == adc1.read()))
    {

        // Acquire the 3D force data in grams force
        Force3D forceReadings = threeAxisLoadCell1.ForceData3D();

        // Format and output the data as CSV
        char buffer[60];
        snprintf(buffer, sizeof(buffer), "%lu,%.1f,%.1f,%.1f",
                 micros(), forceReadings.x, forceReadings.y, forceReadings.z);
        Serial.println(buffer);
    }
}