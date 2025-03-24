// This code captures data on three axis x,y and z axis and smooths it to take readings for the calibration of the 3 axis load cell

#include "MCP356x.h"
#include "MCP356x3axis.h"
#include "MCP356xScale.h"

/** @brief Pins Configuration */
const uint8_t SDI_PIN = 11;
const uint8_t SDO_PIN = 12;
const uint8_t SCK_PIN = 13;

// Pins for ADC1, ADC2 and ADC3
const uint8_t MCP_ADC1_CS_PIN  = 2;
const uint8_t MCP_ADC1_IRQ_PIN = 3;
const uint8_t MCP_ADC2_CS_PIN  = 5;
const uint8_t MCP_ADC2_IRQ_PIN = 4;
const uint8_t MCP_ADC3_CS_PIN  = 7;
const uint8_t MCP_ADC3_IRQ_PIN = 6;

// Configuration for ADC1, ADC2 & ADC3
MCP356xConfig adc1Config = {MCP_ADC1_IRQ_PIN, MCP_ADC1_CS_PIN};
MCP356x       adc1(adc1Config);

// Three-axis Load Cell Instances
// Load cell one (using channels A (x), B(y), C(z) from ADC1)
MCP356xScale scaleA1(&adc1, MCP356xChannel::DIFF_A);
MCP356xScale scaleB1(&adc1, MCP356xChannel::DIFF_B);
MCP356xScale scaleC1(&adc1, MCP356xChannel::DIFF_C);

// Create ThreeAxisLoadCell instances, x, y & z
MCP356x3axis threeAxisLoadCell(&scaleA1, &scaleB1, &scaleC1);

// Filter class for moving average
class Filter {
private:
    float *buffer;
    int index;
    int size;
    float sum;

public:
    Filter(int _size) : index(0), size(_size), sum(0) {
        buffer = new float[size];
        for (int i = 0; i < size; ++i) {
            buffer[i] = 0.0;
        }
    }

    ~Filter() {
        delete[] buffer;
    }

    // Adds a new value to the buffer and computes the moving average
    float addValue(float value) {
        sum -= buffer[index];
        buffer[index] = value;
        sum += value;
        index = (index + 1) % size;
        return sum / size;
    }
};

// Instantiate filter with desired buffer size (e.g., 10 samples)
Filter filterX(10000);
Filter filterY(10000);
Filter filterZ(10000);

void setup() {
    Serial.begin(115200);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    //scanning 4 channels with OSR_64
    adc1.setupADC(&SPI, 3, MCP356xOversamplingRatio::OSR_64);
}

void loop() {
    if (adc1.isr_fired && (2 == adc1.read())) {
        // Acquire the 3D force data
        Reading3D forceReadings = threeAxisLoadCell.digitalReading3D();

        // Filter the readings
        float filteredX = filterX.addValue(forceReadings.x);
        float filteredY = filterY.addValue(forceReadings.y);
        float filteredZ = filterZ.addValue(forceReadings.z);

        // Print the current time in microseconds and the filtered readings for each axis on a single line
        Serial.print(micros());
        Serial.print(",");
        Serial.print(filteredX, 1);  // One decimal place for X-Axis
        Serial.print(",");
        Serial.print(filteredY, 1);  // One decimal place for Y-Axis
        Serial.print(",");
        Serial.println(filteredZ, 1); // One decimal place for Z-Axis
    }
}
