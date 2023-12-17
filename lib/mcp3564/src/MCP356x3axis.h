#ifndef THREE_AXIS_LOAD_CELL_H
#define THREE_AXIS_LOAD_CELL_H

#include "MCP356xScale.h"  // Include the MCP356xScale library

class ThreeAxisLoadCell : public MCP356xScale {

public:

    struct Force3D {
    float x;
    float y;
    float z;
    };

    // Constructor 
    ThreeAxisLoadCell();
    ~ThreeAxisLoadCell();

    // initialization method that can handle both single and dual ADC configurations
    void initSingleADC(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin);
    void initDualADC(uint8_t irq_pin1, uint8_t cs_pin1, uint8_t mclk_pin1,
                     uint8_t irq_pin2, uint8_t cs_pin2, uint8_t mclk_pin2);

        // Method to acquire 3D force data
    Force3D acquireForceData3D();
      // Utility methods to interact with the correct ADC based on channel or criteria
    float readForceFromChannel(MCP356xScale& adc, int channel);
        // Private member variables and helper functions



    // Data Acquisition and Processing
    float getForceValueSingle(int channel);  // Get force value for a single axis
    float getWeightSingle(int channel);      // Get weight measurement for a single axis
    void  setScaleSingle(int channel, float scale);  // Set scale factor for a single axis
    float getScaleSingle(int channel);       // Get scale factor for a single axis
    void  setOffsetSingle(int channel, long offset);  // Set offset for a single axis
    long  getOffsetSingle(int channel);      // Get offset for a single axis

    // Three-axis Force Data
    // Replace the following methods with actual implementations
    float getForceMagnitude3D();             // Calculate force magnitude in 3D
    float getWeight3D();                     // Get weight measurement in 3D


    // Single-axis Methods
    float getForceValueSingle();  // Get force value for a single axis
    float getWeightSingle();  // Get weight measurement for a single axis
    float getUnitsSingle();  // Get unit-specific measurement for a single axis
    void setScaleSingle(float scale);  // Set scale factor for a single axis
    float getScaleSingle();  // Get scale factor for a single axis
    void setOffsetSingle(long offset);  // Set offset for a single axis
    long getOffsetSingle();  // Get offset for a single axis

private:
    MCP356xScale adc1;
    MCP356xScale adc2;
    bool adc1_initialized;
    bool adc2_initialized;

  
};

#endif // THREE_AXIS_LOAD_CELL_H
