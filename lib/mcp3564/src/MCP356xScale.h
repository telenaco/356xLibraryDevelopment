/**
 * @file MCP356xScale.h
 * @brief MCP356xScale class for managing multiple load cells connected to MCP356x ADCs.
 * 
 * This class provides a higher level of abstraction for interfacing with load cells connected
 * to MCP356x ADCs. It handles calibration, conversion, and reading from multiple scales.
 * 
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#ifndef MCP356X_SCALE_H
#define MCP356X_SCALE_H

#include "MCP356x.h"

/**
 * @class MCP356xScale
 * @brief Class for managing multiple load cells connected to MCP356x ADCs.
 * 
 * This class extends the functionality of the base MCP356x class to provide
 * features specific to load cell applications, including calibration, scaling,
 * and taring operations.
 */
class MCP356xScale {
public:
    /**
     * @enum ConversionMode
     * @brief Defines the different conversion modes for interpreting ADC readings.
     */
    enum class ConversionMode : uint8_t {
        UNDEFINED,      ///< Undefined conversion mode
        DIGITAL,        ///< Raw digital reading
        SINGLE_VALUE,   ///< Single value scale factor conversion
        LINEAR,         ///< Linear equation conversion (ax + b)
        POLYNOMIAL      ///< Polynomial equation conversion (ax² + bx + c)
    };

    /**
     * @brief Constructor for the MCP356xScale class.
     * 
     * Initializes the MCP356xScale with the specified number of scales and pin configurations.
     * 
     * @param totalScales Total number of scales (load cells) to manage
     * @param sckPin SPI clock pin
     * @param sdoPin SPI data output pin
     * @param sdiPin SPI data input pin
     * @param irqPin1 Interrupt pin for first ADC
     * @param csPin1 Chip select pin for first ADC
     * @param irqPin2 Interrupt pin for second ADC (optional)
     * @param csPin2 Chip select pin for second ADC (optional)
     * @param irqPin3 Interrupt pin for third ADC (optional)
     * @param csPin3 Chip select pin for third ADC (optional)
     */
    MCP356xScale(int totalScales, int sckPin, int sdoPin, int sdiPin,
                 int irqPin1, int csPin1, int irqPin2 = -1, int csPin2 = -1, 
                 int irqPin3 = -1, int csPin3 = -1);
    
    /**
     * @brief Destructor for the MCP356xScale class.
     * 
     * Cleans up resources used by the MCP356xScale instance.
     */
    ~MCP356xScale();

    /**
     * @brief Sets up channel mappings for the scales.
     * 
     * Automatically maps ADC channels to the scales based on the total number of scales.
     */
    void setupChannelMappings();

    /**
     * @brief Updates ADC readings from all connected ADCs.
     * 
     * @return int Returns 1 if all ADCs have updated data, 0 otherwise.
     */
    int updatedAdcReadings();

    /**
     * @brief Prints debug information about all ADCs.
     * 
     * Outputs detailed debug information about all initialized ADCs to the serial port.
     */
    void printADCsDebug();
    
    /**
     * @brief Prints raw ADC channel values.
     * 
     * Outputs raw ADC values and voltage readings for all channels to the serial port.
     */
    void printAdcRawChannels();
    
    /**
     * @brief Prints the update rate for each ADC in readings per second.
     * 
     * Outputs the number of readings per second for each initialized ADC to the serial port.
     */
    void printReadsPerSecond();
    
    /**
     * @brief Prints the calibration parameters for each channel.
     * 
     * Outputs detailed calibration parameters for all scales to the serial port.
     */
    void printChannelParameters();

    /**
     * @brief Sets the scale to use digital reading mode.
     * 
     * Configures the specified scale to use raw digital readings without calibration.
     * 
     * @param scaleIndex Index of the scale to configure
     */
    void setDigitalRead(int scaleIndex);
    
    /**
     * @brief Sets a simple scale factor for the specified scale.
     * 
     * Configures the scale to use a single scale factor for conversion.
     * 
     * @param scaleIndex Index of the scale to configure
     * @param scale Scale factor to apply to raw readings
     */
    void setScaleFactor(int scaleIndex, float scale);
    
    /**
     * @brief Sets linear calibration parameters for the specified scale.
     * 
     * Configures the scale to use a linear equation (y = ax + b) for conversion.
     * 
     * @param scaleIndex Index of the scale to configure
     * @param slope Slope (a) of the linear equation
     * @param intercept Y-intercept (b) of the linear equation
     */
    void setLinearCalibration(int scaleIndex, float slope, float intercept);
    
    /**
     * @brief Sets polynomial calibration parameters for the specified scale.
     * 
     * Configures the scale to use a polynomial equation (y = ax² + bx + c) for conversion.
     * 
     * @param scaleIndex Index of the scale to configure
     * @param a Coefficient of x²
     * @param b Coefficient of x
     * @param c Constant term
     */
    void setPolynomialCalibration(int scaleIndex, float a, float b, float c);
    
    /**
     * @brief Performs a tare operation on the specified scale.
     * 
     * Sets the current reading as the zero point for the scale.
     * 
     * @param scaleIndex Index of the scale to tare
     * @param times Number of readings to average for the tare operation (optional)
     */
    void tare(int scaleIndex, int times = 1000);

    /**
     * @brief Gets the calibrated reading from the specified scale.
     * 
     * Returns the reading after applying the configured calibration.
     * 
     * @param scaleIndex Index of the scale to read
     * @return float Calibrated reading in the configured units
     */
    float getReading(int scaleIndex);
    
    /**
     * @brief Gets the force reading from the specified scale in Newtons.
     * 
     * Converts the calibrated reading to Newtons using gravitational acceleration.
     * 
     * @param scaleIndex Index of the scale to read
     * @return float Force reading in Newtons
     */
    float getForce(int scaleIndex);
    
    /**
     * @brief Gets the raw uncalibrated value from the specified scale.
     * 
     * @param scaleIndex Index of the scale to read
     * @return int32_t Raw ADC reading
     */
    int32_t getRawValue(int scaleIndex);
    
    /**
     * @brief Gets the read count for the specified ADC.
     * 
     * @param adcIndex Index of the ADC
     * @return uint32_t Number of readings taken by the ADC
     */
    uint32_t getReadCount(int adcIndex);
    
    /**
     * @brief Gets the readings per second for the specified ADC.
     * 
     * @param adcIndex Index of the ADC
     * @return uint16_t Number of readings per second
     */
    uint16_t getReadsPerSecond(int adcIndex);

    /**
     * @brief Default configuration for MCP356x ADCs.
     * 
     * Static configuration structure used as a template for creating ADC instances.
     */
    static MCP356xConfig defaultConfig;

private:
    // Constants
    static const int MAX_SCALES = 12;     ///< Maximum number of scales supported
    static const int MAX_ADCS = 3;        ///< Maximum number of ADCs supported
    static const int CHANNELS_PER_ADC = 4; ///< Number of channels per ADC

    // ADC and scale configuration
    uint8_t _newDataFlags = 0;           ///< Flags indicating new data availability
    int     _totalScales;                ///< Total number of scales being managed
    int     _sckPin;                     ///< SPI clock pin
    int     _sdoPin;                     ///< SPI data output pin
    int     _sdiPin;                     ///< SPI data input pin
    MCP356x* _adcs[MAX_ADCS];            ///< Array of MCP356x ADC instances
    uint16_t _readsPerSecond[MAX_ADCS] = { 0 };     ///< Readings per second for each ADC
    uint32_t _readAccumulator[MAX_ADCS] = { 0 };    ///< Read counter for rate calculation
    uint32_t _microsLastWindow[MAX_ADCS] = { 0 };   ///< Timestamp for rate calculation

    /**
     * @struct ScaleConfig
     * @brief Configuration structure for each scale.
     * 
     * Contains all configuration parameters and state for a single scale.
     */
    struct ScaleConfig {
        ConversionMode mode;    ///< Conversion mode for this scale
        float          param1;  ///< First calibration parameter (depends on mode)
        float          param2;  ///< Second calibration parameter (depends on mode)
        float          param3;  ///< Third calibration parameter (depends on mode)
        int32_t        offset;  ///< Tare offset
        int32_t        rawReading; ///< Last raw reading
        int            adcIndex;   ///< Index of the ADC this scale is connected to
        MCP356xChannel channel;    ///< Channel on the ADC this scale is connected to
    };

    ScaleConfig _scaleConfigs[MAX_SCALES]; ///< Configuration for each scale

    // Conversion methods (private)
    /**
     * @brief Converts a raw reading to a digital reading.
     * 
     * @param scaleIndex Index of the scale
     * @return int32_t Digital reading with tare offset applied
     */
    int32_t convertToDigitalRead(int scaleIndex);
    
    /**
     * @brief Converts a raw reading using a single scale factor.
     * 
     * @param scaleIndex Index of the scale
     * @return float Scaled reading
     */
    float convertToSingleValueForce(int scaleIndex);
    
    /**
     * @brief Converts a raw reading using a linear equation.
     * 
     * @param scaleIndex Index of the scale
     * @return float Calibrated reading
     */
    float convertToLinearForce(int scaleIndex);
    
    /**
     * @brief Converts a raw reading using a polynomial equation.
     * 
     * @param scaleIndex Index of the scale
     * @return float Calibrated reading
     */
    float convertToPolynomialForce(int scaleIndex);

    static constexpr float GRAVITATIONAL_ACCELERATION = 9.81f; ///< Standard gravity in m/s²
};

#endif // MCP356X_SCALE_H
