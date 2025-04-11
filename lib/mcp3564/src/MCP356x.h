/**
 * @file MCP356x.h
 * @brief Class declaration for interfacing with MCP356x family of Analog-to-Digital Converters
 *
 * This header file contains the declarations for the MCP356x class, which provides
 * a complete interface for communicating with and controlling the MCP356x family of
 * high-resolution ADCs. It includes definitions for registers, channels, operation modes,
 * and configuration parameters.
 *
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#ifndef __MCP356x_H__
#define __MCP356x_H__

#include "StringBuilder.h"
#include <SPI.h>
#include <cmath>
#include <inttypes.h>
#include <stdarg.h>
#include <stdint.h>

// Maximum number of MCP356x instances that can be used simultaneously
#define MCP356X_MAX_INSTANCES 3

/**
 * @enum MCP356xRegister
 * @brief Register addresses for the MCP356x ADC
 *
 * These enum values translate directly to register addresses.
 */
enum class MCP356xRegister : uint8_t
{
    ADCDATA = 0x00,   // ADC Data Register (read-only)
    CONFIG0 = 0x01,   // 8-bit Configuration Register 0
    CONFIG1 = 0x02,   // 8-bit Configuration Register 1
    CONFIG2 = 0x03,   // 8-bit Configuration Register 2
    CONFIG3 = 0x04,   // 8-bit Configuration Register 3
    IRQ = 0x05,       // 8-bit Interrupt Register
    MUX = 0x06,       // 8-bit Multiplexer Register
    SCAN = 0x07,      // 24-bit Scan Register
    TIMER = 0x08,     // 24-bit Timer Register
    OFFSETCAL = 0x09, // 24-bit Offset Calibration Register
    GAINCAL = 0x0A,   // 24-bit Gain Calibration Register
    RESERVED0 = 0x0B, // 24-bit Reserved Register 0
    RESERVED1 = 0x0C, // 8-bit Reserved Register 1
    LOCK = 0x0D,      // 8-bit Lock Register
    RESERVED2 = 0x0E, // 16-bit Reserved Register 2
    CRCCFG = 0x0F     // 16-bit CRC Configuration Register (read-only)
};

/**
 * @enum MCP356xChannel
 * @brief ADC channel definitions for the MCP356x
 *
 * The ADC is divided into 16 logical channels, including single-ended,
 * differential, temperature, and internal references.
 */
enum class MCP356xChannel : uint8_t
{
    SE_0 = 0x00,   // Single-ended channel 0
    SE_1 = 0x01,   // Single-ended channel 1
    SE_2 = 0x02,   // Single-ended channel 2
    SE_3 = 0x03,   // Single-ended channel 3
    SE_4 = 0x04,   // Single-ended channel 4
    SE_5 = 0x05,   // Single-ended channel 5
    SE_6 = 0x06,   // Single-ended channel 6
    SE_7 = 0x07,   // Single-ended channel 7
    DIFF_A = 0x08, // Differential channel A
    DIFF_B = 0x09, // Differential channel B
    DIFF_C = 0x0A, // Differential channel C
    DIFF_D = 0x0B, // Differential channel D
    TEMP = 0x0C,   // Internal temperature sensor
    AVDD = 0x0D,   // Analog supply voltage
    VCM = 0x0E,    // Common-mode voltage
    OFFSET = 0x0F  // Offset measurement
};

/**
 * @enum MCP356xMode
 * @brief Conversion modes for the MCP356x ADC
 */
enum class MCP356xMode : uint8_t
{
    ONESHOT_SHUTDOWN = 0, // One-shot conversion then shutdown
    ONESHOT_STANDBY = 2,  // One-shot conversion then standby
    CONTINUOUS = 3        // Continuous conversion mode
};

/**
 * @enum MCP356xGain
 * @brief Gain settings for the MCP356x ADC
 *
 * Enum values convert directly into register values.
 */
enum class MCP356xGain : uint8_t
{
    GAIN_ONETHIRD = 0, // 1/3x gain
    GAIN_1 = 1,        // 1x gain
    GAIN_2 = 2,        // 2x gain
    GAIN_4 = 3,        // 4x gain
    GAIN_8 = 4,        // 8x gain
    GAIN_16 = 5,       // 16x gain
    GAIN_32 = 6,       // 32x gain
    GAIN_64 = 7        // 64x gain
};

/**
 * @enum MCP356xBiasCurrent
 * @brief Bias current settings for the MCP356x ADC
 *
 * Enum values convert directly into register values.
 */
enum class MCP356xBiasCurrent : uint8_t
{
    HALF = 0,      // 0.5x bias current
    TWOTHIRDS = 1, // 2/3x bias current
    ONE = 2,       // 1x bias current
    DOUBLE = 3     // 2x bias current
};

/**
 * @enum MCP356xADCMode
 * @brief Operating mode settings for the MCP356x ADC
 */
enum class MCP356xADCMode : uint8_t
{
    ADC_SHUTDOWN_MODE = 1,  // ADC shutdown mode
    ADC_STANDBY_MODE = 2,   // ADC standby mode
    ADC_CONVERSION_MODE = 3 // ADC active conversion mode
};

/**
 * @enum MCP356xBiasBoost
 * @brief Bias boost settings for the MCP356x ADC
 *
 * Enum value converts directly into register value.
 */
enum class MCP356xBiasBoost : uint8_t
{
    HALF = 0,      // 0.5x bias boost
    TWOTHIRDS = 1, // 2/3x bias boost
    ONE = 2,       // 1x bias boost
    DOUBLE = 3     // 2x bias boost
};

/**
 * @enum MCP356xOversamplingRatio
 * @brief Oversampling ratio settings for the MCP356x ADC
 *
 * Enum values convert directly into register values.
 */
enum class MCP356xOversamplingRatio : uint8_t
{
    OSR_32 = 0x00,    // 32x oversampling ratio
    OSR_64 = 0x01,    // 64x oversampling ratio
    OSR_128 = 0x02,   // 128x oversampling ratio
    OSR_256 = 0x03,   // 256x oversampling ratio (default on reset)
    OSR_512 = 0x04,   // 512x oversampling ratio
    OSR_1024 = 0x05,  // 1024x oversampling ratio
    OSR_2048 = 0x06,  // 2048x oversampling ratio
    OSR_4096 = 0x07,  // 4096x oversampling ratio
    OSR_8192 = 0x08,  // 8192x oversampling ratio
    OSR_16384 = 0x09, // 16384x oversampling ratio
    OSR_20480 = 0x0A, // 20480x oversampling ratio
    OSR_24576 = 0x0B, // 24576x oversampling ratio
    OSR_40960 = 0x0C, // 40960x oversampling ratio
    OSR_49152 = 0x0D, // 49152x oversampling ratio
    OSR_81920 = 0x0E, // 81920x oversampling ratio
    OSR_98304 = 0x0F  // 98304x oversampling ratio
};

/**
 * @enum MCP356xAMCLKPrescaler
 * @brief Clock prescaler options for the MCP356x ADC
 */
enum class MCP356xAMCLKPrescaler : uint8_t
{
    OVER_1 = 0, // Clock divide by 1
    OVER_2 = 1, // Clock divide by 2
    OVER_4 = 2, // Clock divide by 4
    OVER_8 = 3  // Clock divide by 8
};

/**
 * @enum MCP356xBurnoutCurrentSources
 * @brief Burnout current source configuration for the MCP356x ADC
 */
enum class MCP356xBurnoutCurrentSources : uint8_t
{
    DISABLED = 0,       // Burnout current sources disabled
    CURRENT_0_9_UA = 1, // 0.9µA burnout current
    CURRENT_3_7_UA = 2, // 3.7µA burnout current
    CURRENT_15_UA = 3   // 15µA burnout current
};

/**
 * @struct MCP356xConfig
 * @brief Configuration structure for MCP356x ADC initialization
 */
struct MCP356xConfig
{
    uint8_t irq_pin;                                           // Interrupt pin
    uint8_t cs_pin;                                            // Chip select pin
    uint8_t mclk_pin = 0;                                      // Optional: for external clock
    uint8_t addr = 0x01;                                       // Default address
    SPIClass *spiInterface;                                    // SPI interface
    int numChannels;                                           // Number of channels to be used
    MCP356xOversamplingRatio osr;                              // Oversampling ratio
    MCP356xGain gain = MCP356xGain::GAIN_1;                    // ADC gain setting
    MCP356xADCMode mode = MCP356xADCMode::ADC_CONVERSION_MODE; // ADC mode
};

// Class flags that control MCP356x behavior
// NOTE: MCP356X_FLAG_USE_INTERNAL_CLK takes priority over MCP356X_FLAG_GENERATE_MCLK
#define MCP356X_FLAG_DEVICE_PRESENT 0x00000001   // Part is likely an MCP356x
#define MCP356X_FLAG_PINS_CONFIGURED 0x00000002  // Low-level pin setup is complete
#define MCP356X_FLAG_INITIALIZED 0x00000004      // Registers are initialized
#define MCP356X_FLAG_CALIBRATED 0x00000008       // ADC is calibrated
#define MCP356X_FLAG_VREF_DECLARED 0x00000010    // The application has given us Vref
#define MCP356X_FLAG_CRC_ERROR 0x00000020        // The chip has reported a CRC error
#define MCP356X_FLAG_USE_INTERNAL_CLK 0x00000040 // The chip should use its internal oscillator
#define MCP356X_FLAG_MCLK_RUNNING 0x00000080     // MCLK is running
#define MCP356X_FLAG_SAMPLED_AVDD 0x00000100     // This calibration-related channel was sampled
#define MCP356X_FLAG_SAMPLED_VCM 0x00000200      // This calibration-related channel was sampled
#define MCP356X_FLAG_SAMPLED_OFFSET 0x00000400   // This calibration-related channel was sampled
#define MCP356X_FLAG_3RD_ORDER_TEMP 0x00000800   // Spend CPU to make temperature conversion more accurate
#define MCP356X_FLAG_GENERATE_MCLK 0x00001000    // MCU is to generate the clock
#define MCP356X_FLAG_HAS_INTRNL_VREF 0x00004000  // This part was found to support an internal Vref
#define MCP356X_FLAG_USE_INTRNL_VREF 0x00008000  // Internal Vref should be enabled

// Bits to preserve through reset
#define MCP356X_FLAG_RESET_MASK                                   \
    (MCP356X_FLAG_DEVICE_PRESENT | MCP356X_FLAG_PINS_CONFIGURED | \
     MCP356X_FLAG_VREF_DECLARED | MCP356X_FLAG_USE_INTERNAL_CLK | \
     MCP356X_FLAG_3RD_ORDER_TEMP | MCP356X_FLAG_GENERATE_MCLK |   \
     MCP356X_FLAG_HAS_INTRNL_VREF | MCP356X_FLAG_USE_INTRNL_VREF)

// Bits indicating calibration steps
#define MCP356X_FLAG_ALL_CAL_MASK                           \
    (MCP356X_FLAG_SAMPLED_AVDD | MCP356X_FLAG_SAMPLED_VCM | \
     MCP356X_FLAG_SAMPLED_OFFSET)

/**
 * @class MCP356x
 * @brief Class for interfacing with MCP356x family of ADCs
 *
 * This class provides a complete interface for communicating with and controlling
 * the MCP356x family of high-resolution ADCs. It handles initialization, configuration,
 * data acquisition, and calibration.
 */
class MCP356x
{
public:
    // Flag indicating an interrupt has been triggered
    volatile bool isr_fired = false;

    // Constructor and destructor
    MCP356x(const MCP356xConfig &config);
    virtual ~MCP356x();

    // Initialization and setup methods
    virtual int8_t reset();
    virtual int8_t init(SPIClass *);
    inline int8_t init() { return init(_bus); }

    // ADC operations methods
    bool scanComplete();
    int updatedReadings();
    int8_t read();
    int32_t value(MCP356xChannel);
    double valueAsVoltage(MCP356xChannel);

    // Calibration and configuration methods
    void setSpecificChannels(int numChannels);
    int8_t setOption(uint32_t);
    int8_t setOffsetCalibration(int32_t);
    int8_t setGainCalibration(int32_t);
    int8_t setGain(MCP356xGain);
    int8_t setConversionMode(MCP356xMode);
    int8_t setBiasCurrent(MCP356xBiasCurrent);
    int8_t setScanChannels(int count, ...);
    int8_t setReferenceRange(float plus, float minus);
    int8_t setADCMode(MCP356xADCMode);
    int8_t setAMCLKPrescaler(MCP356xAMCLKPrescaler);
    int8_t setBurnoutCurrentSources(MCP356xBurnoutCurrentSources);
    int8_t setOversamplingRatio(MCP356xOversamplingRatio);
    int8_t setScanTimer(uint32_t timerValue);
    inline void setCircuitSettleTime(uint32_t us) { _circuit_settle_us = us; };

    // Getter methods
    MCP356xGain getGain();
    float getTemperature();
    inline double getMCLKFrequency() { return _mclk_freq; };
    inline uint8_t getIRQPin() { return _IRQ_PIN; };
    inline uint16_t getSampleRate() { return _reads_per_second; }
    inline uint32_t getSettlingTime() { return _settling_us; };
    inline uint32_t getCircuitSettleTime() { return _circuit_settle_us; };
    MCP356xOversamplingRatio getOversamplingRatio();

    // Burnout current source methods
    void enableBurnoutCurrentSources();
    void disableBurnoutCurrentSources();
    void discardUnsettledSamples();

    // Status checking methods
    bool isrFired() { return isr_fired; };
    bool usingInternalVref();
    inline bool adcFound() { return _mcp356x_flag(MCP356X_FLAG_DEVICE_PRESENT); }
    inline bool adcConfigured() { return _mcp356x_flag(MCP356X_FLAG_INITIALIZED); }
    inline bool adcCalibrated() { return _mcp356x_flag(MCP356X_FLAG_CALIBRATED); }
    inline bool hasInternalVref() { return _mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF); }
    int8_t refresh();
    int8_t useInternalVref(bool);

    // Debugging and output methods
    void printPins(StringBuilder *);
    void printRegs(StringBuilder *);
    void printTimings(StringBuilder *);
    void printData(StringBuilder *);
    void printChannel(MCP356xChannel, StringBuilder *);
    void printChannelValues(StringBuilder *, bool);
    void regToBinaryString(uint32_t, StringBuilder *, int bits = 32);

    // Inline methods for retrieving ADC states and counters
    inline uint32_t lastRead() { return _micros_last_read; };
    inline uint32_t readCount() { return _read_count; };
    inline void resetReadCount() { _read_count = 0; };

protected:
    int8_t _clear_registers();

private:
    // Pin assignments
    const uint8_t _IRQ_PIN;
    const uint8_t _CS_PIN;
    const uint8_t _MCLK_PIN;
    const uint8_t _DEV_ADDR;

    // Bus and frequency variables
    SPIClass *_bus = nullptr;
    double _mclk_freq = 0.0;
    double _dmclk_freq = 0.0;
    double _drclk_freq = 0.0;
    float _vref_plus = 3.3;
    float _vref_minus = 0.0;

    // Registers and channel values
    uint32_t _reg_shadows[16];
    int32_t _channel_vals[16];

    // Flags and settings
    uint32_t _flags = 0;
    uint32_t _channel_flags = 0;
    uint32_t _discard_until_micros = 0;
    uint32_t _circuit_settle_us = 0;
    uint32_t _settling_us = 0;

    // Read tracking variables
    uint32_t _read_count = 0;
    uint32_t _read_accumulator = 0;
    uint16_t _reads_per_second = 0;
    uint32_t _micros_last_read = 0;
    uint32_t _micros_last_window = 0;

    // Channel backup
    uint32_t _channel_backup = 0;
    uint8_t _slot_number = 0;

    // Internal helper functions
    int8_t _post_reset_fxn();
    int8_t _proc_irq_register();
    int8_t _ll_pin_init();
    uint8_t _get_reg_addr(MCP356xRegister);
    int8_t _send_fast_command(uint8_t cmd);

    uint8_t _channel_count();
    int8_t _set_scan_channels(uint32_t);
    int8_t _calibrate_offset();
    int8_t _mark_calibrated();

    bool _mclk_in_bounds();
    int8_t _detect_adc_clock();
    double _calculate_input_clock(unsigned long);
    int8_t _recalculate_clk_tree();
    int8_t _recalculate_settling_time();

    int8_t _write_register(MCP356xRegister r, uint32_t val);
    int8_t _read_register(MCP356xRegister r);

    uint8_t _output_coding_bytes();
    int8_t _normalize_data_register();
    float _gain_value();

    inline bool _vref_declared() { return _mcp356x_flag(MCP356X_FLAG_VREF_DECLARED); };
    inline bool _scan_covers_channel(MCP356xChannel c) { return (0x01 & (_reg_shadows[(uint8_t)MCP356xRegister::SCAN] >> ((uint8_t)c))); }

    // Flag manipulation inlines
    inline uint32_t _mcp356x_flags() { return _flags; };
    inline bool _mcp356x_flag(uint32_t _flag) { return (_flags & _flag); };
    inline void _mcp356x_flip_flag(uint32_t _flag) { _flags ^= _flag; };
    inline void _mcp356x_clear_flag(uint32_t _flag) { _flags &= ~_flag; };
    inline void _mcp356x_set_flag(uint32_t _flag) { _flags |= _flag; };
    inline void _mcp356x_set_flag(uint32_t _flag, bool nu)
    {
        if (nu)
            _flags |= _flag;
        else
            _flags &= ~_flag;
    };

    // Flag manipulation inlines for individual channels
    inline void _channel_clear_new_flag(MCP356xChannel c) { _channel_flags &= ~(1 << (uint8_t)c); };
    inline void _channel_set_new_flag(MCP356xChannel c) { _channel_flags |= (1 << (uint8_t)c); };
    inline bool _channel_has_new_value(MCP356xChannel c) { return (_channel_flags & (1 << (uint8_t)c)); };
    inline void _channel_clear_ovr_flag(MCP356xChannel c) { _channel_flags &= ~(0x00010000 << (uint8_t)c); };
    inline void _channel_set_ovr_flag(MCP356xChannel c) { _channel_flags |= (0x00010000 << (uint8_t)c); };
    inline void _channel_set_ovr_flag(MCP356xChannel c, bool nu)
    {
        _channel_flags = (nu) ? (_channel_flags | (0x00010000 << (uint8_t)c))
                              : (_channel_flags & ~(0x00010000 << (uint8_t)c));
    };
    inline bool _channel_over_range(MCP356xChannel c)
    {
        return (_channel_flags & (0x00010000 << (uint8_t)c));
    };
};

#endif // __MCP356x_H__