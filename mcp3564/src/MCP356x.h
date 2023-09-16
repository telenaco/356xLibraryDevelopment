/**
 *
 *old header
 *
 * @file MCP356x.h
 * @brief Library for Microchip MCP356x 16-bit Analog-to-Digital Converter
 *
 * This library provides an interface to configure and
 * read from the MCP356x family of ADCs.
 */

#ifndef __MCP356x_H__
#define __MCP356x_H__

#include <inttypes.h>
#include <stdint.h>
#include <stdarg.h>
#include <SPI.h>
#include "StringBuilder.h"

/**
 * @enum MCP356xRegister 
 * @brief Registers within the MCP356x device.
 * 
 * These enum values directly correspond to the register addresses within the MCP356x.
 * Reading or writing to these register addresses allows configuring and accessing 
 * the ADC functionality.
*/
enum class MCP356xRegister : uint8_t {
  ADCDATA   = 0x00,  // ADC data register with complex structure. Read-only access.
  CONFIG0   = 0x01,  // Configuration register 0. Read/Write access.
  CONFIG1   = 0x02,  // Configuration register 1. Read/Write access.
  CONFIG2   = 0x03,  // Configuration register 2. Read/Write access.
  CONFIG3   = 0x04,  // Configuration register 3. Read/Write access.
  IRQ       = 0x05,  // Interrupt request register. Read/Write access.
  MUX       = 0x06,  // Multiplexer control register. Read/Write access.
  SCAN      = 0x07,  // Scan mode control register with 24-bit width. Read/Write access.
  TIMER     = 0x08,  // Timer control register with 24-bit width. Read/Write access.
  OFFSETCAL = 0x09,  // Offset calibration register with 24-bit width. Read/Write access.
  GAINCAL   = 0x0A,  // Gain calibration register with 24-bit width. Read/Write access.
  RESERVED0 = 0x0B,  // Reserved register with 24-bit width. Do not use.
  RESERVED1 = 0x0C,  // Reserved register with 8-bit width. Do not use.
  LOCK      = 0x0D,  // Lock control register. Read/Write access.
  RESERVED2 = 0x0E,  // Reserved register with 16-bit width. Do not use.
  CRCCFG    = 0x0F   // CRC configuration register with 16-bit width. Read-only access.
};


/**
 * @enum MCP356xChannel
 * @brief ADC input channels available in the MCP356x devices.
 *
 * This enum defines the possible ADC input channels, including internal channels
 * like the temperature sensor and external channels exposed on device pins.
*/
enum class MCP356xChannel : uint8_t {
  SE_0    = 0x00,  // Single-ended channel 0.
  SE_1    = 0x01,  // Single-ended channel 1.
  SE_2    = 0x02,  // Single-ended channel 2.
  SE_3    = 0x03,  // Single-ended channel 3.
  SE_4    = 0x04,  // Single-ended channel 4.
  SE_5    = 0x05,  // Single-ended channel 5.
  SE_6    = 0x06,  // Single-ended channel 6.
  SE_7    = 0x07,  // Single-ended channel 7.
  DIFF_A  = 0x08,  // Differential channel A.
  DIFF_B  = 0x09,  // Differential channel B.
  DIFF_C  = 0x0A,  // Differential channel C.
  DIFF_D  = 0x0B,  // Differential channel D.
  TEMP    = 0x0C,  // Internal temperature sensor channel.
  AVDD    = 0x0D,  // Channel for AVDD.
  VCM     = 0x0E,  // Channel for VCM.
  OFFSET  = 0x0F   // Channel for OFFSET.
};

/**
 * @enum MCP356xMode 
 * @brief ADC operating modes that control how conversions are triggered.
 *
 * This enum provides ADC modes like one-shot, continuous sampling, standby etc.
 * The mode affects how often conversions occur and how much power is consumed.
*/
enum class MCP356xMode : uint8_t {
  ONESHOT_SHUTDOWN = 0,     // Perform a single conversion and then shut down the ADC.
  ONESHOT_STANDBY  = 2,     // Perform a single conversion and then place the ADC in standby mode.
  CONTINUOUS       = 3      // Continuously perform conversions.
};

/**
 * @enum MCP356xADCMode
 * @brief Overall power states of the MCP356x ADC.
 *  
 * These states control whether the ADC is fully powered down, in a low-power 
 * standby state, or fully powered for active conversions.
*/
enum class MCP356xADCMode : uint8_t {
  ADC_SHUTDOWN_MODE   = 1,  // ADC is shut down to minimize power consumption.
  ADC_STANDBY_MODE    = 2,  // ADC is in a low power standby mode.
  ADC_CONVERSION_MODE = 3   // ADC is actively converting.
};

/**
 * @enum MCP356xGain  
 * @brief Gain settings of the ADC's analog front-end amplifier.
 *
 * Higher gain settings allow resolving smaller signals but limit the maximum
 * measurable input voltage range but reduce the resolution bits. 
*/
enum class MCP356xGain : uint8_t {
  GAIN_ONETHIRD = 0,        // ADC gain set to 1/3.
  GAIN_1 = 1,               // ADC gain set to 1x (no amplification).
  GAIN_2 = 2,               // ADC gain set to 2x.
  GAIN_4 = 3,               // ADC gain set to 4x.
  GAIN_8 = 4,               // ADC gain set to 8x.
  GAIN_16 = 5,              // ADC gain set to 16x.
  GAIN_32 = 6,              // ADC gain set to 32x.
  GAIN_64 = 7               // ADC gain set to 64x.
};

/** 
 * @enum MCP356xBiasCurrent
 * @brief Bias current settings that affect the ADC's speed and accuracy.
 * 
 * Higher bias currents allow faster settling times but increase power consumption.
*/
enum class MCP356xBiasCurrent : uint8_t {
  NONE           = 0,        ///< No bias current.
  NANOAMPS_900   = 1,        ///< Bias current set to 900 nanoamps.
  NANOAMPS_3700  = 2,        ///< Bias current set to 3700 nanoamps.
  NANOAMPS_15000 = 3         ///< Bias current set to 15,000 nanoamps.
};

/**
 * @enum MCP356xBiasBoost
 * @brief ADC bias boost settings to optimize dynamic performance.
 * 
 * Bias boost helps maintain ADC bandwidth and accuracy at higher gain settings.
*/
enum class MCP356xBiasBoost : uint8_t {
  HALF = 0,                 // Bias boost set to half of the nominal value.
  TWOTHIRDS = 1,            // Bias boost set to two-thirds of the nominal value.
  ONE = 2,                  // Bias boost set to the nominal value.
  DOUBLE = 3                // Bias boost set to double the nominal value.
};

/**
 * @enum MCP356xOversamplingRatio
 * @brief Oversampling ratios that set the ADC resolution.
 *
 * Higher oversampling ratios yield higher resolution but reduce the output 
 * data rate and increase conversion time. This affects conversion speed, noise levels, etc.
*/
enum class MCP356xOversamplingRatio : uint8_t {
  OSR_32 = 0x00,        // Oversampling ratio of 32.
  OSR_64 = 0x01,        // Oversampling ratio of 64.
  OSR_128 = 0x02,       // Oversampling ratio of 128.
  OSR_256 = 0x03,       // Oversampling ratio of 256 (default on reset).
  OSR_512 = 0x04,
  OSR_1024 = 0x05,
  OSR_2048 = 0x06,
  OSR_4096 = 0x07,
  OSR_8192 = 0x08,
  OSR_16384 = 0x09,
  OSR_20480 = 0x0A,
  OSR_24576 = 0x0B,
  OSR_40960 = 0x0C,
  OSR_49152 = 0x0D,
  OSR_81920 = 0x0E,
  OSR_98304 = 0x0F
};

/**
 * @enum MCP356xAMCLKPrescaler
 * @brief Settings for prescaling the ADC's master clock input.
 * 
 * The prescaler allows optimizing the master clock frequency for desired
 * conversion speeds, resolution, power consumption, etc. 
*/
enum class MCP356xAMCLKPrescaler : uint8_t {
  OVER_1 = 0,           // No prescaling. Use the main clock as-is.
  OVER_2 = 1,           // Divide the main clock by 2.
  OVER_4 = 2,           // Divide the main clock by 4.
  OVER_8 = 3            // Divide the main clock by 8.
};


/**
* @brief Class flags defining various states and configurations for the MCP356x ADC.
*
* These flags provide insight and control over the ADC's behavior, initialization, calibration, and more.
* Note: The MCP356X_FLAG_USE_INTERNAL_CLK flag takes precedence over MCP356X_FLAG_GENERATE_MCLK to avoid
* potential pin contention. If both flags are set, the MCLK pin, if provided, will be configured as an input,
* and the directive to generate a clock on that pin will be ignored.
*/
#define MCP356X_FLAG_DEVICE_PRESENT   0x00000001  // Indicates if an MCP356x device is present.
#define MCP356X_FLAG_PINS_CONFIGURED  0x00000002  // Indicates if the pins have been configured.
#define MCP356X_FLAG_INITIALIZED      0x00000004  // Indicates if the ADC's registers are initialized.
#define MCP356X_FLAG_CALIBRATED       0x00000008  // Indicates if the ADC has been calibrated.
#define MCP356X_FLAG_VREF_DECLARED    0x00000010  // Indicates if the application has declared a Vref for the ADC.
#define MCP356X_FLAG_CRC_ERROR        0x00000020  // Indicates if a CRC error has been reported by the chip.
#define MCP356X_FLAG_USE_INTERNAL_CLK 0x00000040  // Directs the ADC to use its internal oscillator.
#define MCP356X_FLAG_MCLK_RUNNING     0x00000080  // Indicates if the main clock (MCLK) is running.
#define MCP356X_FLAG_SAMPLED_AVDD     0x00000100  // Indicates if the AVDD channel was sampled for calibration.
#define MCP356X_FLAG_SAMPLED_VCM      0x00000200  // Indicates if the VCM channel was sampled for calibration.
#define MCP356X_FLAG_SAMPLED_OFFSET   0x00000400  // Indicates if the OFFSET channel was sampled for calibration.
#define MCP356X_FLAG_3RD_ORDER_TEMP   0x00000800  // Indicates if additional computation should be spent to improve temperature conversion accuracy.
#define MCP356X_FLAG_GENERATE_MCLK    0x00001000  // Directs the MCU to generate the ADC's main clock.
#define MCP356X_FLAG_HAS_INTRNL_VREF  0x00004000  // Indicates if the ADC supports an internal voltage reference.
#define MCP356X_FLAG_USE_INTRNL_VREF  0x00008000  // Directs the ADC to enable its internal voltage reference.


/**
* @brief Flags that should be preserved during a reset.
*
* Ensuring these flags are preserved helps maintain consistent behavior post-reset.
*/
#define MCP356X_FLAG_RESET_MASK  (MCP356X_FLAG_DEVICE_PRESENT | MCP356X_FLAG_PINS_CONFIGURED | \
                                  MCP356X_FLAG_VREF_DECLARED | MCP356X_FLAG_USE_INTERNAL_CLK | \
                                  MCP356X_FLAG_3RD_ORDER_TEMP | MCP356X_FLAG_GENERATE_MCLK | \
                                  MCP356X_FLAG_HAS_INTRNL_VREF | MCP356X_FLAG_USE_INTRNL_VREF)

/**
* @brief Flags indicating which calibration steps have been completed.
*
* These flags help track the calibration progress and state of the ADC.
*/
#define MCP356X_FLAG_ALL_CAL_MASK  (MCP356X_FLAG_SAMPLED_AVDD | \
                                    MCP356X_FLAG_SAMPLED_VCM | \
                                    MCP356X_FLAG_SAMPLED_OFFSET)



/**
* @class MCP356x
* @brief Represents the MCP356x ADC and provides methods for its configuration and operation.
*
* This class allows users to interface with the MCP356x ADC, offering methods for initialization,
* configuration, reading values, and debugging.
*/
class MCP356x {
public:
  bool isr_fired = false;                       // Flag indicating if an interrupt has been fired.

  // Constructors for different configurations.
  MCP356x(uint8_t irq_pin, uint8_t cs_pin);
  MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin);
  MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin, uint8_t addr);

  ~MCP356x();                                   

  virtual int8_t reset();                       
  virtual int8_t init(SPIClass*);               
  inline int8_t init() { return init(_bus); }   

  int8_t read();                                
  double valueAsVoltage(MCP356xChannel);        
  int32_t value(MCP356xChannel);                

  bool scanComplete();                          

  // Inline methods for retrieving ADC states and counters.
  inline uint32_t lastRead() { return micros_last_read; };  
  inline uint32_t readCount() { return read_count; };       
  inline void resetReadCount() { read_count = 0; };         

  int8_t setOption(uint32_t);                               

  // Methods for setting and getting various ADC configurations.
  int8_t  setOffsetCalibration(int32_t);
  int8_t  setGainCalibration(int32_t);
  int8_t  setGain(MCP356xGain);
  int8_t  setConversionMode(MCP356xMode);
  int8_t  setADCMode(MCP356xADCMode);
  int8_t  setBiasCurrent(MCP356xBiasCurrent);
  int8_t  setAMCLKPrescaler(MCP356xAMCLKPrescaler);
  int8_t  setOversamplingRatio(MCP356xOversamplingRatio);
  MCP356xOversamplingRatio getOversamplingRatio();
  MCP356xGain getGain();

  int8_t setScanChannels(int count, ...);             
  int8_t setReferenceRange(float plus, float minus);  
  int8_t refresh();                                   

  // Inline methods for retrieving ADC configurations and states.
  inline uint8_t getIRQPin() { return _IRQ_PIN; };
  inline bool    adcFound() { return _mcp356x_flag(MCP356X_FLAG_DEVICE_PRESENT); };
  inline bool    adcConfigured() { return _mcp356x_flag(MCP356X_FLAG_INITIALIZED); };
  inline bool    adcCalibrated() { return _mcp356x_flag(MCP356X_FLAG_CALIBRATED); };
  inline bool    hasInternalVref() { return _mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF); };

  bool usingInternalVref();                           
  int8_t useInternalVref(bool);                       

  bool isrFired() { return isr_fired; };              

  // Methods for handling and converting ADC data.
  void discardUnsettledSamples();
  float getTemperature();                             

  // Inline methods for retrieving ADC timings and frequencies.
  inline uint16_t getSampleRate() { return reads_per_second; };
  inline double   getMCLKFrequency() { return _mclk_freq; };
  inline uint32_t getSettlingTime() { return _settling_us; };
  inline uint32_t getCircuitSettleTime() { return _circuit_settle_us; };
  inline void setCircuitSettleTime(uint32_t us) { _circuit_settle_us = us; };

  // Debug and print functions.
  void printPins(StringBuilder*);                     
  void printRegs(StringBuilder*);                     
  void printTimings(StringBuilder*);                  
  void printData(StringBuilder*);                     
  void printChannelValues(StringBuilder*, bool);      
  void printChannel(MCP356xChannel, StringBuilder*);  


protected:
  int8_t  _clear_registers();                          

private:
    // Pin assignments
    const uint8_t  _IRQ_PIN;            
    const uint8_t  _CS_PIN;             
    const uint8_t  _MCLK_PIN;           
    const uint8_t  _DEV_ADDR;           

    SPIClass* _bus = nullptr;           
    double    _mclk_freq = 0.0;         
    double    _dmclk_freq = 0.0;        
    double    _drclk_freq = 0.0;        
    float     _vref_plus = 3.3;         
    float     _vref_minus = 0.0;        

    uint32_t reg_shadows[16];           
    int32_t  channel_vals[16];          
    uint32_t _flags = 0;                
    uint32_t _channel_flags = 0;        
    uint32_t _discard_until_micros = 0; 
    uint32_t _circuit_settle_us = 0;    
    uint32_t _settling_us = 0;          
    uint32_t read_count = 0;            
    uint32_t read_accumulator = 0;      
    uint16_t reads_per_second = 0;      
    uint32_t micros_last_read = 0;      
    uint32_t micros_last_window = 0;    
    uint32_t _channel_backup = 0;       
    uint8_t  _slot_number = 0;          

    // Private methods for internal operations and configurations.
    int8_t  _post_reset_fxn();                
    int8_t  _proc_irq_register();             
    int8_t  _ll_pin_init();                   
    uint8_t _get_reg_addr(MCP356xRegister);   
    int8_t _send_fast_command(uint8_t cmd);
    
    uint8_t _channel_count();
    int8_t  _set_scan_channels(uint32_t);     
    int8_t  _calibrate_offset();              
    int8_t  _mark_calibrated();               

    bool   _mclk_in_bounds();                     
    int8_t _detect_adc_clock();                   
    double _calculate_input_clock(unsigned long); 
    int8_t _recalculate_clk_tree();               
    int8_t _recalculate_settling_time();          

  int8_t _write_register(MCP356xRegister r, uint32_t val);
  int8_t _read_register(MCP356xRegister r);

  uint8_t _output_coding_bytes();
  int8_t  _normalize_data_register();
  float   _gain_value();


  inline bool _vref_declared() { return _mcp356x_flag(MCP356X_FLAG_VREF_DECLARED); };
  inline bool _scan_covers_channel(MCP356xChannel c) {
    return (0x01 & (reg_shadows[(uint8_t)MCP356xRegister::SCAN] >> ((uint8_t)c)));
  };

  /* Flag manipulation inlines */
  inline uint32_t _mcp356x_flags() { return _flags; };
  inline bool _mcp356x_flag(uint32_t _flag) { return (_flags & _flag); };
  inline void _mcp356x_flip_flag(uint32_t _flag) { _flags ^= _flag; };
  inline void _mcp356x_clear_flag(uint32_t _flag) { _flags &= ~_flag; };
  inline void _mcp356x_set_flag(uint32_t _flag) { _flags |= _flag; };
  inline void _mcp356x_set_flag(uint32_t _flag, bool nu) {
    if (nu) _flags |= _flag;
    else    _flags &= ~_flag;
  };

  /* Flag manipulation inlines for individual channels */
  inline void _channel_clear_new_flag(MCP356xChannel c) { _channel_flags &= ~(1 << (uint8_t)c); };
  inline void _channel_set_new_flag(MCP356xChannel c) { _channel_flags |= (1 << (uint8_t)c); };
  inline bool _channel_has_new_value(MCP356xChannel c) { return (_channel_flags & (1 << (uint8_t)c)); };
  inline void _channel_clear_ovr_flag(MCP356xChannel c) { _channel_flags &= ~(0x00010000 << (uint8_t)c); };
  inline void _channel_set_ovr_flag(MCP356xChannel c) { _channel_flags |= (0x00010000 << (uint8_t)c); };
  inline void _channel_set_ovr_flag(MCP356xChannel c, bool nu) {
    _channel_flags = (nu) ? (_channel_flags | (0x00010000 << (uint8_t)c)) : (_channel_flags & ~(0x00010000 << (uint8_t)c));
  };
  inline bool _channel_over_range(MCP356xChannel c) { return (_channel_flags & (0x00010000 << (uint8_t)c)); };
};

#endif  // __MCP356x_H__
