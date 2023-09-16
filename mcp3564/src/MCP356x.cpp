/*
File:   MCP356x.cpp
Author: J. Ian Lindsay
*/

#include "MCP356x.h"

//#define DEBUG_PRINTS

// Conditional compilation for debug prints.
#ifdef DEBUG_PRINTS
#define DBG_PRINT(x)  Serial.println(x)
#else
#define DBG_PRINT(x)
#endif

/**
 * Static members and initializers should be located here.
*/

// Defines the maximum number of MCP356x instances  
// Up to 4 instances can be used in a given system
#define MCP356X_MAX_INSTANCES    4

// Array of MCP356x instance pointers 
// Stores up to MCP356X_MAX_INSTANCES pointers
volatile static MCP356x* INSTANCES[MCP356X_MAX_INSTANCES] = { 0, };

// SPI settings for MCP356x
// 12 MHz clock, Max is 20MHz. MSB first, SPI mode 0.
static SPISettings spi_settings(12000000, MSBFIRST, SPI_MODE0);

// Array indicating the byte-width of each MCP356x register. Useful for memory allocation and data parsing when reading from/writing to registers. 
static const uint8_t MCP356x_reg_width[16] = {
  1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 1, 1, 2, 2
};

// OSR1 values that determine the ADC conversion resolution and speed. Higher values lead to higher resolution but slower conversion speeds.
static const uint16_t OSR1_VALUES[16] = {
  1, 1, 1, 1, 1, 2, 4, 8, 16, 32, 40, 48, 80, 96, 160, 192
};

// OSR3 values that impact the digital filter performance in the ADC. Higher values offer better noise performance at the expense of data output rate.
static const uint16_t OSR3_VALUES[16] = {
  32, 64, 128, 256, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512
};

// Descriptive names of the MCP356x channels.
static const char* CHAN_NAMES[16] = {
  "SE_0", "SE_1", "SE_2", "SE_3", "SE_4", "SE_5", "SE_6", "SE_7",
  "DIFF_A", "DIFF_B", "DIFF_C", "DIFF_D", "TEMP", "AVDD", "VCM", "OFFSET"
};

/**
 * Interrupt service routines for handling
 * interrupts on each MCP356x instance.
*/

void mcp356x_isr0() {
  // Set interrupt flag on Instance 0
  INSTANCES[0]->isr_fired = true;
}

void mcp356x_isr1() {
  // Set interrupt flag on Instance 1
  INSTANCES[1]->isr_fired = true;
}

void mcp356x_isr2() {
  // Set interrupt flag on Instance 2
  INSTANCES[2]->isr_fired = true;
}

void mcp356x_isr3() {
  // Set interrupt flag on Instance 3
  INSTANCES[3]->isr_fired = true;
}

/**
 * Class constructors, destructors, and initialization functions.
*/

/**
 * @brief Constructor that only specifies the IRQ and CS pins.
 *
 * This constructor uses default values for MCLK and address.
 *
 * @param irq_pin The pin number for the interrupt request.
 * @param cs_pin The pin number for chip select.
 */
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin) :
  MCP356x(irq_pin, cs_pin, 0, 0x01) {}

/**
 * @brief Constructor that specifies the IRQ, CS, and MCLK pins.
 *
 * This constructor uses a default value for the address.
 *
 * @param irq_pin The pin number for the interrupt request.
 * @param cs_pin The pin number for chip select.
 * @param mclk_pin The pin number for the master clock.
 */
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin) :
  MCP356x(irq_pin, cs_pin, mclk_pin, 0x01) {}

/**
 * @brief Constructor to initialize the IRQ, CS, MCLK pins and I2C address.
 *
 * It finds the next available slot in the INSTANCES array up to MCP356X_MAX_INSTANCES,
 * stores the pointer to this instance and the slot number.
 *
 * @param irq_pin The pin number for the interrupt request.
 * @param cs_pin The pin number for chip select.
 * @param mclk_pin The pin number for the master clock.
 * @param addr The I2C address of the MCP356x.
 */
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin, uint8_t addr) :
  _IRQ_PIN(irq_pin),
  _CS_PIN(cs_pin),
  _MCLK_PIN(mclk_pin),
  _DEV_ADDR(addr)
{
  // Find empty slot in INSTANCES array
  bool unslotted = true;
  for (uint8_t i = 0; i < MCP356X_MAX_INSTANCES; i++) {
    if (unslotted) {
      if (nullptr == INSTANCES[i]) {
        _slot_number = i;
        INSTANCES[_slot_number] = this;
        unslotted = false;
      }
    }
  }
}

/**
 * @brief Destructor for the MCP356x class.
 *
 * Detaches any attached interrupt and clears the instance from the INSTANCES array.
 */
MCP356x::~MCP356x() {
  if (255 != _IRQ_PIN) {
    detachInterrupt(digitalPinToInterrupt(_IRQ_PIN));
  }
  // Clear pointer in INSTANCES array
  INSTANCES[_slot_number] = nullptr;
}

/**
 * @brief Sends a fast command to reset the ADC and its registers.
 *
 * After sending the reset command, it waits for the ADC to reset,
 * then it resets the SPI interface and clears the ADC registers.
 *
 * @return int8_t Returns 0 if successful, -2 if refresh after reset fails, and -1 if reset command fails.
 */
int8_t MCP356x::reset() {
  // Send fast command to reset ADC
  int8_t ret = _send_fast_command(0x38);

  if (0 == ret) {

    // Delay to allow reset to complete
    delay(75);

    // Reset SPI interface
    digitalWrite(_CS_PIN, LOW);
    digitalWrite(_CS_PIN, HIGH);
    digitalWrite(_CS_PIN, LOW);
    digitalWrite(_CS_PIN, HIGH);

    // Clear ADC registers
    ret = _clear_registers();

    if (ret == 0) {

      // Delay then refresh registers
      delay(75);
      ret = refresh();
    }
  }
  return ret;
}

/**
 * @brief Initializes and configures the MCP356x ADC.
 *
 * The initialization process includes setting up pins, resetting the ADC and its registers,
 * detecting the MCLK rate, and calibrating the offset.
 *
 * @param b A pointer to the SPI bus that the ADC is connected to.
 * @return int8_t Returns 0 if successful. Negative values indicate specific initialization failures.
 */
int8_t MCP356x::init(SPIClass* b) {
  // Configure pins
  int8_t pin_setup_ret = _ll_pin_init();  // Configure the pins if they are not already.

  // Default return value indicating pin setup failure
  int8_t ret = -4;
  if (pin_setup_ret >= 0) {

    // Check for valid SPI bus reference
    ret = -3;
    if (nullptr != b) {
      ret = -2;
      // Reset ADC and check for success

      _bus = b;
      if (0 == reset()) {
        ret = -5;
        // Post-reset function checks
        if (0 == _post_reset_fxn()) {
          // MCLK bounds checking and detection
          if (!_mclk_in_bounds()) {
            ret = -6;
            int8_t det_ret = _detect_adc_clock();
            // Handle various clock detection outcomes
            // TODO: Add additional handling or logging based on detection results.
            // Currently, these cases are placeholders.
            switch (det_ret) {
            case -3:    break;  // -3 if the class isn't ready for this measurement.
            case -2:    break;  // -2 if measurement timed out.
            case -1:    break;  // -1 if there was some mechanical problem communicating with the ADC
            case 0:     break;  // 0  if a clock signal within expected range was determined
            case 1:     break;  // 1  if we got a clock rate that was out of bounds or appears nonsensical
            default:    break;
            }
            //Serial.print("_detect_adc_clock returned ");
            //Serial.println(det_ret);
          }
          // Offset calibration
          if (_mclk_in_bounds()) {
            switch (_calibrate_offset()) {
            case 0:   ret = 0;    break;
            default:  ret = -7;   break;
            }
          }
        }
      }
    }
  }
  return ret;
}


/**
 * @brief Configures specialized ADC features based on provided flags.
 *
 * This function allows for the configuration of several ADC features, including the use of
 * an internal clock, MCLK signal generation, and the choice of temperature calculation method.
 *
 * @note This function should be called before the init() function to ensure proper configuration.
 *
 * @param flgs Flags OR'd together to specify ADC configuration. Flags are defined in the header file.
 * @return int8_t Returns 0 if options are set successfully. Returns -1 if function is called too late
 *               and pins have already been configured.
 */
int8_t MCP356x::setOption(uint32_t flgs) {

  int8_t ret = 0;

  // Use internal clock
  if (flgs & MCP356X_FLAG_USE_INTERNAL_CLK) {
    if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))) {

      // Only allow this change if the pins are not yet configured.
      _mcp356x_set_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
      _mcp356x_clear_flag(MCP356X_FLAG_GENERATE_MCLK);
    }
    else {
      ret = -1;
    }
  }

  // Generate MCLK signal
  if (flgs & MCP356X_FLAG_GENERATE_MCLK) {
    if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))) {

      // Only allow this change if the pins are not yet configured.
      _mcp356x_set_flag(MCP356X_FLAG_GENERATE_MCLK);
      _mcp356x_clear_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
    }
    else {
      ret = -1;
    }
  }

  // 3rd order temperature calculation
  if (flgs & MCP356X_FLAG_3RD_ORDER_TEMP) {
    _mcp356x_set_flag(MCP356X_FLAG_3RD_ORDER_TEMP);
  }
  return ret;
}


/**
 * @brief Handles post-reset configurations of the ADC.
 *
 * After the ADC is reset, this function configures it by writing default values to specific
 * registers. The configurations include enabling fast commands, configuring the IRQ behavior,
 * and setting data format.
 *
 * @return int8_t Returns 0 if post-reset configuration is successful. Returns -1 if there's a
 *               problem writing to the registers.
 */
int8_t MCP356x::_post_reset_fxn() {
  int8_t ret = -1;

  uint32_t c0_val = 0x000000C3;    // Default config values for CONFIG0
  //uint32_t c1_val = 0x00000000;

  // Enable fast command, disable IRQ on conversion start, IRQ pin is open-drain.
  ret = _write_register(MCP356xRegister::IRQ, 0x00000002);
  if (0 == ret) {
    if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
      c0_val &= 0xFFFFFFCF;   // Set CLK_SEL to use internal clock with no pin output.
      c0_val |= 0x00000020;
    }
    ret = _write_register(MCP356xRegister::CONFIG0, c0_val);
    if (0 == ret) {
      if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
        _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
      }

      // For simplicity, a 32 is selected - bit sign - extended data representation with channel identifiers.
      ret = _write_register(MCP356xRegister::CONFIG3, 0x000000F1);
      if (0 == ret) {
        _mcp356x_set_flag(MCP356X_FLAG_INITIALIZED);
      }
    }
  }
  return ret;
}


/**
 * @brief Reads the ADC's output register and processes data based on the current configuration.
 *
 * This function reads the output register and manipulates the data as per the current configuration.
 * If the `discardUnsettledSamples()` method was recently called, the register shadow might not be updated
 * unless the timeout has elapsed. However, the accurate register value will always be returned.
 * To determine if the read data is valid, consider invoking `newValue()` or `scanComplete()` before
 * using the data.
 *
 * @return Returns:
 *         - `-1` if there's an error reading from the registers.
 *         - `0` if the read is successful but there's no further action.
 *         - `1` if data was available and read, but discarded due to the declared settling time.
 *         - `2` if the read resulted in a full dataset for all channels being scanned.
 */
int8_t MCP356x::read() {
  // Process the interrupt register and get the status
  int8_t ret = _proc_irq_register();

  switch (ret) {
  case -1:
    // Error in processing the interrupt register
    break;

  case 0:
    // Successfully processed but nothing to do
    break;

  case 1:
    // Read the ADC data register
    _read_register(MCP356xRegister::ADCDATA);

    // Record the current microsecond timestamp
    micros_last_read = micros();

    // Check if the read data should be processed or discarded
    if (_discard_until_micros <= micros_last_read) {
      // Process and format the retrieved data
      _normalize_data_register();

      // Verify if scanning across all channels has been completed
      if (scanComplete()) {
        ret = 2;
      }
    }

    // Update read statistics
    read_count++;
    read_accumulator++;

    // Compute the reads-per-second rate and reset counters if a second has elapsed
    if (micros_last_read - micros_last_window >= 1000000) {
      micros_last_window = micros_last_read;
      reads_per_second = read_accumulator;
      read_accumulator = 0;
    }
    break;

  default:
    // Handle any unexpected return value
    break;
  }

  return ret;
}


/**
 * @brief Process the contents of the IRQ register and respond accordingly.
 *
 * Reads the IRQ (Interrupt Request) register and takes necessary actions
 * based on its contents. This function provides insight into the conversion
 * state, errors, and other events triggered by the ADC.
 *
 * @return Returns:
 *         - `-1` if reading the IRQ register failed.
 *         - `0` if no further action is required following this.
 *         - `1` if the ADC data register needs to be read.
 */
int8_t MCP356x::_proc_irq_register() {
  int8_t ret = -1;  // Default return value indicating a failed read

  // Define named constants 
  const uint8_t CONVERSION_FINISHED_MASK = 0x40;
  const uint8_t CRC_ERROR_MASK = 0x20;
  const uint8_t POWER_ON_RESET_MASK = 0x08;
  const uint8_t CONVERSION_STARTED_MASK = 0x01;

  // Attempt to read the IRQ register and check for success
  if (0 == _read_register(MCP356xRegister::IRQ)) {
    ret = 0;   // Set return value indicating no further action required

    // Extract the IRQ register's data from the shadow register array
    uint8_t irq_reg_data = (uint8_t)reg_shadows[(uint8_t)MCP356xRegister::IRQ];

    // Handle CRC errors
    _mcp356x_set_flag(MCP356X_FLAG_CRC_ERROR, !(irq_reg_data & CRC_ERROR_MASK));


    // If conversion has finished, indicate data register needs reading
    if (!(irq_reg_data & CONVERSION_FINISHED_MASK)) {
      ret = 1;
    }

    // Handle Power-on-Reset event
    if (!(irq_reg_data & POWER_ON_RESET_MASK)) {
      // Perform chip-select toggle to handle reset
      digitalWrite(_CS_PIN, LOW);
      digitalWrite(_CS_PIN, HIGH);
      digitalWrite(_CS_PIN, LOW);
      digitalWrite(_CS_PIN, HIGH);
    }

    // Handle CRC configuration errors
    if (!(irq_reg_data & CRC_ERROR_MASK)) {
      _write_register(MCP356xRegister::LOCK, 0x000000A5);
      _send_fast_command(0x28);
    }

    // Check if a conversion has started
    if (irq_reg_data & CONVERSION_STARTED_MASK) {
      // Class isn't configured for this, so IRQ is ignored
    }
  }

  // Update the ISR fired flag based on the IRQ pin status
  isr_fired = !digitalRead(_IRQ_PIN);

  return ret;
}


/**
 * @brief Returns the number of channels supported by the device.
 *
 * The function determines the MCP356x variant by reading the RESERVED2 register.
 * Only valid values are:
 * - MCP3561: 2 channels
 * - MCP3562: 4 channels
 * - MCP3564: 8 channels
 *
 * Any other value is invalid and indicates a need for a register sync (if 0) or
 * the wrong device entirely.
 *
 * @return uint8_t Number of channels supported (2, 4, 8). Returns 0 if invalid or unknown.
 */
uint8_t MCP356x::_channel_count() {
  // Define named constants for the device types
  const uint16_t MCP3561_ID = 0x000C;
  const uint16_t MCP3562_ID = 0x000D;
  const uint16_t MCP3564_ID = 0x000F;

  // Get the device ID from the shadow register
  uint16_t device_id = (uint16_t)reg_shadows[(uint8_t)MCP356xRegister::RESERVED2];

  // Determine the device type and return the channel count
  switch (device_id) {
  case MCP3561_ID:   return 2;
  case MCP3562_ID:   return 4;
  case MCP3564_ID:   return 8;
  default:           return 0;  // Unknown or invalid device
  }
}


/**
 * @brief Converts the ADC's digital value for a specific channel into its equivalent voltage.
 *
 * This function uses the ADC's reference voltages and gain value to convert the digital value
 * of a specified channel to its voltage equivalent.
 *
 * @param chan The ADC channel whose value you wish to convert.
 * @return The voltage equivalent of the ADC's digital value for the specified channel.
 */
double MCP356x::valueAsVoltage(MCP356xChannel chan) {

  // Define constants for
  const double MAX_POSITIVE_24BIT_VALUE = 8388608.0;  // Half the range of 24-bit
  const float AVDD_CHANNEL_GAIN = 0.33;  // Gain for AVDD channel is always 0.33


  // Using previously defined reference voltages
  float  vrp = _vref_plus;     // Positive reference voltage
  float  vrm = _vref_minus;    // Negative reference voltage

  double result = 0.0;         // Initialize result

  // Determine the channel type to decide the conversion method
  switch (chan) {
    // For single-ended channels and differential channels:
  case MCP356xChannel::SE_0:
  case MCP356xChannel::SE_1:
  case MCP356xChannel::SE_2:
  case MCP356xChannel::SE_3:
  case MCP356xChannel::SE_4:
  case MCP356xChannel::SE_5:
  case MCP356xChannel::SE_6:
  case MCP356xChannel::SE_7:
    // Differential channels.    
  case MCP356xChannel::DIFF_A:
  case MCP356xChannel::DIFF_B:
  case MCP356xChannel::DIFF_C:
  case MCP356xChannel::DIFF_D:

  case MCP356xChannel::OFFSET:

    // Convert the ADC's digital value to voltage using the specified formula
    result = (value(chan) * (vrp - vrm)) / (MAX_POSITIVE_24BIT_VALUE * _gain_value());
    break;

  case MCP356xChannel::TEMP:
    // If the channel is a temperature sensor, conversion isn't implemented yet
    // TODO: Implement voltage conversion for the temperature sensor channel
    break;

  case MCP356xChannel::AVDD:

    // Convert the ADC's digital value for the AVDD channel to voltage
    result = value(chan) / (AVDD_CHANNEL_GAIN * MAX_POSITIVE_24BIT_VALUE);   // Gain on this chan is always 0.33.
    break;

  case MCP356xChannel::VCM:

    // Convert the ADC's digital value for the VCM channel to voltage
    result = (value(chan) * (vrp - vrm)) / MAX_POSITIVE_24BIT_VALUE;
    break;
  }

  // Return the calculated voltage value
  return result;
}

/*
* Retrieve the latest reading of a specified ADC channel.
* @param MCP356xChannel chan: The ADC channel whose value needs to be retrieved.
* @return int32_t: The latest reading value for the specified channel.
*/
int32_t MCP356x::value(MCP356xChannel chan) {
  const uint8_t CHANNEL_MASK = 0x0F;

  _channel_clear_new_flag(chan);
  return channel_vals[static_cast<uint8_t>(chan) & CHANNEL_MASK];
}


/**
 * @brief Sets the offset calibration for the ADC.
 *
 * If the provided offset is non-zero, the offset feature is enabled.
 *
 * @param offset The offset value for calibration.
 * @return Returns 0 on success and non-zero on failure.
 */
int8_t MCP356x::setOffsetCalibration(int32_t offset) {
  // Constants for better readability
  const uint32_t OFFSET_ENABLE_BIT = 0x00000002;  // Bit to enable the offset
  const uint32_t OFFSET_DISABLE_MASK = 0xFFFFFFFD; // Mask to disable the offset

  // Write the offset value to the OFFSETCAL register
  int8_t ret = _write_register(MCP356xRegister::OFFSETCAL, (uint32_t)offset);

  if (0 == ret) {
    // Read the current value of the CONFIG3 register
    uint32_t config_val = reg_shadows[(uint8_t)MCP356xRegister::CONFIG3];

    // Modify the CONFIG3 register value to enable or disable the offset feature
    config_val = (0 != offset) ? (config_val | OFFSET_ENABLE_BIT) : (config_val & OFFSET_DISABLE_MASK);

    // Write the modified value back to the CONFIG3 register
    ret = _write_register(MCP356xRegister::CONFIG3, config_val);
  }
  return ret;
}

/**
 * @brief Sets the ADC mode for the MCP356x.
 *
 * This function modifies the CONFIG0 register to set the desired ADC mode.
 *
 * @param mode The desired ADC mode.
 * @return Returns 0 on success and non-zero on failure.
 */
int8_t MCP356x::setADCMode(MCP356xADCMode mode) {
  // Constants for better readability
  const uint32_t ADC_MODE_CLEAR_MASK = 0xFFFFFFFC; // Mask to clear the ADC mode bits
  const uint8_t ADC_MODE_BIT_MASK = 0x03;          // Mask to get the two least significant bits

  // Read the current CONFIG0 register value
  uint32_t config_val = reg_shadows[(uint8_t)MCP356xRegister::CONFIG0];

  // Clear the current ADC_MODE bits
  config_val &= ADC_MODE_CLEAR_MASK;

  // Set the desired ADC mode
  config_val |= ((uint8_t)mode & ADC_MODE_BIT_MASK);

  // Write the modified value back to the CONFIG0 register
  return _write_register(MCP356xRegister::CONFIG0, config_val);
}

/**
 * Sets the gain calibration value for the MCP356x device.
 *
 * @param multiplier The gain calibration value to be written to GAINCAL register.
 *                   The ADC output will be multiplied by this value.
 *
 * @return int8_t Returns the status of the write operation to CONFIG3 register.
 */
int8_t MCP356x::setGainCalibration(int32_t multiplier) {
  // Constants related to the MCP356x ADC
  constexpr uint32_t EN_GAINCAL_BIT_POSITION = 0x00000001;   // Bit position for EN_GAINCAL in CONFIG3 register.

  // Write the provided multiplier to the GAINCAL register
  _write_register(MCP356xRegister::GAINCAL, static_cast<uint32_t>(multiplier));

  // Get the current value of CONFIG3 register from the shadow register array
  uint32_t c_val = reg_shadows[static_cast<uint8_t>(MCP356xRegister::CONFIG3)];

  // If multiplier is not zero, set the EN_GAINCAL bit (bit 0) of CONFIG3 register to enable gain calibration
  // Otherwise, clear the EN_GAINCAL bit to disable gain calibration
  if (multiplier != 0) {
    c_val |= EN_GAINCAL_BIT_POSITION;
  }
  else {
    c_val &= ~EN_GAINCAL_BIT_POSITION;
  }

  // Write the updated value of CONFIG3 register to enable/disable the gain calibration
  return _write_register(MCP356xRegister::CONFIG3, c_val);
}


/**
 * @brief Sets the gain setting for the MCP356x ADC.
 *
 * Configures the MCP356x's CONFIG2 register to adjust the ADC's gain.
 * Proper gain setting ensures accurate ADC measurements.
 *
 * @param g Desired gain setting, as defined in the MCP356xGain enumeration.
 * @return int8_t Status of the register write operation.
 */
int8_t MCP356x::setGain(MCP356xGain g) {
  // Constants for the gain configuration
  const uint32_t CONFIG2_GAIN_MASK = 0xFFFFFFC7;  // Mask to clear gain bits (bits 5-3)
  const uint8_t GAIN_POSITION = 3;                // Gain bits start at position 3

  // Get the current value of the CONFIG2 register from the cache.
  uint32_t currentConfig = reg_shadows[(uint8_t)MCP356xRegister::CONFIG2];

  // Reset the gain bits and set the new gain value.
  uint32_t updatedConfig = (currentConfig & CONFIG2_GAIN_MASK) | ((uint32_t)g << GAIN_POSITION);

  // Write the updated value back to the CONFIG2 register.
  return _write_register(MCP356xRegister::CONFIG2, updatedConfig);
}


/**
 * @brief Retrieves the current gain setting of the MCP356x ADC.
 *
 * This function reads the gain setting from the cached value of the
 * CONFIG2 register. The gain setting is useful to determine the current
 * sensitivity and resolution of the ADC measurements.
 *
 * @return MCP356xGain Current gain setting as defined in the MCP356xGain enumeration.
 */
MCP356xGain MCP356x::getGain() {
  // constants 
  const uint8_t GAIN_BIT_START = 3;
  const uint8_t GAIN_BIT_MASK = 0x07;

  // Extract the gain bits (bits 5-3) from the CONFIG2 register's cached value.
  return (MCP356xGain)((reg_shadows[(uint8_t)MCP356xRegister::CONFIG2] >> GAIN_BIT_START) & GAIN_BIT_MASK);
}


/**
 * @brief Set the current sink/source level for the strain gauge sensor bias.
 *
 * This function configures the MCP356x's CONFIG0 register to set the desired current sink/source
 * level for sensor bias. The bias current setting affects the strain gauge's performance, and
 * choosing the right value is crucial for accurate measurements.
 *
 * @param d Desired bias current setting, as defined in the MCP356xBiasCurrent enumeration.
 * @return int8_t Status of the register write operation.
 */
int8_t MCP356x::setBiasCurrent(MCP356xBiasCurrent d) {
  // constants 
  const uint32_t CONFIG0_BIAS_MASK = 0x00F3FFFF;   // Mask to preserve all bits in CONFIG0 except for CS_SEL[1:0]
  const uint8_t BIAS_BIT_START = 18;               // Starting position of the CS_SEL[1:0] bits in CONFIG0

  // Extract current value of CONFIG0, preserving all bits except CS_SEL[1:0]
  uint32_t c0_val = reg_shadows[(uint8_t)MCP356xRegister::CONFIG0] & CONFIG0_BIAS_MASK;

  // Embed the desired bias current setting into the CS_SEL[1:0] position
  c0_val |= ((static_cast<uint8_t>(d) & 0x03) << BIAS_BIT_START);

  // Write the updated value to the CONFIG0 register
  return _write_register(MCP356xRegister::CONFIG0, c0_val);
}

/**
 * @brief Set the conversion mode for the ADC.
 *
 * This function configures the MCP356x's CONFIG3 register to set the desired conversion
 * mode (either one-shot with shutdown/standby or continuous).
 *
 * @param mode Desired conversion mode, as defined in the MCP356xMode enumeration.
 * @return int8_t Status of the register write operation.
 */
int8_t MCP356x::setConversionMode(MCP356xMode mode) {
  // constants
  const uint8_t CONV_MODE_BIT_START = 6;
  const uint8_t CONV_MODE_MASK = 0x03;         // Mask for the CONV_MODE[1:0] bits
  const uint32_t CONFIG3_PRESERVE_MASK = 0xFFFFFFC0;  // Mask to preserve all bits in CONFIG3 except for CONV_MODE[1:0]

  // Preserve all bits in CONFIG3 except for CONV_MODE[1:0]
  uint32_t c3_val = reg_shadows[(uint8_t)MCP356xRegister::CONFIG3] & CONFIG3_PRESERVE_MASK;

  // Set the CONV_MODE[1:0] bits in CONFIG3 based on the desired conversion mode
  c3_val |= ((uint8_t)mode & CONV_MODE_MASK) << CONV_MODE_BIT_START;

  // Write the updated value to the CONFIG3 register
  return _write_register(MCP356xRegister::CONFIG3, c3_val);
}

/**
 * @brief Configures the AMCLK prescaler value of the MCP356x ADC.
 *
 * This function modifies the AMCLK prescaler settings in the CONFIG1 register
 * based on the provided value. Adjusting the AMCLK prescaler affects the
 * frequency at which the ADC operates.
 *
 * @param d The desired AMCLK prescaler setting as defined in MCP356xAMCLKPrescaler enumeration.
 * Values:
 * - 00: AMCLK = MCLK
 * - 01: AMCLK = MCLK/2
 * - 10: AMCLK = MCLK/4
 * - 11: AMCLK = MCLK/8
 *
 * @return int8_t
 * - -2: If MCLK frequency is unknown, but prescaler setting succeeded.
 * - -1: If reconfiguration of prescaler failed.
 * - 0: On success.
 */
int8_t MCP356x::setAMCLKPrescaler(MCP356xAMCLKPrescaler d) {
  // Define constants
  const uint8_t AMCLK_PRESCALER_BIT_START = 6;
  const uint8_t AMCLK_PRESCALER_MASK = 0x03;
  const uint32_t CONFIG1_PRESCLR_CLEAR_MASK = 0x00FFFF3F;

  // Clear the prescaler bits (bits 7-6) of CONFIG1 register and retain the rest.
  uint32_t c1_val = reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & CONFIG1_PRESCLR_CLEAR_MASK;

  // Set the provided prescaler value in the appropriate position.
  c1_val |= ((((uint8_t)d) & AMCLK_PRESCALER_MASK) << AMCLK_PRESCALER_BIT_START);

  // Write the updated CONFIG1 register value.
  int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);

  // If the write operation succeeded, recalculate the clock tree.
  if (0 == ret) {
    ret = _recalculate_clk_tree();
  }

  return ret;
}


/**
 * @brief Set the Oversampling Ratio (OSR) for the ADC.
 *
 * This function adjusts the OSR in the CONFIG1 register, which affects the
 * number of samples the ADC takes per conversion. A higher OSR generally leads
 * to reduced noise and better effective resolution, but it also slows down the
 * conversion rate.
 *
 * After setting the OSR, the function recalculates the ADC settling time, which
 * might vary depending on the OSR.
 *
 * @param d The desired Oversampling Ratio setting.
 * @return int8_t Returns 0 on success, and -1 if the configuration failed.
 */
int8_t MCP356x::setOversamplingRatio(MCP356xOversamplingRatio d) {
  // Define constants
  const uint8_t OSR_BIT_START = 2;
  const uint8_t OSR_BIT_MASK = 0x0F;
  const uint32_t CONFIG1_MASK = 0x00FFFFC3;

  // Mask out the OSR bits from the cached CONFIG1 value, then set the desired OSR
  uint32_t c1_val = reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & CONFIG1_MASK;
  c1_val |= ((static_cast<uint8_t>(d) & OSR_BIT_MASK) << OSR_BIT_START);

  int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);

  // Recalculate ADC settling time if OSR was successfully updated
  if (0 == ret) {
    ret = _recalculate_settling_time();
  }

  return ret;
}

/**
 * @brief Retrieves the current oversampling ratio setting of the ADC.
 *
 * This function reads the oversampling ratio (OSR) setting from the cached value of the CONFIG1 register
 * and converts it to the corresponding enum value.
 *
 * @return The current OSR setting as an MCP356xOversamplingRatio enumeration value.
 */
MCP356xOversamplingRatio MCP356x::getOversamplingRatio() {
  // Define constants
  const uint8_t OSR_BIT_START = 2;
  const uint8_t OSR_BIT_MASK = 0x3C;  // This mask represents bits [5:2] in binary: 00111100

  // Extract the OSR bits from the CONFIG1 register's cached value.
  return (MCP356xOversamplingRatio)((reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & OSR_BIT_MASK) >> OSR_BIT_START);
}

/**
 * @brief Application-facing accessor for VREF selection, if available.
 *
 * This function checks if the ADC is using its internally generated voltage reference (VREF).
 *
 * @return true if Vref is using the internally generated value.
 */
bool MCP356x::usingInternalVref() {
  // Define constants.
  const uint8_t VREF_BIT_MASK = 0x40;  // Bitmask for the internal VREF enable bit in CONFIG0 register.

  // Check if the ADC supports internal VREF and if it's enabled.
  if (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF)) {
    return (reg_shadows[(uint8_t)MCP356xRegister::CONFIG0] & VREF_BIT_MASK) != 0;
  }
  return false;
}


/**
 * Application-facing accessor for VREF selection, if available.
 *
 * @param x true will enable the internal Vref, if available.
 * @return 0 on success, -1 on "not supported", or -2 on I/O failure.
 */
int8_t MCP356x::useInternalVref(bool x) {
  // Define constants
  const uint32_t INTERNAL_VREF_BIT_MASK = 0x00000040;
  const uint32_t CONFIG0_MASK_CLEAN_VREF = 0x00FFFFBF;
  const float INTERNAL_VREF_PLUS = 3.3;
  const float INTERNAL_VREF_MINUS = 0.0;

  int8_t ret = -1;  // Default return value for "not supported"

  // Check if internal Vref is supported by the chip
  if (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF)) {
    // Get the current value of CONFIG0 register, excluding the Vref bit
    uint32_t c0_val = reg_shadows[(uint8_t)MCP356xRegister::CONFIG0] & CONFIG0_MASK_CLEAN_VREF;

    // Set internal Vref bit if required
    if (x) {
      c0_val |= INTERNAL_VREF_BIT_MASK;
      _vref_plus = INTERNAL_VREF_PLUS;
      _vref_minus = INTERNAL_VREF_MINUS;
    }

    // Update the CONFIG0 register with the new value
    ret = (0 == _write_register(MCP356xRegister::CONFIG0, c0_val)) ? 0 : -2;  // -2 for I/O failure
  }

  return ret;
}


/**
 * @brief Setup the low-level pin details. Execution is idempotent.
 * @return
 *   -1 if the pin setup is wrong. Class must halt.
 *    0 if the pin setup is complete.
 *    1 if the pin setup is complete, and the clock needs measurement.
 */
int8_t MCP356x::_ll_pin_init() {
  // Constants local to the function
  const uint8_t UNDEFINED_PIN = 255;
  const double MCLK_FREQUENCY = 4915200.0;

  int8_t ret = -1;

  DBG_PRINT(" - Entered _ll_pin_init");

  if (_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)) {
    ret = 0;
    DBG_PRINT("Pins already configured");
  }
  else if (_CS_PIN != UNDEFINED_PIN) {
    ret = 1;

    DBG_PRINT("Configuring CS_PIN...");
    pinMode(_CS_PIN, OUTPUT);
    digitalWrite(_CS_PIN, HIGH);

    if (_IRQ_PIN != UNDEFINED_PIN) {
      DBG_PRINT("Configuring IRQ_PIN...");
      pinMode(_IRQ_PIN, INPUT_PULLUP);
      switch (_slot_number) {
      case 0:
        attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr0, FALLING);
        DBG_PRINT("IRQ_PIN ISR set for slot 0");
        break;
      case 1:
        attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr1, FALLING);
        DBG_PRINT("IRQ_PIN ISR set for slot 1");
        break;
      case 2:
        attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr2, FALLING);
        DBG_PRINT("IRQ_PIN ISR set for slot 2");
        break;
      }
    }

    if (_MCLK_PIN != UNDEFINED_PIN) {
      DBG_PRINT("Configuring MCLK_PIN...");
      if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
        pinMode(_MCLK_PIN, INPUT);
        DBG_PRINT("MCLK_PIN set as INPUT for internal clock");
      }
      else {
        pinMode(_MCLK_PIN, OUTPUT);
        if (_mcp356x_flag(MCP356X_FLAG_GENERATE_MCLK)) {
          DBG_PRINT("Generating MCLK...");
          analogWriteFrequency(_MCLK_PIN, MCLK_FREQUENCY);
          analogWrite(_MCLK_PIN, 128);  // Set to 50% duty cycle
          _mclk_freq = MCLK_FREQUENCY;
          DBG_PRINT("MCLK generated and frequency set");
        }
        else {
          digitalWrite(_MCLK_PIN, HIGH);
          DBG_PRINT("MCLK_PIN set HIGH for external oscillator");
        }
        _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
        ret = _recalculate_clk_tree();
        DBG_PRINT("Clock tree recalculated");
      }
    }
    _mcp356x_set_flag(MCP356X_FLAG_PINS_CONFIGURED);
    DBG_PRINT("Pins configured and flags set");
  }

  DBG_PRINT("Exiting _ll_pin_init");
  return ret;
}

/**
 * @brief   Resets the internal register shadows to their default values.
 *          Also resets internal second-order data to avoid corrupted results.
 *
 * @return  int8_t Returns 0 after successful execution.
 */
int8_t MCP356x::_clear_registers() {

  const uint32_t TOTAL_REGISTERS = 16;

  // Preserve specific flags during reset
  uint32_t flg_mask = MCP356X_FLAG_RESET_MASK;
  if (!(_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK))) {
    // The only way the clock isn't running is if it is running internally.
    flg_mask |= MCP356X_FLAG_MCLK_RUNNING;
  }
  _flags = _flags & flg_mask;  // Reset the flags.

  // Reset register shadows and channel values
  for (uint8_t i = 0; i < TOTAL_REGISTERS; i++) {
    // We decline to revert RESERVED2, since we need it for device identity.
    // RESERVED2 values differ for 3564, 3562, and 3561
    reg_shadows[i] = (14 != i) ? 0 : reg_shadows[i];
    channel_vals[i] = 0;
  }
  _channel_flags = 0;
  _discard_until_micros = 0;
  _settling_us = 0;
  read_count = 0;
  read_accumulator = 0;
  reads_per_second = 0;
  micros_last_read = 0;
  micros_last_window = 0;

  return 0;
}

/**
 * @brief Writes a value to a specified register, applying safety checks to ensure valid values.
 *
 * This function ensures that only valid bits are written to the specified registers.
 * It first masks out any invalid bits, then writes the resulting value to the register.
 * The function also ensures that non-writable registers are protected.
 *
 * @param r Register to which the value needs to be written.
 * @param val The value to be written to the register.
 * @return Returns 0 on successful write.
 *         Returns -1 if register is not handled.
 *         Returns -2 on unexpected register width.
 *         Returns -3 if trying to write to a non-writable register.
 */
int8_t MCP356x::_write_register(MCP356xRegister r, uint32_t val) {
  const uint32_t CONFIG1_MASK = 0xFFFFFFFC;
  const uint32_t SCAN_MASK = 0xFFE0FFFF;
  const uint32_t RESERVED0_VALUE = 0x00900000;
  const uint32_t RESERVED2_MASK = 0x0000000F;

  uint32_t safe_val = 0;
  int8_t ret = -1;
  uint8_t register_size = MCP356x_reg_width[(uint8_t)r];

  switch (r) {
  case MCP356xRegister::CONFIG1:
    safe_val = val & CONFIG1_MASK;
    break;
  case MCP356xRegister::CONFIG2:
    // If the ADC has an internal voltage reference, set the least significant bit of safe_val.
    // Otherwise, set the two least significant bits. This manipulates the settings of the CONFIG2 register.
    safe_val = val | (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000001 : 0x00000003);
    break;
  case MCP356xRegister::SCAN:
    safe_val = val & SCAN_MASK;
    break;
  case MCP356xRegister::RESERVED0:
    safe_val = RESERVED0_VALUE;
    break;
  case MCP356xRegister::RESERVED1:
    safe_val = (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000030 : 0x00000050);
    break;
  case MCP356xRegister::RESERVED2:
    safe_val = val & RESERVED2_MASK;
    break;
  case MCP356xRegister::CONFIG0:
  case MCP356xRegister::CONFIG3:
  case MCP356xRegister::IRQ:
  case MCP356xRegister::MUX:
  case MCP356xRegister::TIMER:
  case MCP356xRegister::OFFSETCAL:
  case MCP356xRegister::GAINCAL:
  case MCP356xRegister::LOCK:
    safe_val = val;
    break;
  case MCP356xRegister::ADCDATA:
  case MCP356xRegister::CRCCFG:
    return -3;
  }

  _bus->beginTransaction(spi_settings);
  digitalWrite(_CS_PIN, 0);
  _bus->transfer(_get_reg_addr(r));
  switch (register_size) {
  case 3:
    _bus->transfer((uint8_t)(safe_val >> 16) & 0xFF);   // MSB-first
  case 2:
    _bus->transfer((uint8_t)(safe_val >> 8) & 0xFF);
  case 1:
    _bus->transfer((uint8_t)safe_val & 0xFF);
    ret = 0;
    reg_shadows[(uint8_t)r] = safe_val;
    break;
  default:
    ret = -2;   // Error on unexpected width.
  }
  digitalWrite(_CS_PIN, 1);
  _bus->endTransaction();
  return ret;
}


/**
 * @brief Reads the value from the specified ADC register.
 *
 * This function reads the value of a given register of the MCP356x ADC. The
 * number of bytes read varies depending on the register, and this is handled
 * within the function.
 *
 * @param r The MCP356x register to be read.
 *
 * @return Always returns `0`.
 */
int8_t MCP356x::_read_register(MCP356xRegister r) {
  // Constants for SPI communication
  const uint8_t SPI_READ_COMMAND = 0x01;
  const uint8_t SPI_CS_ACTIVE = 0;
  const uint8_t SPI_CS_INACTIVE = 1;

  // Determine the width of the register (how many bytes to read)
  uint8_t bytes_to_read = MCP356x_reg_width[(uint8_t)r];

  // If we're reading ADCDATA, determine the byte count by the output coding setting
  if (MCP356xRegister::ADCDATA == r) {
    bytes_to_read = _output_coding_bytes();
  }

  // Start an SPI transaction
  _bus->beginTransaction(spi_settings);

  // Pull the CS (Chip Select) pin low to initiate communication
  digitalWrite(_CS_PIN, SPI_CS_ACTIVE);

  // Send the address of the register we want to read, with the R/W bit set for reading
  _bus->transfer((uint8_t)_get_reg_addr(r) | SPI_READ_COMMAND);

  // Read bytes from the register and store the value
  uint32_t temp_val = 0;
  for (int i = 0; i < bytes_to_read; i++) {
    temp_val = (temp_val << 8) + _bus->transfer(0);
  }

  // Deactivate the CS pin to end communication
  digitalWrite(_CS_PIN, SPI_CS_INACTIVE);

  // End the SPI transaction
  _bus->endTransaction();

  // Update the shadow register array with the read value
  reg_shadows[(uint8_t)r] = temp_val;

  return 0;  // Currently always returns 0
}



/*
* Returns the number of bytes to be read from the data register.
*/
uint8_t MCP356x::_output_coding_bytes() {
  return (0 == reg_shadows[(uint8_t)MCP356xRegister::CONFIG3]) ? 3 : 4;
}

/**
 * @brief Normalize ADC data register.
 *
 * This function processes and normalizes the ADC data based on the channel
 * and output coding. It also checks for calibration flags and sets
 * calibration markers as necessary.
 *
 * @return Always returns 0.
 */
int8_t MCP356x::_normalize_data_register() {

  // Constants for ADC data processing.
  constexpr uint32_t ADC_UPPER_MASK = 0x0F0000000;
  constexpr uint32_t ADC_SIGN_MASK = 0x01000000;
  constexpr uint32_t ADC_DATA_MASK = 0x01FFFFFF;
  constexpr int32_t ADC_MAX_VALUE = 8388608; // 2^23
  constexpr double ADC_VREF_CONVERSION_FACTOR = 0.33; // Assumed value from the datasheet.

  // Extract the raw ADC value from the shadows register.
  uint32_t rval = reg_shadows[static_cast<uint8_t>(MCP356xRegister::ADCDATA)];

  // Determine the active channel from the upper 4 bits of the ADC data.
  MCP356xChannel chan = static_cast<MCP356xChannel>((rval & ADC_UPPER_MASK) >> 28);

  // Sign extend to get the ADC data as a signed value.
  int32_t nval = (rval & ADC_SIGN_MASK) ? (rval | ~ADC_DATA_MASK) : (rval & ADC_DATA_MASK);

  // Store the decoded ADC reading and update channel flags for new data and over-range.
  channel_vals[static_cast<uint8_t>(chan)] = nval;
  _channel_set_ovr_flag(chan, ((nval > ADC_MAX_VALUE - 1) || (nval < -ADC_MAX_VALUE)));
  _channel_set_new_flag(chan);

  switch (chan) {
    // Single-ended and Differential channels have no specific action in this function.
  case MCP356xChannel::SE_0:
  case MCP356xChannel::SE_1:
  case MCP356xChannel::SE_2:
  case MCP356xChannel::SE_3:
  case MCP356xChannel::SE_4:
  case MCP356xChannel::SE_5:
  case MCP356xChannel::SE_6:
  case MCP356xChannel::SE_7:
  case MCP356xChannel::DIFF_A:
  case MCP356xChannel::DIFF_B:
  case MCP356xChannel::DIFF_C:
  case MCP356xChannel::DIFF_D:
    break;
  case MCP356xChannel::TEMP:
    // For the TEMP channel, no specific action is taken at the moment.
    break;
  case MCP356xChannel::AVDD:
    _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_AVDD);
    if (!_mcp356x_flag(MCP356X_FLAG_VREF_DECLARED)) {
      // Adjust the reference voltage based on the ADC reading.
      // _vref_plus = nval / (ADC_MAX_VALUE * ADC_VREF_CONVERSION_FACTOR);
    }
    if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
      _mark_calibrated();
    }
    break;
  case MCP356xChannel::VCM:
    _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_VCM);
    if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
      _mark_calibrated();
    }
    break;
  case MCP356xChannel::OFFSET:
    if (0 == setOffsetCalibration(nval)) {
      _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_OFFSET);
    }
    if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
      _mark_calibrated();
    }
    break;
  }
  return 0;
}


/**
 * @brief Retrieve the gain of the ADC.
 *
 * This function returns the gain value of the ADC based on the currently
 * set gain configuration. The returned value represents the multiplication
 * factor applied to the input signal.
 *
 * @return Gain of the ADC as a float. It returns zero if the gain configuration
 * is out-of-bounds or not recognized.
 */
float MCP356x::_gain_value() {
  switch (getGain()) {
  case MCP356xGain::GAIN_ONETHIRD: return 0.33f;
  case MCP356xGain::GAIN_1:        return 1.0f;
  case MCP356xGain::GAIN_2:        return 2.0f;
  case MCP356xGain::GAIN_4:        return 4.0f;
  case MCP356xGain::GAIN_8:        return 8.0f;
  case MCP356xGain::GAIN_16:       return 16.0f;
  case MCP356xGain::GAIN_32:       return 32.0f;
  case MCP356xGain::GAIN_64:       return 64.0f;
  default:                         return 0.0f;  // Default case to handle unexpected values.
  }
}

/**
 * @brief Constructs a control byte for SPI transaction with the ADC.
 *
 * Combines the device address and register address into a single byte formatted
 * for use in an SPI transaction with the ADC. Always sets up for incremental read/write.
 *
 * @param r The MCP356x register address.
 * @return Formatted control byte for SPI transaction.
 */
uint8_t MCP356x::_get_reg_addr(MCP356xRegister r) {
  // Device address is shifted left by 6 positions to make space for the register address and command bits.
  uint8_t devAddrBits = (_DEV_ADDR & 0x03) << 6;

  // Register address is shifted left by 2 positions to make space for the command bits.
  uint8_t regAddrBits = static_cast<uint8_t>(r) << 2;

  // Combine device address, register address, and set for incremental read/write (0x02).
  return devAddrBits | regAddrBits | 0x02;
}

/**
 * @brief Refreshes the local shadows with the current state of the hardware registers.
 *
 * Reads all the hardware registers and updates the shadows. It checks
 * specific reserved register values to determine if an MCP356x device is found
 * and its default states are valid.
 *
 * @return
 *     - -2 if reads worked, but register values don't match defaults.
 *     - -1 if a register read failed.
 *     -  0 if an MCP356x device is found.
 */
int8_t MCP356x::refresh() {
  int8_t ret = 0;
  uint8_t registerIndex = 0;

  // Read all registers and update shadows.
  while ((0 == ret) & (registerIndex < 16)) {
    ret = _read_register(static_cast<MCP356xRegister>(registerIndex++));
  }

  // Continue if all registers were read successfully.
  if (0 == ret) {
    ret = -2;  // Default to "registers don't match defaults."

    // Check the default value of RESERVED0 register.
    if (0x00900000 == reg_shadows[static_cast<uint8_t>(MCP356xRegister::RESERVED0)]) {

      // Extract the RESERVED1 value.
      uint8_t reserved1Value = static_cast<uint8_t>(reg_shadows[static_cast<uint8_t>(MCP356xRegister::RESERVED1)]);

      switch (reserved1Value) {
      case 0x30:
      case 0x50:
        // Check if the chip has an internal Vref and set the corresponding flag.
        _mcp356x_set_flag(MCP356X_FLAG_HAS_INTRNL_VREF, (reserved1Value == 0x30));

        // Validate the presence of the device based on RESERVED2 values.
        switch (reg_shadows[static_cast<uint8_t>(MCP356xRegister::RESERVED2)]) {
        case 0x0C:
        case 0x0D:
        case 0x0F:
          _mcp356x_set_flag(MCP356X_FLAG_DEVICE_PRESENT);
          ret = 0;
          break;
        default:
          // Invalid RESERVED2 value detected.
          // Could use logging here if needed.
          break;
        }
        break;
      default:
        // Invalid RESERVED1 value detected.
        // Could use logging here if needed.
        break;
      }
    }
  }
  return ret;
}


/**
 * @brief Discards ADC samples that are not fully settled.
 *
 * When the analog input is known to be undergoing changes, this function ensures that
 * the ADC discards any samples until they have settled.
 */
void MCP356x::discardUnsettledSamples() {
  _discard_until_micros = getSettlingTime() + _circuit_settle_us + micros();
}

/**
 * @brief Sends a fast command to the MCP356x device.
 *
 * This function initiates an SPI transaction, sends a fast command that isn't
 * related to register access, and then ends the SPI transaction.
 *
 * @param cmd The fast command byte to be sent.
 * @return Always returns 0, indicating successful execution.
 */
int8_t MCP356x::_send_fast_command(uint8_t cmd) {
  // Start the SPI transaction
  _bus->beginTransaction(spi_settings);

  // Pull the CS (Chip Select) pin low to initiate communication with the device
  digitalWrite(_CS_PIN, LOW);

  // Formulate the command byte with the device address and command, then send it
  uint8_t commandByte = (static_cast<uint8_t>(_DEV_ADDR & 0x03) << 6) | cmd;
  _bus->transfer(commandByte);

  // Pull the CS pin high to end communication with the device
  digitalWrite(_CS_PIN, HIGH);

  // End the SPI transaction
  _bus->endTransaction();

  return 0; // Indicating successful execution
}



/**
 * @brief Sets the channels to be scanned.
 *
 * Configures the ADC to scan specific channels in SCAN mode.
 *
 * @param count Number of channels to be set for scanning.
 * @param ... List of channels (of type MCP356xChannel) to be set for scanning.
 * @return Return codes:
 *         -4 if a channel was requested that the driver doesn't support.
 *         -3 if a channel was requested that the hardware doesn't support.
 *         -2 if no channels were given.
 *         -1 on failure to write the SCAN register.
 *         0  on success.
 */
int8_t MCP356x::setScanChannels(int count, ...) {
  if (count <= 0) {
    return -2;
  }

  uint8_t chan_count = _channel_count();
  uint32_t existing_scan = reg_shadows[static_cast<uint8_t>(MCP356xRegister::SCAN)];
  uint32_t chans = 0;
  va_list args;
  va_start(args, count);

  for (int i = 0; i < count; i++) {
    MCP356xChannel chan = va_arg(args, MCP356xChannel);

    // Check channel support based on channel count capability
    switch (chan) {
    case MCP356xChannel::SE_0:
    case MCP356xChannel::SE_1:
    case MCP356xChannel::DIFF_A:
    case MCP356xChannel::TEMP:
    case MCP356xChannel::AVDD:
    case MCP356xChannel::VCM:
    case MCP356xChannel::OFFSET:
      if (2 > chan_count) {
        va_end(args);
        return -3;    // Not supported by current hardware configuration
      }
      break;
    case MCP356xChannel::SE_2:
    case MCP356xChannel::SE_3:
    case MCP356xChannel::DIFF_B:
      if (4 > chan_count) {
        va_end(args);
        return -3;    // Not supported by current hardware configuration
      }
      break;
    case MCP356xChannel::SE_4:
    case MCP356xChannel::SE_5:
    case MCP356xChannel::SE_6:
    case MCP356xChannel::SE_7:
    case MCP356xChannel::DIFF_C:
    case MCP356xChannel::DIFF_D:
      if (8 != chan_count) {
        va_end(args);
        return -3;    // Not supported by current hardware configuration
      }
      break;
    default:
      va_end(args);
      return -4;        // Channel is not recognized by the driver
    }

    chans |= (1 << static_cast<uint8_t>(chan));
  }
  va_end(args);

  // Mask to preserve the upper 16 bits of a 32-bit number.
  const uint32_t UPPER_HALF_MASK = 0xFFFF0000;

  // Preserve upper bits of the existing scan register and combine with selected channels
  chans |= (existing_scan & UPPER_HALF_MASK);
  _channel_backup = chans;

  // If the device is calibrated, set scan channels
  if (_mcp356x_flag(MCP356X_FLAG_CALIBRATED)) {
    return _set_scan_channels(chans);
  }

  return 0;
}


/**
 * @brief Set the channels to be scanned by the ADC.
 *
 * This function sets the channels that the MCP356x ADC should scan. Before setting,
 * it checks if the ADC has been calibrated. If not, the current scan configuration is backed up.
 *
 * @param rval The scan configuration to be set.
 *
 * @return The result of the write operation to the SCAN register.
 */
int8_t MCP356x::_set_scan_channels(uint32_t rval) {
  // If the ADC is not calibrated, back up the current scan configuration.
  if (!_mcp356x_flag(MCP356X_FLAG_CALIBRATED)) {
    _channel_backup = reg_shadows[static_cast<uint8_t>(MCP356xRegister::SCAN)];
  }

  // Write the new scan configuration to the SCAN register and return the result.
  return _write_register(MCP356xRegister::SCAN, rval);
}


/**
 * @brief Checks if data for all requested channels is available.
 *
 * This function examines the scanned channels as per the SCAN register and
 * ensures that all the requested channels have valid data.
 *
 * @return Returns 'true' if data for all requested channels is available, otherwise 'false'.
 */
bool MCP356x::scanComplete() {
  // Extract the channels that were set to be scanned from the SCAN register.
  uint32_t scan_chans = reg_shadows[static_cast<uint8_t>(MCP356xRegister::SCAN)] & 0x0000FFFF;

  // Compare the channels we have data for (_channel_flags) against the ones we set to be scanned.
  // If they match, then data for all scanned channels is available.
  return (scan_chans == (_channel_flags & scan_chans));
}


/**
 * @brief Set the reference voltage range for the ADC.
 *
 * In some hardware arrangements, the ADC doesn't use a rail-to-rail Vref.
 * This function allows the application to manually define the reference voltage range.
 *
 * @param plus Positive reference voltage.
 * @param minus Negative reference voltage.
 *
 * @return Always returns 0.
 */
int8_t MCP356x::setReferenceRange(float plus, float minus) {
  _vref_plus = plus;
  _vref_minus = minus;
  _mcp356x_set_flag(MCP356X_FLAG_VREF_DECLARED);
  return 0;
}

/**
 * @brief Calculate the die temperature.
 *
 * Computes the current temperature value using the temperature transfer function.
 * The computation can use either a third-order fit or a simple linear equation
 * depending on the MCP356X_FLAG_3RD_ORDER_TEMP flag.
 *
 * @return The calculated temperature in degrees Celsius.
 */
float MCP356x::getTemperature() {
  int32_t t_lsb = value(MCP356xChannel::TEMP);
  float ret = 0.0;

  if (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP)) {
    // Use the third-order polynomial fit for high-accuracy temperature calculation.
    const double k1 = 0.0000000000000271 * pow(t_lsb, 3);
    const double k2 = -0.000000018 * pow(t_lsb, 2);
    const double k3 = 0.0055 * t_lsb;
    const double k4 = -604.22;
    ret = k1 + k2 + k3 + k4;
  }
  else {
    // Use a simple linear equation for temperature calculation.
    ret = 0.001581 * t_lsb - 324.27;
  }

  return ret;
}


/*******************************************************************************
* Hardware discovery functions
*******************************************************************************/

/**
 * @brief Checks if the MCLK frequency is within the operational boundaries.
 *
 * The MCP356x ADC requires its input clock (MCLK) to be between 1MHz and 20MHz.
 * This function verifies if the current MCLK frequency is within these boundaries.
 *
 * @return True if MCLK is within operational boundaries, otherwise false.
 */
bool MCP356x::_mclk_in_bounds() {
  DBG_PRINT(String("Current MCLK frequency: ") + _mclk_freq + " Hz");

  // Check if the MCLK frequency is within the acceptable range.
  return ((_mclk_freq > 1e6) && (_mclk_freq < 2e7));
}

/**
 * @brief Detect the frequency of the ADC's clock.
 *
 * This function measures the frequency of the ADC's clock by performing ADC reads with specific timing parameters.
 * The result is stored in the class variable `_mclk_freq`.
 *
 * @note Ensure the ADC pins are properly configured before invoking this function.
 *
 * @return
 *     - \c -3 if the class is not ready for this measurement (pins not configured).
 *     - \c -2 if the measurement timed out (couldn't obtain enough samples within time limits).
 *     - \c -1 if there was an issue communicating with the ADC.
 *     - \c 0 if a clock signal within the expected range was determined.
 *     - \c 1 if the clock rate was out of bounds or nonsensical.
 */
int8_t MCP356x::_detect_adc_clock() {
  const uint32_t SAMPLE_TIME_MAX = 5000000;
  const uint32_t SAMPLE_TIME_MIN = 100000;
  int8_t ret = -3;

  DBG_PRINT("Starting ADC clock detection...");

  if (!_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)) {
    DBG_PRINT("Pins are not configured.");
    DBG_PRINT("Exiting ADC clock detection.");
    return ret;
  }

  DBG_PRINT("Pins are configured.");

  if (0 != _write_register(MCP356xRegister::SCAN, 0)) {
    DBG_PRINT("Failed to write to SCAN register.");
    DBG_PRINT("Exiting ADC clock detection.");
    return -1;
  }
  DBG_PRINT("SCAN register written successfully.");

  if (0 != _write_register(MCP356xRegister::MUX, 0xDE)) {
    DBG_PRINT("Failed to write to MUX register.");
    DBG_PRINT("Exiting ADC clock detection.");
    return -1;
  }
  DBG_PRINT("MUX register written successfully.");

  unsigned long start_time = micros();
  uint16_t read_count = 0;

  while (((read_count < 10000) || ((micros() - start_time) < SAMPLE_TIME_MIN)) &&
    ((micros() - start_time) < SAMPLE_TIME_MAX)) {

    if (isr_fired && (0 < read())) {
      if (read_count == 0) {
        resetReadCount();
        DBG_PRINT("Resetting read count.");
      }
      read_count++;
    }
  }

  if ((micros() - start_time) >= SAMPLE_TIME_MAX) {
    DBG_PRINT("Sampling timed out.");
    DBG_PRINT("Exiting ADC clock detection.");
    return -2;
  }

  DBG_PRINT("ADC sampling completed successfully within time limits.");

  _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
  _mclk_freq = _calculate_input_clock(micros() - start_time);

  if (!_mclk_in_bounds()) {
    DBG_PRINT("MCLK frequency is out of bounds.");
    DBG_PRINT("Exiting ADC clock detection.");
    return 1;
  }

  DBG_PRINT("MCLK frequency is within bounds.");
  _recalculate_clk_tree();

  DBG_PRINT("Exiting ADC clock detection.");
  return 0;
}


/**
 * @brief Calculate the true rate of the input clock after the ADC has been running for a while.
 *
 * After the ADC has been running for a while, this function calculates the true rate of the input clock
 * if it is not already known. It uses the elapsed time in microseconds and the number of samples taken
 * to determine the input clock frequency.
 *
 * @note Since the sample count doesn't reset when timing parameters are altered, this function provides
 *       accurate results only if the settings are unchanged from initialization, or the caller has reset
 *       the read count using `resetReadCount()` before taking the measurement.
 *
 * @param elapsed_us The elapsed time in microseconds since the ADC started running.
 * @return The calculated input clock frequency.
 */
double MCP356x::_calculate_input_clock(unsigned long elapsed_us) {
  DBG_PRINT(" - Starting input clock calculation...");

  // Constants for CONFIG1 register processing.
  constexpr uint32_t OSR_INDEX_MASK = 0x0000003C;
  constexpr uint32_t OSR_INDEX_SHIFT = 2;
  constexpr uint32_t PRE_VALUE_MASK = 0x000000C0;
  constexpr uint32_t PRE_VALUE_SHIFT = 6;

  // Extract the OSR index from the CONFIG1 register shadow
  uint32_t osr_idx = (reg_shadows[static_cast<uint8_t>(MCP356xRegister::CONFIG1)] & OSR_INDEX_MASK) >> OSR_INDEX_SHIFT;
  DBG_PRINT(String("OSR index extracted: ") + osr_idx);

  // Retrieve the corresponding OSR values from the arrays
  uint16_t osr1 = OSR1_VALUES[osr_idx];
  uint16_t osr3 = OSR3_VALUES[osr_idx];
  DBG_PRINT(String("OSR1 value: ") + osr1);
  DBG_PRINT(String("OSR3 value: ") + osr3);

  // Extract the pre-value from the CONFIG1 register shadow
  uint32_t pre_val = (reg_shadows[static_cast<uint8_t>(MCP356xRegister::CONFIG1)] & PRE_VALUE_MASK) >> PRE_VALUE_SHIFT;
  DBG_PRINT(String("Pre-value extracted: ") + pre_val);

  // Calculate DRCLK (Data Rate Clock)
  double _drclk = static_cast<double>(read_count) / elapsed_us * 1000000.0;  // Conversion to Hz
  DBG_PRINT(String("Calculated DRCLK: ") + _drclk);

  // Calculate and return the input clock value
  double calculated_input_clock = (4 * (osr3 * osr1) * (1 << pre_val) * _drclk);
  DBG_PRINT(String("Calculated input clock: ") + calculated_input_clock);

  DBG_PRINT("Exiting input clock calculation.");
  return calculated_input_clock;
}


/**
 * @brief Recalculate the DRCLK and settling time based on the MCLK frequency.
 *
 * Given the MCLK frequency, this function calculates the DRCLK (Data Rate Clock) frequency and stores
 * it locally in the `_dmclk_freq` variable. It also recalculates the settling time based on the new
 * DRCLK frequency.
 *
 * @return -2 if the MCLK frequency is out-of-bounds, 0 if the calculation completed successfully.
 */
int8_t MCP356x::_recalculate_clk_tree() {
  DBG_PRINT(" - Entering _recalculate_clk_tree...");

  if (_mclk_in_bounds()) {
    DBG_PRINT("MCLK is within bounds.");

    // Extract the pre-scaler value from the CONFIG1 register.
    uint32_t pre_val = (reg_shadows[static_cast<uint8_t>(MCP356xRegister::CONFIG1)] & 0x000000C0) >> 6;

    DBG_PRINT(String("Pre-value from CONFIG1: ") + pre_val);

    // Calculate the DRCLK frequency based on the MCLK and pre-scaler value.
    _dmclk_freq = _mclk_freq / (4 * (1 << pre_val));

    DBG_PRINT(String("Calculated _dmclk_freq: ") + _dmclk_freq);
    DBG_PRINT("Recalculating settling time...");

    // Recalculate the settling time based on the new DRCLK frequency.
    return _recalculate_settling_time();
  }

  DBG_PRINT("MCLK is out of bounds. Exiting _recalculate_clk_tree with status -2.");

  return -2;  // Indicate that MCLK is out of the supported bounds.
}


/**
 * @brief Recalculate the ADC's settling time based on its OSR settings and DMCLK frequency.
 *
 * The function uses values from the MCP356x datasheet, specifically from
 * Section 5 - "Conversion Time Calculation". It calculates the settling time
 * based on the Over Sampling Rate (OSR) settings from the CONFIG1 register
 * and the DMCLK frequency.
 *
 * @return Always returns 0 (indicating successful calculation).
 */
int8_t MCP356x::_recalculate_settling_time() {
  DBG_PRINT(" - Recalculating settling time...");

  // Extract the OSR index from the CONFIG1 register shadow.
  uint32_t osr_idx = (reg_shadows[static_cast<uint8_t>(MCP356xRegister::CONFIG1)] & 0x0000003C) >> 2;

  // Retrieve the corresponding OSR values.
  uint16_t osr1 = OSR1_VALUES[osr_idx];
  uint16_t osr3 = OSR3_VALUES[osr_idx];

  // Calculate dmclks.
  uint32_t dmclks = (3 * osr3) + ((osr1 - 1) * osr3);

  // Calculate settling time in microseconds.
  _settling_us = (1000000.0 * dmclks) / _dmclk_freq;

  return 0;
}


/**
 * @brief Calibrates the ADC's offset using special channels.
 *
 * Calibration relies on specific ADC channels, as detailed in the MCP356x datasheet.
 *
 * @note This function will change the SCAN register settings for calibration and clear
 * the MCP356X_FLAG_CALIBRATED flag if calibration is successful.
 *
 * @return -1 if register I/O failed, 0 if an offset calibration value was found and set.
 */
int8_t MCP356x::_calibrate_offset() {

  // Backup current SCAN register settings
  _channel_backup = reg_shadows[static_cast<uint8_t>(MCP356xRegister::SCAN)];

  // Set SCAN register for calibration channels
  int8_t ret = _set_scan_channels(0x0000E000);

  // If the SCAN setting was successful, clear the calibrated flag
  if (0 == ret) {
    _mcp356x_clear_flag(MCP356X_FLAG_CALIBRATED);
  }

  return ret;
}

/**
 * @brief Mark the MCP356x ADC as calibrated.
 *
 * This function attempts to restore the original scan channels and, if successful,
 * marks the MCP356x ADC as calibrated.
 *
 * @return Returns 0 if successfully marked as calibrated, otherwise returns the error code from _set_scan_channels().
 */
int8_t MCP356x::_mark_calibrated() {
  int8_t ret = _set_scan_channels(_channel_backup);
  if (0 == ret) {
    _mcp356x_set_flag(MCP356X_FLAG_CALIBRATED);
  }
  return ret;
}

/**
 * @brief Prints the current state of MCP356x's registers.
 *
 * This function fetches the values from the shadow registers and prints them
 * in a formatted manner. It is primarily used for debugging purposes.
 *
 * @param output A pointer to the StringBuilder object to store the formatted register values.
 */
void MCP356x::printRegs(StringBuilder* output) {
  output->concatf("reg_shadows[0] (ADCDATA)     = 0x%08x\n", reg_shadows[0]);
  output->concatf("reg_shadows[1] (CONFIG0)     = 0x%02x\n", reg_shadows[1]);
  output->concatf("reg_shadows[2] (CONFIG1)     = 0x%02x\n", reg_shadows[2]);
  output->concatf("reg_shadows[3] (CONFIG2)     = 0x%02x\n", reg_shadows[3]);
  output->concatf("reg_shadows[4] (CONFIG3)     = 0x%02x\n", reg_shadows[4]);
  output->concatf("reg_shadows[5] (IRQ)         = 0x%02x\n", reg_shadows[5]);
  output->concatf("reg_shadows[6] (MUX)         = 0x%02x\n", reg_shadows[6]);
  output->concatf("reg_shadows[7] (SCAN)        = 0x%06x\n", reg_shadows[7]);
  output->concatf("reg_shadows[8] (TIMER)       = 0x%06x\n", reg_shadows[8]);
  output->concatf("reg_shadows[9] (OFFSETCAL)   = 0x%06x\n", reg_shadows[9]);
  output->concatf("reg_shadows[10] (GAINCAL)    = 0x%06x\n", reg_shadows[10]);
  output->concatf("reg_shadows[11] (RESERVED0)  = 0x%06x\n", reg_shadows[11]);
  output->concatf("reg_shadows[12] (RESERVED1)  = 0x%02x\n", reg_shadows[12]);
  output->concatf("reg_shadows[13] (LOCK)       = 0x%02x\n", reg_shadows[13]);
  output->concatf("reg_shadows[14] (RESERVED2)  = 0x%04x\n", reg_shadows[14]);
  output->concatf("reg_shadows[15] (CRCCFG)     = 0x%04x\n", reg_shadows[15]);
}


/**
 * @brief Prints the pin configuration of the MCP356x to the provided output.
 *
 * This function provides an easy way to visualize the pin setup of the MCP356x.
 *
 * @param output Pointer to a StringBuilder object where the pin configuration will be printed.
 */
void MCP356x::printPins(StringBuilder* output) {
  output->concatf("IRQ:   %u\n", _IRQ_PIN);
  output->concatf("CS:    %u\n", _CS_PIN);
  output->concatf("MCLK:  %u\n", _MCLK_PIN);
}

/**
 * @brief Prints the timing details of the MCP356x to the provided output.
 *
 * This function provides an overview of various timing and frequency metrics
 * related to the operation of the MCP356x.
 *
 * @param output Pointer to a StringBuilder object where the timing details will be printed.
 */
void MCP356x::printTimings(StringBuilder* output) {
  output->concatf("\tMCLK                = %.4f MHz\n", _mclk_freq / 1000000.0);
  output->concatf("\tDMCLK               = %.4f MHz\n", _dmclk_freq / 1000000.0);
  output->concatf("\tData rate           = %.4f KHz\n", _drclk_freq / 1000.0);
  //output->concatf("\tReal sample rate    = %u\n", reads_per_second);
  output->concatf("\tADC settling time   = %u\n", getSettlingTime());
  output->concatf("\tTotal settling time = %u\n", _circuit_settle_us);
  output->concatf("\tLast read (micros)  = %u\n", micros_last_read);
}


/**
 * @brief Prints the data of the MCP356x ADC.
 *
 * This function outputs the current state and configuration of the MCP356x ADC.
 * Various parameters, such as channel count, calibration status, gain, and more
 * are printed to the provided StringBuilder object.
 *
 * @param output Pointer to the StringBuilder object to which data will be appended.
 */
void MCP356x::printData(StringBuilder* output) {

  // Constructing a product string based on the ADC's state and channel count.
  StringBuilder prod_str("MCP356");
  if (adcFound()) {
    prod_str.concatf("%d", _channel_count() >> 1);
    if (hasInternalVref()) prod_str.concat('R');
  }
  else {
    prod_str.concat("x (not found)");
  }

  // Append the product string as a header.
  StringBuilder::styleHeader2(output, (const char*)prod_str.string());

  // If the ADC is found, print its configuration and status.
  if (adcFound()) {
    output->concatf("\tChannels:       %u\n", _channel_count());
    output->concatf("\tClock running:  %c\n", (_mcp356x_flag(MCP356X_FLAG_MCLK_RUNNING) ? 'y' : 'n'));
    output->concatf("\tConfigured:     %c\n", (adcConfigured() ? 'y' : 'n'));
    output->concatf("\tCalibrated:     %c\n", (adcCalibrated() ? 'y' : 'n'));
    if (adcCalibrated()) {
      output->concat("\t");
      printChannel(MCP356xChannel::OFFSET, output);
      output->concat("\t");
      printChannel(MCP356xChannel::VCM, output);
      output->concat("\t");
      printChannel(MCP356xChannel::AVDD, output);
    }
    else {
      output->concatf("\t  SAMPLED_OFFSET: %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_OFFSET) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_VCM:    %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_VCM) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_AVDD:   %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_AVDD) ? 'y' : 'n'));
    }
    output->concatf("\tCRC Error:      %c\n", (_mcp356x_flag(MCP356X_FLAG_CRC_ERROR) ? 'y' : 'n'));
    output->concatf("\tisr_fired:      %c\n", (isr_fired ? 'y' : 'n'));
    output->concatf("\tRead count:     %u\n", read_count);
    output->concatf("\tGain:           x%.2f\n", _gain_value());
    uint8_t _osr_idx = static_cast<uint8_t>(getOversamplingRatio());
    output->concatf("\tOversampling:   x%u\n", OSR1_VALUES[_osr_idx] * OSR3_VALUES[_osr_idx]);
    output->concatf("\tVref source:    %sternal\n", (usingInternalVref() ? "In" : "Ex"));
    output->concatf("\tVref declared:  %c\n", (_vref_declared() ? 'y' : 'n'));
    output->concatf("\tVref range:     %.3f / %.3f\n", _vref_minus, _vref_plus);
    output->concatf("\tClock SRC:      %sternal\n", (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK) ? "In" : "Ex"));
    if (_scan_covers_channel(MCP356xChannel::TEMP)) {
      output->concatf("\tTemperature:    %.2fC\n", getTemperature());
      output->concatf("\tThermo fitting: %s\n", (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP) ? "3rd-order" : "Linear"));
    }
  }
}


/**
 * @brief Prints the voltage value of a specific channel.
 *
 * This function displays the reading of the specified channel with its name.
 * If the channel reading is over the range, "OvR" will be displayed next to the voltage value.
 *
 * @param chan   The MCP356x channel for which the voltage value needs to be printed.
 * @param output Pointer to a StringBuilder object where the formatted output will be constructed.
 */
void MCP356x::printChannel(MCP356xChannel chan, StringBuilder* output) {
  // Fetch the channel name, convert the ADC value to voltage, and check if the value is over the range.
  // These values are then formatted into a string.
  output->concatf(
    "%s:\t%.6fv\t%s\n",
    CHAN_NAMES[static_cast<uint8_t>(chan) & 0x0F],
    valueAsVoltage(chan),
    _channel_over_range(chan) ? "OvR" : " "
  );
}


/**
 * @brief Prints the values of all enabled channels.
 *
 * Iterates over all channels and, if they are enabled for scanning, displays their readings.
 * It provides special treatment for the temperature channel by displaying the temperature value.
 * The VCM channel value is also printed when applicable.
 *
 * @param output Pointer to a StringBuilder object to construct the output string for each channel.
 * @param asVoltage If true, display values converted to voltage. If false, display raw ADC values.
 */
void MCP356x::printChannelValues(StringBuilder* output, bool asVoltage) {
  // Iterate over all possible channel enum values
  for (uint8_t i = static_cast<uint8_t>(MCP356xChannel::SE_0);
    i <= static_cast<uint8_t>(MCP356xChannel::VCM); i++) {

    MCP356xChannel chan = static_cast<MCP356xChannel>(i);

    // Check if the current channel is enabled for scanning
    if (_scan_covers_channel(chan)) {
      switch (chan) {
      case MCP356xChannel::TEMP:
        // Special handling for temperature channel
        output->concatf("Die temperature     = %.2fC\n", getTemperature());
        break;
      default:
        // For all other channels, including VCM
        if (asVoltage) {
          // Display values in voltage format
          output->concatf(
            "%s:\t%.6fv\t%s\n",
            CHAN_NAMES[i & 0x0F],
            valueAsVoltage(chan),
            _channel_over_range(chan) ? "OvR" : " "
          );
        }
        else {
          // Display raw ADC values
          output->concatf(
            "%s:\t%d\t%s\n",
            CHAN_NAMES[i & 0x0F],
            value(chan),  // Assuming there's a function named `value()` to get raw values.
            _channel_over_range(chan) ? "OvR" : " "
          );
        }
        break;
      }
    }
  }
}

