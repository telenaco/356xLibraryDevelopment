/**
 * @file MCP356x.cpp
 * @brief Implementation of the MCP356x class for interfacing with MCP356x ADCs
 *
 * This file contains the implementation of the MCP356x class methods for configuring,
 * calibrating, and reading data from MCP356x analog-to-digital converters. It includes
 * low-level SPI communication, register manipulation, and data processing functions.
 *
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

#include "MCP356x.h"

// Static array to store pointers to MCP356x instances for interrupt handling
volatile static MCP356x *INSTANCES[MCP356X_MAX_INSTANCES] = {nullptr, nullptr, nullptr};

// SPI settings: 20MHz max clock rate, MSB first, SPI mode 0
static SPISettings spi_settings(20000000, MSBFIRST, SPI_MODE0);

// Register widths in bytes, indexed by register address
static const uint8_t MCP356x_reg_width[16] = {1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 1, 1, 2, 2};

// Oversampling ratio (OSR) values for various configurations
static const uint16_t OSR1_VALUES[16] = {1, 1, 1, 1, 1, 2, 4, 8, 16, 32, 40, 48, 80, 96, 160, 192};
static const uint16_t OSR3_VALUES[16] = {32, 64, 128, 256, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512};

// Human-readable names for ADC channels
static const char *CHAN_NAMES[16] = {"SE_0", "SE_1", "SE_2", "SE_3", "SE_4", "SE_5", "SE_6", "SE_7",
                                     "DIFF_A", "DIFF_B", "DIFF_C", "DIFF_D", "TEMP", "AVDD", "VCM", "OFFSET"};

// Interrupt Service Routines (ISRs) for MCP356x instances
void mcp356x_isr0()
{
    if (INSTANCES[0])
        INSTANCES[0]->isr_fired = true;
}
void mcp356x_isr1()
{
    if (INSTANCES[1])
        INSTANCES[1]->isr_fired = true;
}
void mcp356x_isr2()
{
    if (INSTANCES[2])
        INSTANCES[2]->isr_fired = true;
}

/*******************************************************************************
 * MCP356x Constructors, Destructor, and Initialization Functions
 *
 * This section contains constructors, destructor, and other functions for initializing
 * and configuring the MCP356x class. It includes methods for setting up the ADC, configuring
 * channels, oversampling ratios, and other ADC parameters.
 *******************************************************************************/

/**
 * @brief Constructor for the MCP356x class.
 *
 * Initializes an instance of the MCP356x class with a specified configuration.
 * It assigns the ADC to a free slot if available and sets up pin configurations.
 *
 * @param config Configuration structure containing pin assignments and device address.
 */
MCP356x::MCP356x(const MCP356xConfig &config)
    : _IRQ_PIN(config.irq_pin), _CS_PIN(config.cs_pin), _MCLK_PIN(config.mclk_pin), _DEV_ADDR(config.addr)
{

    bool unslotted = true;
    for (uint8_t i = 0; i < MCP356X_MAX_INSTANCES; i++)
    {
        if (unslotted)
        {
            if (nullptr == INSTANCES[i])
            {
                _slot_number = i;
                INSTANCES[_slot_number] = this;
                unslotted = false;
            }
        }
    }

    if (unslotted)
    { // If still true, no slot was found.
        Serial.println("Error: No free slots for MCP356x instances.");
    }

    // Setup ADC functionality configuration
    setOption(MCP356X_FLAG_USE_INTERNAL_CLK); // internal clock is default
    init(config.spiInterface);
    setGain(config.gain);
    setOversamplingRatio(config.osr);
    setADCMode(config.mode);
    setSpecificChannels(config.numChannels);
}

/**
 * @brief Destructor for the MCP356x class.
 *
 * Detaches the interrupt associated with the MCP356x instance and
 * clears its slot, allowing for other instances to be created.
 */
MCP356x::~MCP356x()
{
    if (255 != _IRQ_PIN)
    {
        detachInterrupt(digitalPinToInterrupt(_IRQ_PIN));
    }
    INSTANCES[_slot_number] = nullptr;
}

/**
 * @brief Configures specific ADC channels for scanning.
 *
 * Sets up to 4 differential channels based on the specified number of channels.
 *
 * @param numChannels Number of channels to set (1-4).
 */
void MCP356x::setSpecificChannels(int numChannels)
{
    MCP356xChannel channels[] = {MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D};

    // Ensure numChannels does not exceed the size of the channels array
    numChannels = (numChannels > 4) ? 4 : numChannels;

    switch (numChannels)
    {
    case 1:
        setScanChannels(1, channels[0]);
        break;
    case 2:
        setScanChannels(2, channels[0], channels[1]);
        break;
    case 3:
        setScanChannels(3, channels[0], channels[1], channels[2]);
        break;
    case 4:
        setScanChannels(4, channels[0], channels[1], channels[2], channels[3]);
        break;
    default:
        // Possibly handle error or invalid input
        break;
    }
}

/**
 * @brief Resets the MCP356x ADC.
 *
 * Sends a fast command to reset the ADC. Ensures the SPI bus is properly reset by
 * toggling the chip select (CS) line. Then clears the registers and refreshes them.
 *
 * @return int8_t
 *         - -2 if refresh found unexpected values.
 *         - -1 if there was a problem writing the reset command or refreshing the registers.
 *         - 0 if reset command was sent successfully.
 */
int8_t MCP356x::reset()
{
    int8_t ret = _send_fast_command(0x38);
    if (0 == ret)
    {
        delayMicroseconds(1); // <--- Arbitrary delay value
        digitalWrite(_CS_PIN, 0);
        digitalWrite(_CS_PIN, 1);
        digitalWrite(_CS_PIN, 0);
        digitalWrite(_CS_PIN, 1); // Twiddle the CS line to ensure SPI reset.
        ret = _clear_registers();
        if (0 == ret)
        {
            delayMicroseconds(1); // <--- Arbitrary delay value
            ret = refresh();
        }
    }
    return ret;
}

/**
 * @brief Initializes the MCP356x ADC.
 *
 * Sets up the chip, configures pins, and performs initial calibration.
 * Performs various checks and calibrations, including clock detection and offset calibration.
 *
 * @param b Pointer to the SPI bus object.
 *
 * @return int8_t
 *         - -7 if offset calibration failed.
 *         - -6 if clock detection failed.
 *         - -5 if register refresh failed.
 *         - -4 if pin setup failed.
 *         - -3 if bad bus reference.
 *         - -2 if reset failed.
 *         - -1 if clock detection or calibration failed for some reason.
 *         - 0 on success.
 */
int8_t MCP356x::init(SPIClass *b)
{
    bool debug = false; // Set this to true when you want to see debug outputs

    int8_t pin_setup_ret = _ll_pin_init(); // Configure the pins if they are not already.

    int8_t ret = -4;
    if (pin_setup_ret >= 0)
    {
        ret = -3;
        if (nullptr != b)
        {
            ret = -2;
            _bus = b;

            if (0 == reset())
            {
                ret = -5;
                if (0 == _post_reset_fxn())
                {
                    if (!_mclk_in_bounds())
                    {
                        ret = -6; //  -6 if clock detection failed.
                        int8_t det_ret = _detect_adc_clock();
                        if (debug)
                        { // Only enter the switch statement if debug is true
                            switch (det_ret)
                            {
                            case -3:
                                Serial.println("-> Class isn't ready for clock measurement.");
                                break;
                            case -2:
                                Serial.println("-> Clock measurement timed out.");
                                break;
                            case -1:
                                Serial.println("-> Problem communicating with the ADC.");
                                break;
                            case 0:
                                Serial.println("-> Clock signal within expected range determined.");
                                break;
                            case 1:
                                Serial.println("-> Clock rate out of bounds or appears nonsensical.");
                                break;
                            default:
                                Serial.println("Unknown return from _detect_adc_clock.");
                                break;
                            }
                        }
                    }
                    if (_mclk_in_bounds())
                    {
                        switch (_calibrate_offset())
                        {
                        case 0:
                            ret = 0;
                            if (debug)
                            {
                                Serial.println("Offset calibration successful.");
                            }
                            break;
                        default:
                            ret = -7;
                            if (debug)
                            {
                                Serial.println("Offset calibration failed.");
                            }
                            break;
                        }
                    }
                }
                else
                {
                    if (debug)
                    {
                        Serial.println("Post reset function failed.");
                    }
                }
            }
            else
            {
                if (debug)
                {
                    Serial.println("ADC Reset failed.");
                }
            }
        }
        else
        {
            if (debug)
            {
                Serial.println("SPI Bus is nullptr.");
            }
        }
    }
    else
    {
        if (debug)
        {
            Serial.println("Pin setup failed.");
        }
    }

    if (debug)
    {
        Serial.print("Exiting MCP356x::init with return value: ");
        Serial.println(ret);
    }
    return ret;
}

/**
 * @brief Sets specialized features of the MCP356x chip.
 *
 * Configures the MCP356x with a combination of flags defined in the header file.
 * Should be called before the first `init()` to avoid conflicts with ADC configuration.
 *
 * @param flgs Combination of flags for setting options.
 *
 * @return int8_t
 *         - -1 if called after pin configuration, for options that require pre-configuration.
 *         - 0 on successful setting of options.
 */
int8_t MCP356x::setOption(uint32_t flgs)
{
    int8_t ret = 0;
    if (flgs & MCP356X_FLAG_USE_INTERNAL_CLK)
    {
        if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)))
        {
            // Only allow this change if the pins are not yet configured.
            _mcp356x_set_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
            _mcp356x_clear_flag(MCP356X_FLAG_GENERATE_MCLK);
        }
        else
        {
            ret = -1;
        }
    }
    if (flgs & MCP356X_FLAG_GENERATE_MCLK)
    {
        if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)))
        {
            // Only allow this change if the pins are not yet configured.
            _mcp356x_set_flag(MCP356X_FLAG_GENERATE_MCLK);
            _mcp356x_clear_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
        }
        else
        {
            ret = -1;
        }
    }
    if (flgs & MCP356X_FLAG_3RD_ORDER_TEMP)
    {
        _mcp356x_set_flag(MCP356X_FLAG_3RD_ORDER_TEMP);
    }
    return ret;
}

/**
 * @brief Sets the burnout current sources configuration for the MCP356x.
 *
 * Configures the burnout current sources based on the provided setting.
 *
 * @param config The desired burnout current source configuration.
 *
 * @return int8_t Status of the register write operation.
 */
int8_t MCP356x::setBurnoutCurrentSources(MCP356xBurnoutCurrentSources config)
{
    uint32_t config0_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG0];
    config0_val = (config0_val & 0xFFFCFFFF) | (((uint8_t)config & 0x03) << 16);
    return _write_register(MCP356xRegister::CONFIG0, config0_val);
}

/**
 * @brief Enables the burnout current sources at maximum current.
 *
 * Sets burnout current sources to 15µA.
 */
void MCP356x::enableBurnoutCurrentSources()
{
    setBurnoutCurrentSources(MCP356xBurnoutCurrentSources::CURRENT_15_UA);
}

/**
 * @brief Disables the burnout current sources.
 *
 * Turns off all burnout current sources.
 */
void MCP356x::disableBurnoutCurrentSources()
{
    setBurnoutCurrentSources(MCP356xBurnoutCurrentSources::DISABLED);
}

/**
 * @brief Configures the MCP356x after a reset.
 *
 * Sets up necessary configurations after the MCP356x chip has been reset.
 * Handles register writing for IRQ settings, clock source selection, and data representation.
 *
 * @return int8_t
 *         - -1 on failure to write a register.
 *         - 0 on successful configuration post-reset.
 */
int8_t MCP356x::_post_reset_fxn()
{
    int8_t ret = -1;
    uint32_t c0_val = 0x000000C3;

    // Enable fast command, disable IRQ on conversion start, IRQ pin is
    // open-drain.
    ret = _write_register(MCP356xRegister::IRQ, 0x00000002);
    if (0 == ret)
    {
        if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK))
        {
            // Set CLK_SEL to use internal clock with no pin output.
            c0_val &= 0xFFFFFFCF;
            c0_val |= 0x00000020;
        }
        ret = _write_register(MCP356xRegister::CONFIG0, c0_val);
        if (0 == ret)
        {
            if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK))
            {
                _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
            }
            // For simplicity, we select a 32-bit sign-extended data representation
            // with channel identifiers.
            ret = _write_register(MCP356xRegister::CONFIG3, 0x000000F0);
            if (0 == ret)
            {
                _mcp356x_set_flag(MCP356X_FLAG_INITIALIZED);
            }
        }
    }
    return ret;
}

/**
 * @brief Reads the output register of the MCP356x and processes the data.
 *
 * Handles reading of the ADC data register, performing necessary data manipulation
 * according to the current configuration. Manages data validity and discards data
 * if within the declared settling time.
 *
 * @return int8_t
 *         - -1 on error reading from registers.
 *         - 0 on successful read with no action required.
 *         - 1 if data was ready and read but discarded due to settling time.
 *         - 2 if the read resulted in a full set of data for all channels.
 */
int8_t MCP356x::read()
{
    // Process the interrupt register and get the status
    int8_t ret = _proc_irq_register();

    switch (ret)
    {
    case -1:
        break;
    case 0:
        break;
        // If the IRQ register indicates data is ready
    case 1:
        // Read the ADC data register and save it on the corresponding
        // microcontroller register declared on this class
        _read_register(MCP356xRegister::ADCDATA);

        // Record the current microsecond timestamp
        _micros_last_read = micros();

        // Check if the read data should be processed or discarded
        if (_discard_until_micros <= _micros_last_read)
        {
            // Process and format the retrieved data
            _normalize_data_register();

            // Verify if scanning across all channels has been completed
            if (scanComplete())
            {
                ret = 2;
            }
        }

        _read_count++;
        _read_accumulator++;

        if (_micros_last_read - _micros_last_window >= 1000000)
        {
            _micros_last_window = _micros_last_read;
            _reads_per_second = _read_accumulator;
            _read_accumulator = 0;
        }
        break;
    default:
        break;
    }

    return ret;
}

/**
 * @brief Checks for updated readings from the ADC and processes them.
 *
 * Monitors the interrupt flag and calls read() to process any new data.
 *
 * @return int
 *         - 1 if new data has been updated on the local register.
 *         - 0 if no new data is available.
 */
int MCP356x::updatedReadings()
{
    if (isr_fired && (2 == read()))
    {
        return 1; // new data has been updated on the local register.
    }
    return 0;
}

/**
 * @brief Processes the IRQ register and responds to its contents.
 *
 * Reads the IRQ register of the MCP356x and takes actions based on its contents.
 * Handles conversion completion, power-on-reset events, and CRC configuration errors.
 *
 * @return int8_t
 *         - -1 if reading the IRQ register failed.
 *         - 0 if there is nothing to do after reading.
 *         - 1 if the data register needs to be read.
 */
int8_t MCP356x::_proc_irq_register()
{
    int8_t ret = -1;
    if (0 == _read_register(MCP356xRegister::IRQ))
    {
        ret = 0;
        uint8_t irq_reg_data = (uint8_t)_reg_shadows[(uint8_t)MCP356xRegister::IRQ];
        _mcp356x_set_flag(MCP356X_FLAG_CRC_ERROR, (0 == (0x20 & irq_reg_data)));
        if (0 == (0x40 & irq_reg_data))
        { // Conversion is finished.
            ret = 1;
        }
        if (0 == (0x08 & irq_reg_data))
        { // Power-on-Reset has happened.
            // Not sure why this happened, but reset the class.
            //_post_reset_fxn();
            digitalWrite(_CS_PIN, 0);
            digitalWrite(_CS_PIN, 1);
            digitalWrite(_CS_PIN, 0);
            digitalWrite(_CS_PIN, 1);
        }
        if (0 == (0x20 & irq_reg_data))
        { // CRC config error.
            // Something is sideways in the configuration.
            // Send start/restart conversion. Might also write 0xA5 to the LOCK
            // register.
            _write_register(MCP356xRegister::LOCK, 0x000000A5);
            _send_fast_command(0x28);
        }
        if (0x01 & irq_reg_data)
        { // Conversion started
            // We don't configure the class this way, and don't observe the IRQ.
        }
    }
    isr_fired = !digitalRead(_IRQ_PIN);
    return ret;
}

/**
 * @brief Returns the number of channels supported by the MCP356x.
 *
 * Checks the device's configuration to determine the number of supported channels.
 * Should return 1, 2, 4, or 8 based on the MCP356x model used.
 *
 * @return uint8_t Number of supported channels (1, 2, 4, or 8). Returns 0 if an issue is detected.
 */
uint8_t MCP356x::_channel_count()
{
    switch ((uint16_t)_reg_shadows[(uint8_t)MCP356xRegister::RESERVED2])
    {
    case 0x000C:
        return 2; // MCP3561
    case 0x000D:
        return 4; // MCP3562
    case 0x000F:
        return 8; // MCP3564
    }
    return 0;
}

/**
 * @brief Converts the ADC value to a voltage for a specified channel.
 *
 * Calculates the voltage on a given channel by reversing the ADC transfer function.
 * Assumes the reference voltage is equal to AVdd unless otherwise set.
 *
 * @param chan The channel to calculate the voltage for.
 *
 * @return double The voltage value for the specified channel.
 */
double MCP356x::valueAsVoltage(MCP356xChannel chan)
{
    float vrp = _vref_plus;
    float vrm = _vref_minus;
    double result = 0.0;
    switch (chan)
    {
    case MCP356xChannel::SE_0: // Single-ended channels.
    case MCP356xChannel::SE_1:
    case MCP356xChannel::SE_2:
    case MCP356xChannel::SE_3:
    case MCP356xChannel::SE_4:
    case MCP356xChannel::SE_5:
    case MCP356xChannel::SE_6:
    case MCP356xChannel::SE_7:
    case MCP356xChannel::DIFF_A: // Differential channels.
    case MCP356xChannel::DIFF_B:
    case MCP356xChannel::DIFF_C:
    case MCP356xChannel::DIFF_D:
    case MCP356xChannel::OFFSET:
        result = (value(chan) * (vrp - vrm)) / (8388608.0 * _gain_value());
        break;
    case MCP356xChannel::TEMP:
        // TODO: voltage transfer fxn for temperature diode.
        result = value(chan);
        break;
    case MCP356xChannel::AVDD:
        result = value(chan) / (0.33 * 8388608.0); // Gain on this chan is always 0.33.
        break;
    case MCP356xChannel::VCM:
        result = (value(chan) * (vrp - vrm)) / 8388608.0;
        break;
    }
    return result;
}

/**
 * @brief Returns the last read value for a specified channel.
 *
 * Gets the stored value for the specified channel and clears its "new data" flag.
 *
 * @param chan The channel to get the value from.
 *
 * @return int32_t The last value read from the specified channel.
 */
int32_t MCP356x::value(MCP356xChannel chan)
{
    _channel_clear_new_flag(chan);
    return _channel_vals[(uint8_t)chan & 0x0F];
}

/**
 * @brief Sets the offset calibration for the ADC.
 *
 * Writes the offset calibration value to the ADC and enables the offset feature
 * if the provided value is non-zero.
 *
 * @param offset The offset calibration value.
 *
 * @return int8_t Status of the operation (0 on success, non-zero on error).
 */
int8_t MCP356x::setOffsetCalibration(int32_t offset)
{
    int8_t ret = _write_register(MCP356xRegister::OFFSETCAL, (uint32_t)offset);
    if (0 == ret)
    {
        uint32_t c_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG3];
        c_val = (0 != offset) ? (c_val | 0x00000002) : (c_val & 0xFFFFFFFD);
        ret = _write_register(MCP356xRegister::CONFIG3, c_val);
    }
    return ret;
}

/**
 * @brief Sets the gain calibration for the ADC.
 *
 * Writes the gain calibration multiplier to the ADC and enables the gain feature
 * if the provided value is non-zero.
 *
 * @param multiplier The gain calibration multiplier.
 *
 * @return int8_t Status of the operation (0 on success, non-zero on error).
 */
int8_t MCP356x::setGainCalibration(int32_t multiplier)
{
    _write_register(MCP356xRegister::GAINCAL, (uint32_t)multiplier);
    uint32_t c_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG3];
    c_val = (0 != multiplier) ? (c_val | 0x00000001) : (c_val & 0xFFFFFFFE);
    return _write_register(MCP356xRegister::CONFIG3, c_val);
}

/**
 * @brief Sets the gain setting of the MCP356x.
 *
 * Configures the ADC's gain setting to adjust the input signal amplitude.
 *
 * @param g The gain setting as defined in the MCP356xGain enumeration.
 *
 * @return int8_t Status of the operation (0 on success, non-zero on error).
 */
int8_t MCP356x::setGain(MCP356xGain g)
{
    uint32_t c_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG2];
    uint32_t gain_val = (c_val & 0xFFFFFFC7) | ((uint32_t)g << 3);
    return _write_register(MCP356xRegister::CONFIG2, gain_val);
}

/**
 * @brief Retrieves the current gain setting of the MCP356x.
 *
 * Returns the current gain setting from the CONFIG2 register.
 *
 * @return MCP356xGain The current gain setting.
 */
MCP356xGain MCP356x::getGain()
{
    return (MCP356xGain)((_reg_shadows[(uint8_t)MCP356xRegister::CONFIG2] >> 3) & 0x07);
}

/**
 * @brief Sets the bias current setting of the MCP356x.
 *
 * Adjusts the ADC's bias current, impacting power consumption and performance.
 *
 * @param d The bias current setting from the MCP356xBiasCurrent enumeration.
 *
 * @return int8_t Status of the operation (0 on success, non-zero on error).
 */
int8_t MCP356x::setBiasCurrent(MCP356xBiasCurrent d)
{
    uint32_t c0_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG0] & 0x00F3FFFF;
    c0_val += ((((uint8_t)d) & 0x03) << 18);
    return _write_register(MCP356xRegister::CONFIG0, c0_val);
}

/**
 * @brief Sets the ADC mode of the MCP356x.
 *
 * Configures the ADC mode which determines operational characteristics.
 *
 * @param mode The desired ADC mode from the MCP356xADCMode enumeration.
 *
 * @return int8_t Status of the operation (0 on success, non-zero on error).
 */
int8_t MCP356x::setADCMode(MCP356xADCMode mode)
{
    // Constants for better readability
    const uint32_t ADC_MODE_CLEAR_MASK = 0xFFFFFFFC; // Mask to clear the ADC mode bits
    const uint8_t ADC_MODE_BIT_MASK = 0x03;          // Mask to get the two least significant bits

    // Read the current CONFIG0 register value
    uint32_t config_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG0];

    // Clear the current ADC_MODE bits
    config_val &= ADC_MODE_CLEAR_MASK;

    // Set the desired ADC mode
    config_val |= ((uint8_t)mode & ADC_MODE_BIT_MASK);

    // Write the modified value back to the CONFIG0 register
    return _write_register(MCP356xRegister::CONFIG0, config_val);
}

/**
 * @brief Sets the conversion mode of the MCP356x.
 *
 * Configures how the ADC converts analog signals into digital values.
 *
 * @param mode The desired conversion mode from the MCP356xMode enumeration.
 *
 * @return int8_t Status of the operation (0 on success, non-zero on error).
 */
int8_t MCP356x::setConversionMode(MCP356xMode mode)
{
    // constants
    const uint8_t CONV_MODE_BIT_START = 6;
    const uint8_t CONV_MODE_MASK = 0x03;               // Mask for the CONV_MODE[1:0] bits
    const uint32_t CONFIG3_PRESERVE_MASK = 0xFFFFFFC0; // Mask to preserve all bits in CONFIG3 except for
    // CONV_MODE[1:0]

    // Preserve all bits in CONFIG3 except for CONV_MODE[1:0]
    uint32_t c3_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG3] & CONFIG3_PRESERVE_MASK;

    // Set the CONV_MODE[1:0] bits in CONFIG3 based on the desired conversion mode
    c3_val |= ((uint8_t)mode & CONV_MODE_MASK) << CONV_MODE_BIT_START;

    // Write the updated value to the CONFIG3 register
    return _write_register(MCP356xRegister::CONFIG3, c3_val);
}

/**
 * @brief Sets the scan timer value for the MCP356x.
 *
 * Configures the timer value that controls the auto-scan sequence.
 *
 * @param timerValue The timer value to set.
 *
 * @return int8_t Status of the register write operation.
 */
int8_t MCP356x::setScanTimer(uint32_t timerValue)
{
    return _write_register(MCP356xRegister::TIMER, timerValue);
}

/**
 * @brief Changes the AMCLK prescaler of the MCP356x.
 *
 * Adjusts the AMCLK prescaler, which affects the modulator clock frequency.
 * Ensures the configuration is valid and recalculates the clock tree if necessary.
 *
 * @param d The desired AMCLK prescaler setting.
 *
 * @return int8_t
 *         - -2 if MCLK frequency is unknown but prescaler setting succeeded.
 *         - -1 if reconfiguration of the prescaler failed.
 *         - 0 on successful update of the prescaler.
 */
int8_t MCP356x::setAMCLKPrescaler(MCP356xAMCLKPrescaler d)
{
    uint32_t c1_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x00FFFF3F;
    c1_val |= ((((uint8_t)d) & 0x03) << 6);
    int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);
    if (0 == ret)
    {
        ret = _recalculate_clk_tree();
    }
    return ret;
}

/**
 * @brief Sets the oversampling ratio for the MCP356x.
 *
 * Configures the oversampling ratio of the ADC, affecting resolution and noise performance.
 * Recalculates the settling time based on the new ratio.
 *
 * @param d The desired oversampling ratio.
 *
 * @return int8_t
 *         - -1 if reconfiguration failed.
 *         - 0 on successful update of the oversampling ratio.
 */
int8_t MCP356x::setOversamplingRatio(MCP356xOversamplingRatio d)
{
    uint32_t c1_val = _reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x00FFFFC3;
    c1_val |= ((((uint8_t)d) & 0x0F) << 2);
    int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);
    if (0 == ret)
    {
        ret = _recalculate_clk_tree();
    }
    return ret;
}

/**
 * @brief Retrieves the current oversampling ratio of the MCP356x.
 *
 * Returns the current oversampling ratio setting from the CONFIG1 register.
 *
 * @return MCP356xOversamplingRatio The current oversampling ratio.
 */
MCP356xOversamplingRatio MCP356x::getOversamplingRatio()
{
    return (MCP356xOversamplingRatio)((_reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x0000003C) >> 2);
}

/**
 * @brief Checks if the MCP356x is using the internally generated Vref.
 *
 * Returns whether the ADC is configured to use its internal voltage reference.
 *
 * @return bool True if using internal Vref, false otherwise.
 */
bool MCP356x::usingInternalVref()
{
    bool ret = false;
    if (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF))
    {
        ret = (0 != (_reg_shadows[(uint8_t)MCP356xRegister::CONFIG0] & 0x00000040));
    }
    return ret;
}

/**
 * @brief Configures the MCP356x to use its internal Vref.
 *
 * Enables or disables the use of the internal voltage reference for the ADC.
 * Updates the configuration and adjusts the reference voltage values accordingly.
 *
 * @param x Boolean flag to enable (true) or disable (false) the internal Vref.
 *
 * @return int8_t
 *         - 0 on successful configuration.
 *         - -1 if the feature is not supported.
 *         - -2 on I/O failure.
 */
int8_t MCP356x::useInternalVref(bool x)
{
    int8_t ret = -1;
    if (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF))
    {
        uint32_t c0_val =
            _reg_shadows[(uint8_t)MCP356xRegister::CONFIG0] & 0x00FFFFBF;
        if (x)
        {
            c0_val |= 0x00000040;
            _vref_plus = 2.4;
            _vref_minus = 0;
        }
        ret = (0 == _write_register(MCP356xRegister::CONFIG0, c0_val)) ? 0 : -2;
    }
    return ret;
}

/*******************************************************************************
 * Internal functions
 *******************************************************************************/

/**
 * @brief Initializes the low-level pin configuration.
 *
 * Sets up the necessary pins for the MCP356x operation, including CS, IRQ, and MCLK.
 * The execution is idempotent, meaning it can be called multiple times without adverse effects.
 *
 * @return int8_t
 *         - -1 if the pin setup is incorrect, requiring the class to halt.
 *         - 0 if the pin setup is already complete.
 *         - 1 if the pin setup is complete and the clock needs measurement.
 */
int8_t MCP356x::_ll_pin_init()
{
    int8_t ret = -1;
    if (_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))
    {
        ret = 0;
    }
    else if (255 != _CS_PIN)
    {
        ret = 1;
        pinMode(_CS_PIN, OUTPUT);
        digitalWrite(_CS_PIN, 1);
        if (255 != _IRQ_PIN)
        {
            pinMode(_IRQ_PIN, INPUT_PULLUP);
            switch (_slot_number)
            {
            case 0:
                attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr0, FALLING);
                break;
            case 1:
                attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr1, FALLING);
                break;
            case 2:
                attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), mcp356x_isr2, FALLING);
                break;
            default:

                break;
            }
        }
        if (255 != _MCLK_PIN)
        {
            if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK))
            {
                pinMode(_MCLK_PIN, INPUT); // when using the internal clock, the clock frequency is fixed and known (4.9152 MHz in this case).
                _mclk_freq = 4915200.0;    // Set the internal clock frequency
                _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
                ret = _recalculate_clk_tree();
            }
            else
            {
                pinMode(_MCLK_PIN, OUTPUT);
                if (_mcp356x_flag(MCP356X_FLAG_GENERATE_MCLK))
                {
                    analogWriteFrequency(_MCLK_PIN, 4915200);
                    analogWrite(_MCLK_PIN, 128);
                    _mclk_freq = 4915200.0;
                }
                else
                {
                    digitalWrite(_MCLK_PIN, 1);
                }
                _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
                ret = _recalculate_clk_tree();
            }
        }
        _mcp356x_set_flag(MCP356X_FLAG_PINS_CONFIGURED);
    }
    return ret;
}

/**
 * @brief Resets the internal register shadows to their default values.
 *
 * Resets the internal state of the MCP356x, including register shadows,
 * second-order data, and various internal counters and flags.
 *
 * @return int8_t Always returns 0, indicating the reset was successful.
 */
int8_t MCP356x::_clear_registers()
{
    uint32_t flg_mask = MCP356X_FLAG_RESET_MASK;
    if (!(_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)))
    {
        // The only way the clock isn't running is if it is running internally.
        flg_mask |= MCP356X_FLAG_MCLK_RUNNING;
    }
    _flags = _flags & flg_mask; // Reset the flags.
    for (uint8_t i = 0; i < 16; i++)
    {
        // We decline to revert RESERVED2, since we need it for device identity.
        // RESERVED2 (0x000F for 3564, 0xD for 3562, 0xC for 3561)
        _reg_shadows[i] = (14 != i) ? 0 : _reg_shadows[i];
        _channel_vals[i] = 0;
    }
    _channel_flags = 0;
    _discard_until_micros = 0;
    _settling_us = 0;
    _read_count = 0;
    _read_accumulator = 0;
    _reads_per_second = 0;
    _micros_last_read = 0;
    _micros_last_window = 0;
    return 0;
}

/**
 * @brief Writes a value to a specified register in the MCP356x.
 *
 * Performs safety checks on the value/register combination before writing
 * to the register. Filters out unimplemented bits and handles special cases.
 *
 * @param r The register to write to.
 * @param val The value to write to the register.
 *
 * @return int8_t
 *         - -3 if the register is not writable.
 *         - -2 if the register size is unexpected.
 *         - -1 on a general error.
 *         - 0 on success.
 */
int8_t MCP356x::_write_register(MCP356xRegister r, uint32_t val)
{
    uint32_t safe_val = 0;
    int8_t ret = -1;
    uint8_t register_size = MCP356x_reg_width[(uint8_t)r];
    switch (r)
    {
        // Filter out the unimplemented bits.
    case MCP356xRegister::CONFIG1:
        safe_val = val & 0xFFFFFFFC;
        break;
    case MCP356xRegister::CONFIG2:
        safe_val =
            val | (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000001 : 0x00000003);
        break;
    case MCP356xRegister::SCAN:
        safe_val = val & 0xFFE0FFFF;
        break;
    case MCP356xRegister::RESERVED0:
        safe_val = 0x00900000;
        break;
    case MCP356xRegister::RESERVED1:
        safe_val = (_mcp356x_flag(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000030 : 0x00000050);
        break;
    case MCP356xRegister::RESERVED2:
        safe_val = val & 0x0000000F;
        break;
        // No safety required.
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
        // Not writable.
    case MCP356xRegister::ADCDATA:
    case MCP356xRegister::CRCCFG:
        return -3;
    }

    _bus->beginTransaction(spi_settings);
    digitalWrite(_CS_PIN, LOW);
    _bus->transfer(_get_reg_addr(r));
    switch (register_size)
    {
    case 3:
        _bus->transfer((uint8_t)(safe_val >> 16) & 0xFF); // MSB-first
    case 2:
        _bus->transfer((uint8_t)(safe_val >> 8) & 0xFF);
    case 1:
        _bus->transfer((uint8_t)safe_val & 0xFF);
        ret = 0;
        _reg_shadows[(uint8_t)r] = safe_val;
        break;
    default:
        ret = -2; // Error on unexpected width.
    }
    digitalWrite(_CS_PIN, HIGH);
    _bus->endTransaction();
    return ret;
}

/**
 * @brief Reads the value from a specified register in the MCP356x.
 *
 * Reads the current value of the specified register, handling different
 * register sizes and special cases for the ADC data register.
 *
 * @param r The register to read from.
 *
 * @return int8_t Always returns 0, indicating the read was successful.
 */
int8_t MCP356x::_read_register(MCP356xRegister r)
{
    uint8_t bytes_to_read = MCP356x_reg_width[(uint8_t)r];
    if (MCP356xRegister::ADCDATA == r)
    {
        bytes_to_read = _output_coding_bytes();
    }

    _bus->beginTransaction(spi_settings);
    digitalWrite(_CS_PIN, LOW);
    _bus->transfer((uint8_t)_get_reg_addr(r) | 0x01);
    uint32_t temp_val = 0;
    for (int i = 0; i < bytes_to_read; i++)
    {
        temp_val = (temp_val << 8) + _bus->transfer(0);
    }
    digitalWrite(_CS_PIN, HIGH);
    _bus->endTransaction();
    _reg_shadows[(uint8_t)r] = temp_val;
    return 0;
}

/**
 * @brief Determines the number of bytes to read from the ADC data register.
 *
 * Returns the number of bytes that should be read from the ADC data register,
 * based on the current configuration settings.
 *
 * @return uint8_t The number of bytes to read from the ADC data register.
 */
uint8_t MCP356x::_output_coding_bytes()
{
    return (0 == _reg_shadows[(uint8_t)MCP356xRegister::CONFIG3]) ? 3 : 4;
}

/**
 * @brief Normalizes and updates the data from the ADC data register.
 *
 * Processes the raw data from the ADC data register, performing sign extension
 * and updating internal values. It handles different types of channels and
 * updates flags based on the data read.
 *
 * @return int8_t Always returns 0, indicating the normalization was successful.
 */
int8_t MCP356x::_normalize_data_register()
{
    uint32_t rval = _reg_shadows[(uint8_t)MCP356xRegister::ADCDATA];
    MCP356xChannel chan = (MCP356xChannel)((rval >> 28) & 0x0F);

    // Sign extend, if needed.
    int32_t nval =
        (int32_t)(rval & 0x01000000) ? (rval | 0xFE000000) : (rval & 0x01FFFFFF);

    // Update the over-range marker...
    _channel_vals[(uint8_t)chan] = nval; // Store the decoded ADC reading.
    _channel_set_ovr_flag(chan, ((nval > 8388609) | (nval < -8388609)));
    _channel_set_new_flag(chan); // Mark the channel as updated.

    switch (chan)
    {
        // Different channels are interpreted differently...
    case MCP356xChannel::SE_0: // Single-ended channels.
    case MCP356xChannel::SE_1:
    case MCP356xChannel::SE_2:
    case MCP356xChannel::SE_3:
    case MCP356xChannel::SE_4:
    case MCP356xChannel::SE_5:
    case MCP356xChannel::SE_6:
    case MCP356xChannel::SE_7:
    case MCP356xChannel::DIFF_A: // Differential channels.
    case MCP356xChannel::DIFF_B:
    case MCP356xChannel::DIFF_C:
    case MCP356xChannel::DIFF_D:
        break;
    case MCP356xChannel::TEMP:
        break;
    case MCP356xChannel::AVDD:
        _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_AVDD);
        if (!_mcp356x_flag(MCP356X_FLAG_VREF_DECLARED))
        {
            // If we are scanning the AVDD channel, we use that instead of the
            //   assumed 3.3v.
            //_vref_plus = nval / (8388608.0 * 0.33);
        }
        if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK))
        {
            _mark_calibrated();
        }
        break;
    case MCP356xChannel::VCM:
        // Nothing done here yet. Value should always be near 1.2v.
        _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_VCM);
        if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK))
        {
            _mark_calibrated();
        }
        break;
    case MCP356xChannel::OFFSET:
        if (0 == setOffsetCalibration(nval))
        {
            _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_OFFSET);
        }
        if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK))
        {
            _mark_calibrated();
        }
        break;
    }
    return 0;
}

/**
 * @brief Returns the current gain value of the ADC.
 *
 * Retrieves the gain setting from the ADC and returns its corresponding
 * float value.
 *
 * @return float The gain value as a float, or zero if the enum is out-of-bounds.
 */
float MCP356x::_gain_value()
{
    switch (getGain())
    {
    case MCP356xGain::GAIN_ONETHIRD:
        return 0.33;
    case MCP356xGain::GAIN_1:
        return 1.0;
    case MCP356xGain::GAIN_2:
        return 2.0;
    case MCP356xGain::GAIN_4:
        return 4.0;
    case MCP356xGain::GAIN_8:
        return 8.0;
    case MCP356xGain::GAIN_16:
        return 16.0;
    case MCP356xGain::GAIN_32:
        return 32.0;
    case MCP356xGain::GAIN_64:
        return 64.0;
    }
    return 0.0;
}

/**
 * @brief Generates a control byte for SPI transactions based on the given register.
 *
 * Applies the device address and shifts the register address into a control byte.
 * It sets up for incremental read/write operations and always returns a valid byte.
 *
 * @param r The register to generate the control byte for.
 *
 * @return uint8_t The control byte for the specified register.
 */
uint8_t MCP356x::_get_reg_addr(MCP356xRegister r)
{
    return (((_DEV_ADDR & 0x03) << 6) | (((uint8_t)r) << 2) | 0x02);
}

/**
 * @brief Refreshes the internal register shadows with the state of the hardware registers.
 *
 * Performs a full refresh of the register shadows, updating them to match the
 * current state of the MCP356x's hardware registers.
 *
 * @return int8_t
 *         - -2 if reads were successful but register values don't match defaults.
 *         - -1 if a register read failed.
 *         - 0 if an MCP356x was successfully identified.
 */
int8_t MCP356x::refresh()
{
    uint8_t i = 0;
    int8_t ret = 0;
    while ((0 == ret) & (i < 16))
    {
        ret = _read_register((MCP356xRegister)i++);
    }
    if (0 == ret)
    {
        ret = -2;
        if (0x00900000 == _reg_shadows[(uint8_t)MCP356xRegister::RESERVED0])
        {
            uint8_t res1_val =
                (uint8_t)_reg_shadows[(uint8_t)MCP356xRegister::RESERVED1];
            switch (res1_val)
            {
            case 0x30:
            case 0x50:
                // If the chip has an internal Vref, it will start up running and
                //   connected.
                _mcp356x_set_flag(MCP356X_FLAG_HAS_INTRNL_VREF, (res1_val == 0x30));
                switch (_reg_shadows[(uint8_t)MCP356xRegister::RESERVED2])
                {
                case 0x0C:
                case 0x0D:
                case 0x0F:
                    _mcp356x_set_flag(MCP356X_FLAG_DEVICE_PRESENT);
                    ret = 0;
                    break;
                default:
                    //
                    break;
                }
                break;
            default:
                //
                break;
            }
        }
        // else
    }
    return ret;
}

/**
 * @brief Discards samples that are not fully settled.
 *
 * When the analog value is changing, this function can be invoked to ensure
 * only fully-settled values are reported by the ADC.
 */
void MCP356x::discardUnsettledSamples()
{
    _discard_until_micros = getSettlingTime() + _circuit_settle_us + micros();
}

/**
 * @brief Sends a non-register access fast command to the MCP356x.
 *
 * Used for sending fast commands that don't involve direct register access.
 *
 * @param cmd The fast command to send.
 *
 * @return int8_t
 *         - -1 on failure to write SPI.
 *         - 0 on success.
 */
int8_t MCP356x::_send_fast_command(uint8_t cmd)
{
    int8_t ret = 0;
    _bus->beginTransaction(spi_settings);
    digitalWrite(_CS_PIN, LOW);
    _bus->transfer((uint8_t)((_DEV_ADDR & 0x03) << 6) | cmd);
    digitalWrite(_CS_PIN, HIGH);
    _bus->endTransaction();
    return ret;
}

/**
 * @brief Sets the list of channels to be read in SCAN mode.
 *
 * Configures which channels are to be read by the MCP356x in SCAN mode.
 * Checks for channel support and handles errors accordingly.
 *
 * @param count The number of channels to set.
 * @param ... Variable arguments representing the channels to set.
 *
 * @return int8_t
 *         - -4 if an unsupported channel was requested.
 *         - -3 if a requested channel is not supported by the hardware.
 *         - -2 if no channels were provided.
 *         - -1 on failure to write the SCAN register.
 *         - 0 on success.
 */
int8_t MCP356x::setScanChannels(int count, ...)
{
    int8_t ret = (count > 0) ? 0 : -2;
    uint8_t chan_count = _channel_count();
    uint32_t existing_scan = _reg_shadows[(uint8_t)MCP356xRegister::SCAN];
    uint32_t chans = 0;
    va_list args;
    va_start(args, count);
    for (int i = 0; i < count; i++)
    {
        MCP356xChannel chan = va_arg(args, MCP356xChannel);
        switch (chan)
        {
        case MCP356xChannel::SE_0:
        case MCP356xChannel::SE_1:
        case MCP356xChannel::DIFF_A:
        case MCP356xChannel::TEMP:
        case MCP356xChannel::AVDD:
        case MCP356xChannel::VCM:
        case MCP356xChannel::OFFSET:
            if (2 > chan_count)
            {
                ret = -3;
            }
            break;
        case MCP356xChannel::SE_2:
        case MCP356xChannel::SE_3:
        case MCP356xChannel::DIFF_B:
            if (4 > chan_count)
            {
                ret = -3;
            }
            break;
        case MCP356xChannel::SE_4:
        case MCP356xChannel::SE_5:
        case MCP356xChannel::SE_6:
        case MCP356xChannel::SE_7:
        case MCP356xChannel::DIFF_C:
        case MCP356xChannel::DIFF_D:
            if (8 != chan_count)
            {
                ret = -3;
            }
            break;
        default:
            ret = -4;
            break;
        }
        if (0 == ret)
        {
            chans = chans | (1 << ((uint8_t)chan));
        }
    }
    va_end(args);
    if (0 == ret)
    { // If there were no foul ups, we can write the registers.
        chans = chans | (existing_scan & 0xFFFF0000);
        _channel_backup = chans;
        if (_mcp356x_flag(MCP356X_FLAG_CALIBRATED))
        {
            ret = _set_scan_channels(chans);
        }
        else
        {
            _channel_backup = chans;
            ret = 0;
        }
    }
    return ret;
}

/**
 * @brief Helper function to set the scan channels in the SCAN register.
 *
 * @param rval The value to write to the SCAN register.
 *
 * @return int8_t Result of the register write operation.
 */
int8_t MCP356x::_set_scan_channels(uint32_t rval)
{
    if (!_mcp356x_flag(MCP356X_FLAG_CALIBRATED))
    {
        _channel_backup = _reg_shadows[(uint8_t)MCP356xRegister::SCAN];
    }
    return _write_register(MCP356xRegister::SCAN, (uint32_t)rval);
}

/**
 * @brief Checks if the scan across the requested channels is complete.
 *
 * Determines if data for all the channels requested in SCAN mode have been read.
 *
 * @return bool True if scanning is complete for all requested channels, false otherwise.
 */
bool MCP356x::scanComplete()
{
    uint32_t scan_chans = _reg_shadows[(uint8_t)MCP356xRegister::SCAN] & 0x0000FFFF;
    return (scan_chans == (_channel_flags & scan_chans));
};

/**
 * @brief Sets the reference voltage range for the MCP356x.
 *
 * Allows manual definition of the reference voltage range, useful for hardware
 * arrangements that do not use a rail-to-rail Vref.
 *
 * @param plus The positive reference voltage.
 * @param minus The negative reference voltage.
 *
 * @return int8_t Always returns 0, indicating the reference range was set successfully.
 */
int8_t MCP356x::setReferenceRange(float plus, float minus)
{
    _vref_plus = plus;
    _vref_minus = minus;
    _mcp356x_set_flag(MCP356X_FLAG_VREF_DECLARED);
    return 0;
}

/**
 * @brief Calculates the temperature from the ADC's temperature channel.
 *
 * Reads the current temperature value from the ADC and converts it to
 * the die temperature using a high-accuracy third-order fit or linear approximation.
 *
 * @return float The calculated temperature in degrees Celsius.
 */
float MCP356x::getTemperature()
{
    int32_t t_lsb = value(MCP356xChannel::TEMP);
    float ret = 0.0;
    if (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP))
    {
        const double k1 = 0.0000000000000271 * (t_lsb * t_lsb * t_lsb);
        const double k2 = -0.000000018 * (t_lsb * t_lsb);
        const double k3 = 0.0055 * t_lsb;
        const double k4 = -604.22;
        ret = k1 + k2 + k3 + k4;
    }
    else
    {
        ret = 0.001581 * t_lsb - 324.27;
    }
    return ret;
}

/*******************************************************************************
 * Hardware discovery functions
 *******************************************************************************/

/**
 * @brief Checks if the MCLK frequency is within operational boundaries.
 *
 * Verifies if the measured MCLK frequency is within the operational range
 * of 1MHz to 20MHz.
 *
 * @return bool True if MCLK is within bounds, false otherwise.
 */
bool MCP356x::_mclk_in_bounds()
{
    bool inBounds = (_mclk_freq > 1000000.0) && (_mclk_freq < 20000000.0);
    return inBounds;
}

/**
 * @brief Detects the ADC input clock frequency.
 *
 * Measures the ADC reads timing with known clocking parameters to discover
 * the MCLK frequency. Stores the result in _mclk_freq and recalculates the clock tree.
 *
 * @return int8_t
 *         - -3 if the class isn't ready for measurement.
 *         - -2 if measurement timed out.
 *         - -1 on mechanical problems communicating with the ADC.
 *         - 0 if a clock signal within expected range was determined.
 *         - 1 if the clock rate is out of bounds or nonsensical.
 */
int8_t MCP356x::_detect_adc_clock()
{
    const uint32_t SAMPLE_TIME_MAX = 2000000; // 200ms in microseconds
    int8_t ret = -3;

    if (_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))
    {
        ret = -1;
        if (0 == _write_register(MCP356xRegister::SCAN, 0))
        {
            if (0 == _write_register(MCP356xRegister::MUX, 0xDE))
            {
                unsigned long micros_passed = 0;
                unsigned long micros_adc_time_0 = micros();
                uint16_t rcount = 0;
                while ((1000 > rcount) && (micros_passed < SAMPLE_TIME_MAX))
                {
                    if (isr_fired)
                    {
                        if (0 < read())
                        {
                            if (0 == rcount)
                            {
                                resetReadCount();
                            }
                            rcount++;
                        }
                    }
                    micros_passed = micros() - micros_adc_time_0;
                }
                ret = -2;
                if (micros_passed < SAMPLE_TIME_MAX)
                {
                    ret = 1;
                    _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
                    //           StringBuilder temp_str;
                    // temp_str.concatf("Took %u samples in %luus.\n", rcount, micros_passed);
                    // Serial.print((char*) temp_str.string());
                    _mclk_freq = _calculate_input_clock(micros_passed);
                    if (_mclk_in_bounds())
                    {
                        _recalculate_clk_tree();
                        ret = 0;
                    }
                }
            }
        }
    }
    return ret;
}

/**
 * @brief Calculates the true rate of the input clock.
 *
 * Calculates the MCLK frequency based on elapsed time and ADC sampling parameters.
 * Provides accurate results if the settings are unchanged from init or after resetReadCount().
 *
 * @param elapsed_us Elapsed time in microseconds used for the calculation.
 *
 * @return double The calculated MCLK frequency.
 */
double MCP356x::_calculate_input_clock(unsigned long elapsed_us)
{
    uint32_t osr_idx = (_reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x0000003C) >> 2;
    uint16_t osr1 = OSR1_VALUES[osr_idx];
    uint16_t osr3 = OSR3_VALUES[osr_idx];
    uint32_t pre_val = (_reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x000000C0) >> 6;

    double _drclk = ((double)_read_count) / ((double)elapsed_us) * 1000000.0;

    return (4 * (osr3 * osr1) * (1 << pre_val) * _drclk);
}

/**
 * @brief Recalculates the clock tree based on the MCLK frequency.
 *
 * Recalculates the DRCLK based on the MCLK frequency and updates internal parameters.
 *
 * @return int8_t
 *         - -2 if MCLK frequency is out-of-bounds.
 *         - 0 if the calculation completed successfully.
 */
int8_t MCP356x::_recalculate_clk_tree()
{
    if (_mclk_in_bounds())
    {
        uint32_t pre_val = (_reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x000000C0) >> 6;
        _dmclk_freq = _mclk_freq / (4 * (1 << pre_val));
        return _recalculate_settling_time();
    }

    return -2;
}

/**
 * @brief Calculates the ADC conversion time based on the oversampling ratio settings.
 *
 * Retrieves the OSR settings from CONFIG1 register and calculates the conversion time.
 * The calculated conversion time is stored in the `_settling_us` member variable.
 *
 * @return int8_t Always returns 0, indicating successful calculation.
 */
int8_t MCP356x::_recalculate_settling_time()
{
    // Extract the OSR index from the CONFIG1
    uint32_t osr_idx = (_reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x0000003C) >> 2;

    // Retrieve the corresponding OSR values.
    uint16_t osr1 = OSR1_VALUES[osr_idx];
    uint16_t osr3 = OSR3_VALUES[osr_idx];

    // Calculate dmclks.
    uint32_t dmclks = (3 * osr3) + ((osr1 - 1) * osr3);

    // Calculate settling time in microseconds.
    _settling_us = (1000000.0 * dmclks) / _dmclk_freq;

    _drclk_freq = _dmclk_freq / (osr1 * osr3);

    // StringBuilder output;
    // output.concatf("-----------------------\n");
    // output.concatf("OSR Index: %lu\n", osr_idx);
    // output.concatf("OSR1: %u, OSR3: %u\n", osr1, osr3);
    // output.concatf("dmclks: %lu\n", dmclks);
    // output.concatf("_dmclk_freq (Hz): %f\n", _dmclk_freq);
    // output.concatf("Settling time (us): %lu\n", _settling_us);
    // output.concatf("-----------------------\n");
    // this->printRegs(&output);
    // Serial.print((char*)output.string());

    return 0;
}

/**
 * @brief Initiates the offset calibration process.
 *
 * Sets up the ADC for offset calibration by configuring the scan channels and
 * clearing the calibration flag.
 *
 * @return int8_t
 *         - -1 if a register I/O operation failed.
 *         - 0 if the offset calibration setup was successful.
 */
int8_t MCP356x::_calibrate_offset()
{
    _channel_backup = _reg_shadows[(uint8_t)MCP356xRegister::SCAN];
    int8_t ret = _set_scan_channels(0x0000E000);
    if (0 == ret)
    {
        _mcp356x_clear_flag(MCP356X_FLAG_CALIBRATED);
    }
    return ret;
}

/**
 * @brief Marks the ADC as calibrated.
 *
 * Restores the scan channel settings and sets the calibrated flag upon successful calibration.
 *
 * @return int8_t Status of restoring the scan channel settings.
 */
int8_t MCP356x::_mark_calibrated()
{
    int8_t ret = _set_scan_channels(_channel_backup);
    if (0 == ret)
    {
        _mcp356x_set_flag(MCP356X_FLAG_CALIBRATED);
    }
    return ret;
}

/**
 * @brief Prints the current register values of the MCP356x.
 *
 * Outputs the values of all the registers in binary format to the provided StringBuilder.
 *
 * @param output Pointer to the StringBuilder to append the register values to.
 */
void MCP356x::printRegs(StringBuilder *output)
{
    // Iterate over each register value and print its binary representation
    const char *regNames[] = {"ADCDATA", "CONFIG0", "CONFIG1", "CONFIG2", "CONFIG3",
                              "IRQ", "MUX", "SCAN", "TIMER", "OFFSETCAL",
                              "GAINCAL", "RESERVED0", "RESERVED1", "LOCK",
                              "RESERVED2", "CRCCFG"};
    const int bitsPerReg[] = {32, 8, 8, 8, 8, 8, 8, 24, 24, 24, 24, 24, 8, 8, 16, 16};

    for (unsigned int i = 0; i < sizeof(regNames) / sizeof(regNames[0]); ++i)
    {
        output->concatf("_reg_shadows[%d] (%s) = ", i, regNames[i]);
        regToBinaryString(_reg_shadows[i], output, bitsPerReg[i]);
        output->concat("\n");
    }
}

/**
 * @brief Converts a register value to a binary string representation.
 *
 * @param regValue The register value to convert.
 * @param output Pointer to the StringBuilder to append the binary string to.
 * @param bits Number of bits to include in the binary representation.
 */
void MCP356x::regToBinaryString(uint32_t regValue, StringBuilder *output, int bits)
{
    for (int i = bits - 1; i >= 0; --i)
    {
        output->concat((regValue & (1 << i)) ? "1" : "0");
        if (i % 4 == 0 && i != 0)
        { // Optional: Add a space every 4 bits for readability
            output->concat(" ");
        }
    }
}

/**
 * @brief Prints the pin configuration of the MCP356x.
 *
 * Outputs the current IRQ, CS, and MCLK pin assignments to the provided StringBuilder.
 *
 * @param output Pointer to the StringBuilder to append the pin information to.
 */
void MCP356x::printPins(StringBuilder *output)
{
    output->concatf("IRQ:   %u\n", _IRQ_PIN);
    output->concatf("CS:    %u\n", _CS_PIN);
    output->concatf("MCLK:  %u\n", _MCLK_PIN);
}

/**
 * @brief Prints timing information of the MCP356x.
 *
 * Outputs various timing parameters such as MCLK, DMCLK, data rate, sample rate, and settling times.
 *
 * @param output Pointer to the StringBuilder to append the timing information to.
 */
void MCP356x::printTimings(StringBuilder *output)
{

    uint32_t osr_idx = (_reg_shadows[(uint8_t)MCP356xRegister::CONFIG1] & 0x0000003C) >> 2;

    // Retrieve the corresponding OSR values.
    uint16_t osr1 = OSR1_VALUES[osr_idx];
    uint16_t osr3 = OSR3_VALUES[osr_idx];

    output->concatf("-----------------------\n");
    output->concatf("\tOSR1: %u, OSR3: %u\n", osr1, osr3);

    output->concatf("\tMCLK                = %.4f MHz\n", _mclk_freq / 1000000.0);
    output->concatf("\tDMCLK               = %.4f MHz\n", _dmclk_freq / 1000000.0);
    output->concatf("\tData rate           = %.4f KHz\n", _drclk_freq / 1000.0);
    output->concatf("\tReal sample rate    = %u samples/s\n", _reads_per_second);

    // Calculate the average time per reading in microseconds.
    float averageTimePerRead = 0;
    if (_reads_per_second > 0)
    {
        averageTimePerRead = 1000000.0f / _reads_per_second;
    }
    output->concatf("\tAverage time per read = %.2f us\n", averageTimePerRead);
    output->concatf("\tADC settling time   = %u us\n", getSettlingTime());
    output->concatf("\tTotal settling time (micros) = %u\n", _circuit_settle_us);
    output->concatf("\tLast read (micros)  = %u\n", _micros_last_read);
}

/**
 * @brief Prints detailed data and status information of the MCP356x.
 *
 * Outputs a comprehensive set of information including device model, channel count, clock status,
 * calibration status, CRC error status, and more.
 *
 * @param output Pointer to the StringBuilder to append the data information to.
 */
void MCP356x::printData(StringBuilder *output)
{
    StringBuilder prod_str("MCP356");
    if (adcFound())
    {
        prod_str.concatf("%d", _channel_count() >> 1);
        if (hasInternalVref())
            prod_str.concat('R');
    }
    else
        prod_str.concat("x (not found)");

    StringBuilder::styleHeader2(output, (const char *)prod_str.string());
    if (adcFound())
    {
        output->concatf("\tChannels:       %u\n", _channel_count());
        output->concatf("\tClock running:  %c\n", (_mcp356x_flag(MCP356X_FLAG_MCLK_RUNNING) ? 'y' : 'n'));
        output->concatf("\tConfigured:     %c\n", (adcConfigured() ? 'y' : 'n'));
        output->concatf("\tCalibrated:     %c\n", (adcCalibrated() ? 'y' : 'n'));
        if (adcCalibrated())
        {
            output->concat("\t");
            printChannel(MCP356xChannel::OFFSET, output);
            output->concat("\t");
            printChannel(MCP356xChannel::VCM, output);
            output->concat("\t");
            printChannel(MCP356xChannel::AVDD, output);
        }
        else
        {
            output->concatf("\t  SAMPLED_OFFSET: %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_OFFSET) ? 'y' : 'n'));
            output->concatf("\t  SAMPLED_VCM:    %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_VCM) ? 'y' : 'n'));
            output->concatf("\t  SAMPLED_AVDD:   %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_AVDD) ? 'y' : 'n'));
        }
        output->concatf("\tCRC Error:      %c\n",
                        (_mcp356x_flag(MCP356X_FLAG_CRC_ERROR) ? 'y' : 'n'));
        output->concatf("\tisr_fired:      %c\n", (isr_fired ? 'y' : 'n'));
        output->concatf("\tRead count:     %u\n", _read_count);
        output->concatf("\tGain:           x%.2f\n", _gain_value());
        uint8_t _osr_idx = (uint8_t)getOversamplingRatio();
        output->concatf("\tOversampling:   x%u\n", OSR1_VALUES[_osr_idx] * OSR3_VALUES[_osr_idx]);
        output->concatf("\tVref source:    %sternal\n", (usingInternalVref() ? "In" : "Ex"));
        output->concatf("\tVref declared:  %c\n", (_vref_declared() ? 'y' : 'n'));
        output->concatf("\tVref range:     %.3f / %.3f\n", _vref_minus, _vref_plus);
        output->concatf("\tClock SRC:      %sternal\n", (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK) ? "In" : "Ex"));
        if (_scan_covers_channel(MCP356xChannel::TEMP))
        {
            output->concatf("\tTemperature:    %.2fC\n", getTemperature());
            output->concatf("\tThermo fitting: %s\n", (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP) ? "3rd-order" : "Linear"));
        }
    }
}

/**
 * @brief Prints information about a specific ADC channel.
 *
 * Outputs the voltage value and over-range status for a specified ADC channel.
 *
 * @param chan The channel to print information about.
 * @param output Pointer to the StringBuilder to append the channel information to.
 */
void MCP356x::printChannel(MCP356xChannel chan, StringBuilder *output)
{
    output->concatf("%s:\t%.6fv\t%s\n", CHAN_NAMES[((uint8_t)chan) & 0x0F], valueAsVoltage(chan), _channel_over_range(chan) ? "OvR" : " ");
}

/**
 * @brief Prints the values of all enabled channels.
 *
 * Iterates over all channels and outputs their values. Can display values
 * either as raw ADC readings or in voltage format.
 *
 * @param output Pointer to the StringBuilder to append channel values to.
 * @param asVoltage Boolean flag to select display format (true for voltage).
 */
void MCP356x::printChannelValues(StringBuilder *output, bool asVoltage)
{
    // Iterate over all possible channel enum values
    for (uint8_t i = static_cast<uint8_t>(MCP356xChannel::SE_0);
         i <= static_cast<uint8_t>(MCP356xChannel::VCM); i++)
    {
        MCP356xChannel chan = static_cast<MCP356xChannel>(i);

        // Check if the current channel is enabled for scanning
        if (_scan_covers_channel(chan))
        {
            switch (chan)
            {
            case MCP356xChannel::TEMP:
                // Special handling for temperature channel
                output->concatf("Die temperature     = %.2fC\n", getTemperature());
                break;
            default:
                // For all other channels, including VCM
                if (asVoltage)
                {
                    // Display values in voltage format
                    output->concatf("%s:\t%.6fv\t%s\n", CHAN_NAMES[i & 0x0F], valueAsVoltage(chan), _channel_over_range(chan) ? "OvR" : " ");
                }
                else
                {
                    // Display raw ADC values
                    output->concatf("%s:\t%d\t%s\n", CHAN_NAMES[i & 0x0F], value(chan), _channel_over_range(chan) ? "OvR" : " ");
                }
                break;
            }
        }
    }
}