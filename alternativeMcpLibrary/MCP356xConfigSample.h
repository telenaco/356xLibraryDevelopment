/**
 * @brief Configuration class for the MCP356x ADC. 
 * 
 * This class holds the configuration settings for an MCP356x ADC instance.
 * It allows the settings to be configured prior to ADC initialization.
 */
class MCP356xConfig {

public:

  /**
   * @brief Bitmask indicating channels to be scanned. 
   *
   * 1 bit per channel. Set bits enable scanning for that channel.
   */
  uint32_t scan;

  /** 
   * @brief Option flags.
   * 
   * Bitmask of options like using the internal clock.
   * See MCP356X_FLAG_* defines for available options.
   */
  uint32_t flags;

  /**
   * @brief ADC operating mode.
   *
   * Options:
   * - ONESHOT_SHUTDOWN
   * - ONESHOT_STANDBY 
   * - CONTINUOUS
   */
  MCP356xMode mode;

  /**
   * @brief ADC gain setting.
   *
   * Options:
   * - GAIN_ONETHIRD
   * - GAIN_1
   * - GAIN_2
   * - GAIN_4  
   * - GAIN_8
   * - GAIN_16
   * - GAIN_32
   * - GAIN_64
   */
  MCP356xGain gain;

  /**
   * @brief Bias current setting.
   *
   * Options:
   * - NONE
   * - NANOAMPS_900  
   * - NANOAMPS_3700
   * - NANOAMPS_15000 
   */
  MCP356xBiasCurrent bias;

  /**
   * @brief Oversampling ratio.
   *
   * Samples taken per conversion. 
   * Options:
   * - OSR_32
   * - OSR_64
   * - ...
   * - OSR_24576
   * - OSR_40960
   */
  MCP356xOversamplingRatio over;

  /**
   * @brief AMCLK input clock prescaler value.
   *
   * Options:
   * - OVER_1
   * - OVER_2
   * - OVER_4
   * - OVER_8
   */
  MCP356xAMCLKPrescaler prescaler;


      MCP356xConfig() : scan(0), flags(0),
        mode(MCP356xMode::ONESHOT_STANDBY),
        gain(MCP356xGain::GAIN_1),
        bias(MCP356xBiasCurrent::NONE),
        over(MCP356xOversamplingRatio::OSR_256),
        prescaler(MCP356xAMCLKPrescaler::OVER_1) {};

    MCP356xConfig(
      const uint32_t SCAN,
      const uint32_t FLAGS,
      const MCP356xMode MODE,
      const MCP356xGain GAIN,
      const MCP356xBiasCurrent BIAS,
      const MCP356xOversamplingRatio OVER,
      const MCP356xAMCLKPrescaler PRESCALER
    ) : scan(SCAN), flags(FLAGS),
        mode(MODE), gain(GAIN), bias(BIAS),
        over(OVER), prescaler(PRESCALER) {};

    MCP356xConfig(const MCP356xConfig* CFG) : scan(CFG->scan), flags(CFG->flags),
        mode(CFG->mode), gain(CFG->gain), bias(CFG->bias),
        over(CFG->over), prescaler(CFG->prescaler) {};


    ~MCP356xConfig() {};



  };
