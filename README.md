# Zephyr LTR390
Zephyr Driver for LTR390 Sensor.

This is a basic Zephyr driver for the liteon LTR390 sensor based on modifications of existing drivers referenced under license.

[License](https://github.com/twaymouth/Zephyr_LTR390/edit/main/README.md#license)

## License
This code is based on the [RAKWireless LTR390 Driver](https://github.com/RAKWireless/RAK12019_LTR390/tree/main) which in turn is based on the [Adafruit_LTR390 Driver](https://github.com/adafruit/Adafruit_LTR390) with specific changes in order to work with Zephyr and improve error handling. 
 ## Documentation
Todo - upload data sheet
 ## Installation
 

 ## Basic Usage

 ## Functions

* `bool ltr390_init()`;
  Initialization. Returns true on success, or false.

* `bool ltr390_reset(void)`;
  Performs a soft reset with a 10-ms delay. Returns true on success, or false.

* `void ltr390_enable(bool en)`;
  Enables or disables the light sensor.

* `bool ltr390_enabled(void)`;
  Reads the enabled-bit from the sensor.

* `void ltr390_setMode(ltr390_mode_t mode)`;
  Sets the sensor mode to either ambient (`LTR390_MODE_ALS`) or UV (`LTR390_MODE_UVS`).

* `ltr390_mode_t ltr390_getMode(void)`;
  Gets the current mode: `LTR390_MODE_UVS` or `LTR390_MODE_ALS`.

* `void ltr390_setGain(ltr390_gain_t gain)`;
  Sets the sensor gain. See `ltr390_gain_t`.

* `ltr390_gain_t ltr390_getGain(void)`;
  Gets the current gain.

* `void ltr390_setResolution(ltr390_resolution_t res)`;
  Sets the sensor resolution. Note: higher resolutions take longer to read. See `ltr390_resolution_t`.

* `ltr390_resolution_t ltr390_getResolution(void)`;
  Gets the sensor resolution.

* `void ltr390_setThresholds(uint32_t lower, uint32_t higher)`;
  Sets the lower and upper interrupt output threshold ranges. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire.

* `void ltr390_configInterrupt(bool enable, ltr390_mode_t source, uint8_t persistance = 0)`;
  Configures the interrupt based on the thresholds in `setThresholds()`. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire.

* `bool ltr390_newDataAvailable(void)`;
  Checks whether new data is available in the data register.

* `uint32_t ltr390_readReg(void)`;
  Reads 3 bytes out of UV data register, but does not check if data is new.

* `float ltr390_getLUX(void)`;
  Gets lux data. Returns the ambient light data in lux.

* `float ltr390_getUVI(void)`;
  Gets UVI data. Returns the ultraviolet light data in uw/cm2.

* `uint8_t writeRegister(uint8_t reg, uint8_t data)`;
  Writes one byte to a register. - Used internally in ltr390.h

* `uint8_t readRegister(uint8_t reg)`;
  Writes a register's value. Used internally in ltr390.h
