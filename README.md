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

* `bool LTR390_init(const struct device *dev)`
  Verify i2c device provided is enabled, attmpts to read part ID from sensor and then enables sensor. Returns 0 if part number able to be read from sensor and sensor successfully enabled.

* `bool LTR390_reset(void)`
  Performs a soft reset with a 10-ms delay. Returns 0 on success, error code on error.

* `int LTR390_enable(bool en)`
  Enables or disables the light sensor, returns 0 on success.

* `bool LTR390_enabled(void)`
  Reads the enabled-bit from the sensor.

* `int LTR390_setMode(ltr390_mode_t mode)`
  Sets the sensor mode to either ambient (`LTR390_MODE_ALS`) or UV (`LTR390_MODE_UVS`). Returns 0 on success

* `ltr390_mode_t LTR390_getMode(void)`
  Gets the current mode: `LTR390_MODE_UVS` or `LTR390_MODE_ALS`.

* `int LTR390_setGain(ltr390_gain_t gain)`
  Sets the sensor gain. See `ltr390_gain_t, returns 0 on success`.

* `ltr390_gain_t LTR390_getGain(void)`
  Gets the current gain.

* `int LTR390_setResolution(ltr390_resolution_t res)`
  Sets the sensor resolution. Note: higher resolutions take longer to read. See `ltr390_resolution_t`. Returns 0 on success

* `LTR390_resolution_t ltr390_getResolution(void)`
  Gets the sensor resolution.

* `int LTR390_setThresholds(uint32_t lower, uint32_t higher)`
  Sets the lower and upper interrupt output threshold ranges. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire. Returns 0 on success

* `int LTR390_configInterrupt(bool enable, ltr390_mode_t source, uint8_t persistance = 0)`
  Configures the interrupt based on the thresholds in `setThresholds()`. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire. Returns 0 on success

* `bool LTR390_newDataAvailable(void)`
  Checks whether new data is available in the data register.

* `int LTR390_readDataReg(bool reg, uint32_t *readData)`
  ReRead 3-bytes out of selected data register, does not check if data is new, sensor is set to correct mode or perform any conversions!. Returns 0 on success.

* `int LTR390_getLUX(float *lux)`
  Gets lux data. Returns the ambient light data in lux. Returns 0 on success

* `int LTR390_getUVI(float *uvi)`
  Gets UVI data. Returns the ultraviolet light data in uw/cm2. Returns 0 on success

* `static int writeRegister(uint8_t reg, uint8_t data)`
  Writes one byte to a register. Used internally in ltr390.h

* `static int readRegister(uint8_t reg, uint8_t *data)`
  Reads a register's value. Used internally in ltr390.h
