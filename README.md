# Zephyr LTR390
Zephyr Driver for LTR390 Sensor.

This is a basic Zephyr driver for the liteon LTR390 sensor based on modifactions of existing drivers refrenced under license.

[License](https://github.com/twaymouth/Zephyr_LTR390/edit/main/README.md#license)

# License
This code is based on the [RAKWireless LTR390 Driver](https://github.com/RAKWireless/RAK12019_LTR390/tree/main) which in turn is based on the [Adafruit_LTR390 Driver](https://github.com/adafruit/Adafruit_LTR390) with specicif changes made to work with Zephyr and improve error handling. 
 # Documentation

 # Installation

 # Basic Usage

 # Functions

* `UVlight_LTR390()`;
* `UVlight_LTR390(TwoWire *w)`;
* `UVlight_LTR390(int addr)`;
* `UVlight_LTR390(TwoWire *w, int addr)`;
  Constructor
  
* `bool init()`;
  Initialization. Returns true on success, or false.

* `bool reset(void)`;
  Performs a soft reset with a 10-ms delay. Returns true on success, or false.

* `void enable(bool en)`;
  Enables or disables the light sensor.

* `bool enabled(void)`;
  Reads the enabled-bit from the sensor.

* `void setMode(ltr390_mode_t mode)`;
  Sets the sensor mode to either ambient (`LTR390_MODE_ALS`) or UV (`LTR390_MODE_UVS`).

* `ltr390_mode_t getMode(void)`;
  Gets the current mode: `LTR390_MODE_UVS` or `LTR390_MODE_ALS`.

* `void setGain(ltr390_gain_t gain)`;
  Sets the sensor gain. See `ltr390_gain_t`.

* `ltr390_gain_t getGain(void)`;
  Gets the current gain.

* `void setResolution(ltr390_resolution_t res)`;
  Sets the sensor resolution. Note: higher resolutions take longer to read. See `ltr390_resolution_t`.

* `ltr390_resolution_t getResolution(void)`;
  Gets the sensor resolution.

* `void setThresholds(uint32_t lower, uint32_t higher)`;
  Sets the lower and upper interrupt output threshold ranges. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire.

* `void configInterrupt(bool enable, ltr390_mode_t source, uint8_t persistance = 0)`;
  Configures the interrupt based on the thresholds in `setThresholds()`. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire.

* `bool newDataAvailable(void)`;
  Checks whether new data is available in the data register.

* `uint32_t readUVS(void)`;
  Reads 3 bytes out of UV data register, but does not check if data is new.

* `uint32_t readALS(void)`;
  Reads 3 bytes out of ambient data register, but does not check if data is new.

* `float getLUX(void)`;
  Gets lux data. Returns the ambient light data in lux.

* `float getUVI(void)`;
  Gets UVI data. Returns the ultraviolet light data in uw/cm2.

* `uint8_t writeRegister(uint8_t reg, uint8_t data)`;
  Writes one byte to a register.

* `uint8_t readRegister(uint8_t reg)`;
  Writes a register's value.
