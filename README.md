# Zephyr LTR390
Zephyr Driver for LTR390 Sensor.

This is a basic Zephyr driver for the liteon LTR390 sensor based on modifications of existing drivers referenced under license.

Note - This is somthing I threw together for my own use, based on my testing it works as expected however my programming expertises is that of a bean dip so you have been warned. 

TBC- The readings I am recieving are off by what seems a factor of 4, this is being investigated. 
~~This driver uses a slightly different formula to calculate the UVI compared to others, this modified formula does not take the sensors configured resolution into consideration as based on real world testing I do not believe it is required.~~

~~If the resolution is included in the calculation the calculated sensativity (counts per UVI) is incorrect and the resultant UV index value is to high.~~

## License
This code is based on the [RAKWireless LTR390 Driver](https://github.com/RAKWireless/RAK12019_LTR390/tree/main) which in turn is based on the [Adafruit_LTR390 Driver](https://github.com/adafruit/Adafruit_LTR390) with specific changes in order to work with Zephyr and improve error handling. 
 ## Hardware
 LTR390 sensor breakout is available from a number of vendors:
 - Adafruit - https://www.adafruit.com/product/4831
 - Waveshare - https://www.waveshare.com/uv-sensor-c.htm
 - Various other options from Aliexpress and the like however these are mostly clones of the Adafruit design.

 For ease of use I would recomend the Adafruit breakout, the Waveshare is also nice however requires modifications in order to function with certian microcontrollers (i.e. NRF family) due to the low value pull up resistors used. 

 Both require modification in order to function in a truly low power state, however due to the design of the Adafruit board these are minimal, i.e. remove the power LED and power directly from 3.3v. The Waveshare on the other hand almost all components need to be removed apart from the actual sensor and associated filter capacitors in order to achieve a low power consumption. 
 
 ## Installation

Copy LTR390.c and LTR390.h into your projects source directory and import as applicable based on your projects directory structure e.g. `#include "LTR390.h"`

 ## Basic Usage
 
 Basic usage using sensors default power up settings and mode (Lux mode)
 
 Import driver as above.
 
 Declare instance of i2c device to which sensor is connected:
 
 `const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));`
 
 Initalise sensor:
 ```
int err = LTR390_init(i2c_dev);
if (err){
printk("Sensor Init Err %d\n", err);
}
```
Take readings
```
float lux = 0;
while (!LTR390_newDataAvailable()){
k_msleep(20);
}
if (!LTR390_getLUX(&lux)){
printf("Lux read %3.2f\n", lux);
}
```

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
  Sets the sensor mode to either ambient (`LTR390_MODE_ALS`) or UV (`LTR390_MODE_UVS`). Returns 0 on success.

* `ltr390_mode_t LTR390_getMode(void)`
  Gets the current mode: `LTR390_MODE_UVS` or `LTR390_MODE_ALS`.

* `int LTR390_setGain(ltr390_gain_t gain)`
  Sets the sensor gain. Returns 0 on success.

* `ltr390_gain_t LTR390_getGain(void)`
  Gets the current gain.

* `int LTR390_setResolution(ltr390_resolution_t res)`
  Sets the sensor resolution. Note: higher resolutions take longer to read. See `ltr390_resolution_t`. Returns 0 on success.

* `LTR390_resolution_t ltr390_getResolution(void)`
  Gets the sensor resolution.

* `int LTR390_setRate(ltr390_rate_t res)`
  Set sensor measurement rate. Returns 0 on success.
  
* `ltr390_rate_t LTR390_getRate(void)`
  Get sensor measurement rate. 

* `int LTR390_setThresholds(uint32_t lower, uint32_t higher)`
  Sets the lower and upper interrupt output threshold ranges. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire. Returns 0 on success.

* `int LTR390_configInterrupt(bool enable, ltr390_mode_t source, uint8_t persistance = 0)`
  Configures the interrupt based on the thresholds in `setThresholds()`. When the sensor is below the lower threshold, or above the upper threshold, interrupt will fire. Returns 0 on success.

* `bool LTR390_newDataAvailable(void)`
  Checks whether new data is available in the data register.

* `int LTR390_readDataReg(bool reg, uint32_t *readData)`
  ReRead 3-bytes out of selected data register, does not check if data is new, sensor is set to correct mode or perform any conversions!. Returns 0 on success.

* `int LTR390_getLUX(float *lux)`
  Gets lux data. Returns the ambient light data in lux. Returns 0 on success.

* `int LTR390_getUVI(float *uvi)`
  Gets UVI data. Returns the ultraviolet light data in uw/cm2. Returns 0 on success.

* `static int writeRegister(uint8_t reg, uint8_t data)`
  Writes one byte to a register. Used internally in ltr390.h

* `static int readRegister(uint8_t reg, uint8_t *data)`
  Reads a register's value. Used internally in ltr390.h
