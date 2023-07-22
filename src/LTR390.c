/** Based on Adafruit LTR390 Arduino Library **/
/** Changed to work with Zephyr by Tom Waymouth**/
/*!
 *  @file LTR390.cpp
 *
 *  @mainpage Adafruit LTR390 I2C UV and Light Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the LTR390 I2C UV and Light sensor
 *
 * 	This is a library for the Adafruit LTR390 breakout:
 * 	https://www.adafruit.com/product/4831
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried (Adafruit Industries)
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "LTR390.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

const struct device *LTR390_device;
const static float gain_Buf[5] = {1, 3, 6, 9, 18};
const static float resolution_Buf[6] = {4, 2, 1, 0.5, 0.25, 0.03125};

static int writeRegister(uint8_t reg, uint8_t data)
{
  return i2c_reg_write_byte(LTR390_device, LTR390_ADDRESS, reg, data);
}

static int readRegister(uint8_t reg, uint8_t *data)
{
  uint8_t tx_buf[1] = {reg};
  int rc = i2c_write_read(LTR390_device, LTR390_ADDRESS, tx_buf, 1, data, 1);
  //printk("Read %d\n", *data); //for debug

  if (rc < 0)
  {
    return rc;
  }
  return 0;
}

/*!
    @brief  Perform a soft reset with 10ms delay, resets all paramaters to factory default.
    @returns 0 on success (reset bit was cleared post-write). Err code on error
*/
bool LTR390_reset(void)
{
  int rc = 0;
  uint8_t readData;
  rc += writeRegister(LTR390_MAIN_CTRL, 0x10);
  k_msleep(10);
  rc += readRegister(LTR390_MAIN_CTRL, &readData);
  readData += rc;
  return rc;
}
/*!
    @brief  Checks if new data is available in data register
    @returns True on new data available
*/
bool LTR390_newDataAvailable(void)
{
  uint8_t readStatus;
  readRegister(LTR390_MAIN_STATUS, &readStatus);
  readStatus >>= 3;
  readStatus &= 1;
  return readStatus;
}

/*!
    @brief  Read 3-bytes out of selected data register, does not check if data is new, sensor is set to correct mode or perform any conversions!
    @returns Returns 0 on success, negative on failure
    @param reg 1 to read ALS data register, 0 to read UVS data register
    @param *readData Pointer to store raw data read from chosen register
*/
int LTR390_readDataReg(bool reg, uint32_t *readData)
{
  uint8_t readLsb;
  uint8_t readMsb;
  uint8_t readHsb;
  int rc = 0;
  rc += readRegister(reg ? LTR390_ALSDATA_LSB : LTR390_UVSDATA_LSB, &readLsb);
  rc += readRegister(reg ? LTR390_ALSDATA_MSB : LTR390_UVSDATA_MSB, &readMsb);
  rc += readRegister(reg ? LTR390_ALSDATA_HSB : LTR390_UVSDATA_HSB, &readHsb);
  if (rc < 0)
  {
    return rc;
  }
  readHsb &= 0x0F;
  *readData = ((uint32_t)readHsb << 16) | ((uint16_t)readMsb << 8) | readLsb;
  return 0;
}

/*

    @brief  Enable or disable the light sensor
    @param  en True to enable, False to disable
    @returns 0 if succesfully set to chosen state, negative error code on error
*/
int LTR390_enable(bool en)
{
  uint8_t readData;
  int rc = 0;
  rc += readRegister(LTR390_MAIN_CTRL, &readData);
  en ? (readData = (readData & 0x8) | 0x2) : (readData &= 0x8);
  rc += writeRegister(LTR390_MAIN_CTRL, readData);
  rc += readRegister(LTR390_MAIN_CTRL, &readData);
  readData >>= 1;
  readData &= 1; 
  (readData == en) ? (rc = 0) : (rc = -EIO);
  return rc;
}

/*!
    @brief  Read the enabled-bit from the sensor
    @returns True if enabled False if disabled or error
*/
bool LTR390_enabled(void)
{
  uint8_t readData;
  readRegister(LTR390_MAIN_CTRL, &readData);
  readData >>= 1;
  readData &= 1;
  return readData;
}

/*!
 *  @brief  Set the sensor mode to EITHER ambient (LTR390_MODE_ALS) or UV
 * (LTR390_MODE_UVS)
 *  @param  mode The desired mode - LTR390_MODE_UVS or LTR390_MODE_ALS
 * @returns 0 on success.
 */
int LTR390_setMode(ltr390_mode_t mode)
{
  uint8_t readData;
  int rc = 0;
  rc += readRegister(LTR390_MAIN_CTRL, &readData);
  readData &= 0xF7;
  readData |= ((uint8_t)mode << 3);
  rc += writeRegister(LTR390_MAIN_CTRL, (uint8_t)readData);
  k_msleep(20); // sleep for 20ms to allow for mode change
  return rc;
}

/*!
 *  @brief  get the sensor's mode
 *  @returns The current mode - LTR390_MODE_UVS or LTR390_MODE_ALS
 */
ltr390_mode_t LTR390_getMode(void)
{
  uint8_t readData;
  readRegister(LTR390_MAIN_CTRL, &readData);
  readData >>= 3;
  readData &= 1;
  return (ltr390_mode_t)readData;
}

/*!
 *  @brief  Set the sensor gain
 *  @param gain The desired gain: LTR390_GAIN_1, LTR390_GAIN_3, LTR390_GAIN_6
 *  LTR390_GAIN_9 or LTR390_GAIN_18
 * @returns 0 on success.
 */
int LTR390_setGain(ltr390_gain_t gain)
{
  int rc = writeRegister(LTR390_GAIN, (uint8_t)gain);
  return rc;
}

/*!
 *  @brief  Get the sensor's gain
 *  @returns gain The current gain: LTR390_GAIN_1, LTR390_GAIN_3, LTR390_GAIN_6
 *  LTR390_GAIN_9 or LTR390_GAIN_18
 */
ltr390_gain_t LTR390_getGain(void)
{
  uint8_t readData;
  readRegister(LTR390_GAIN, &readData);
  readData &= 7;
  return (ltr390_gain_t)readData;
}

/*!
 *  @brief  Set the sensor resolution. Higher resolutions take longer to read!
 *  @param  res The desired resolution: LTR390_RESOLUTION_13BIT,
 *  LTR390_RESOLUTION_16BIT, LTR390_RESOLUTION_17BIT, LTR390_RESOLUTION_18BIT,
 *  LTR390_RESOLUTION_19BIT or LTR390_RESOLUTION_20BIT
 * @returns 0 on success.
 */
int LTR390_setResolution(ltr390_resolution_t res)
{
  int rc = 0;
  uint8_t readData = 0;
  rc += readRegister(LTR390_MEAS_RATE, &readData);
  readData = (readData & 0x8F) | res << 4;
 //readData |= (res << 4);
  rc += writeRegister(LTR390_MEAS_RATE, (uint8_t)readData);
  return rc;
}

/*!
 *  @brief  Get the sensor's resolution
 *  @returns The current resolution: LTR390_RESOLUTION_13BIT,
 *  LTR390_RESOLUTION_16BIT, LTR390_RESOLUTION_17BIT, LTR390_RESOLUTION_18BIT,
 *  LTR390_RESOLUTION_19BIT or LTR390_RESOLUTION_20BIT
 */
ltr390_resolution_t LTR390_getResolution(void)
{
  uint8_t readData;
  readRegister(LTR390_MEAS_RATE, &readData);
  readData &= 0x70;
  readData = 7 & (readData >> 4);
  return (ltr390_resolution_t)readData;
}

/*!
 *  @brief  Set the sensor sample rate. (How often to take measurements) When measurement rate is set faster than conversion time 
      for programmed resoloution rate will be lower than programmed - See datasheet.- 100ms by default
 *  @param rate The desired rate: LTR390_RATE_25ms, LTR390_RATE_50ms, LTR390_RATE_100ms, LTR390_RATE_200ms,
  LTR390_RATE_500ms, LTR390_RATE_1000ms, LTR390_RATE_2000ms, 
 * @returns 0 on success.
 */
int LTR390_setRate(ltr390_rate_t rate)
{
  uint8_t readData = 0;
  readRegister(LTR390_MEAS_RATE, &readData);
  //readData &= ((rate << 1) + 15);
  readData = (readData & 0xF8) | rate;
  //readData |= (rate << 1);
  int rc = writeRegister(LTR390_MEAS_RATE, (uint8_t)readData);
  return rc;
}

/*!
 *  @brief  Get the sensor's resolution
 *  @return configured rate  - LTR390_RATE_25ms, LTR390_RATE_50ms, LTR390_RATE_100ms, LTR390_RATE_200ms,
  LTR390_RATE_500ms, LTR390_RATE_1000ms, LTR390_RATE_2000ms, 
 */
ltr390_rate_t LTR390_getRate(void)
{
  uint8_t readData;
  readRegister(LTR390_MEAS_RATE, &readData);
  printk("Read %d\n", readData);
  readData &= 0x7;
  return (ltr390_rate_t)readData;
}

/*!
    @brief  Set the interrupt output threshold range for lower and upper.
    When the sensor is below the lower, or above upper, interrupt will fire.
    @param  lower The lower value to compare against the data register.
    @param  higher The higher value to compare against the data register.
    @returns 0 on success
*/
int LTR390_setThresholds(uint32_t lower, uint32_t higher)
{
  uint8_t readData = higher;
  int rc = +writeRegister(LTR390_THRESH_UP, readData);
  readData = higher >> 8;
  rc += writeRegister(LTR390_THRESH_UP + 1, readData);
  readData = higher >> 16;
  readData &= 0x0F;
  rc += writeRegister(LTR390_THRESH_UP + 2, readData);
  readData = lower;
  rc += writeRegister(LTR390_THRESH_LOW, readData);
  readData = lower >> 8;
  rc += writeRegister(LTR390_THRESH_LOW + 1, readData);
  readData = lower >> 16;
  readData &= 0x0F;
  rc += writeRegister(LTR390_THRESH_LOW + 2, readData);
  return rc;
}

/*!
    @brief  Configure the interrupt based on the thresholds in setThresholds()
    When the sensor is below the lower, or above upper thresh, interrupt will
   fire
    @param  enable Whether the interrupt output is enabled
    @param  source Whether to use the ALS or UVS data register to compare
    @param  persistance The number of consecutive out-of-range readings before
            we fire the IRQ. Default is 0 (each reading will fire)
    @returns 0 on success
*/
int LTR390_configInterrupt(bool enable, ltr390_mode_t source, uint8_t persistance)
{
  uint8_t readData = 0;
  int rc = 0;
  readData |= (enable << 2) | (1 << 4) | (source << 5);
  rc += writeRegister(LTR390_INT_CFG, readData);
  if (persistance > 0x0F)
    persistance = 0x0F;
  uint8_t _p = 0;
  _p |= persistance << 4;
  rc += writeRegister(LTR390_INT_PST, _p);
  return rc;
}

/*!
    @brief  get lux data,LUX is calculated according to the formula
    @param lux* pointer to store calculated ambient light data ,unit lux.
    @returns 0 on success
*/
int LTR390_getLUX(float *lux)
{
  uint32_t raw;
  int rc = LTR390_readDataReg(1, &raw);
  uint8_t _gain = (uint8_t)(LTR390_getGain());
  uint8_t readResolution = (uint8_t)(LTR390_getResolution());
  *lux = 0.6 * (float)(raw) / (gain_Buf[_gain] * resolution_Buf[readResolution]) * (float)(WFAC);
  return rc;
}

/*!
    @brief  get UVI data,UVI is calculated according to the formula
    @param uvi* pointer to store calculated ultraviolet light data,unit uw/cm2.
    @returns 0 on success.
*/
int LTR390_getUVI(float *uvi)
{
  uint32_t raw;
  int rc = LTR390_readDataReg(0, &raw);
  uint8_t _gain = (uint8_t)(LTR390_getGain());
  uint8_t readResolution = (uint8_t)(LTR390_getResolution());
  printk("%3.2f Gain\n",gain_Buf[_gain]);
  printk("%3.2f Res\n",resolution_Buf[readResolution]);
  printk("%u Raw\n", raw);
  *uvi = (float)(raw) / ((gain_Buf[_gain] / gain_Buf[LTR390_GAIN_18]) * (resolution_Buf[readResolution] / resolution_Buf[LTR390_RESOLUTION_20BIT]) * (float)(LTR390_SENSITIVITY)) * (float)(WFAC);
  return rc;
}
/*!
    @brief  Verify i2c device provided is enabled, attmpts to read part ID from sensor and then enables sensor.
    @attention By default sensor is reset to factory default at power up, this function does not reset, adjust or validate any settings.
    settings and mode should be validated prior to taking readings, to factory default sensor if required use LTR390_reset() function.
    @param *dev pointer to i2c device to which sensor is connected.
    @returns 0 if part number able to be read from sensor and sensor successfully enabled.
*/
int LTR390_init(const struct device *dev)
{
  uint8_t ltrId;
  LTR390_device = dev;
  if (!device_is_ready(LTR390_device))
  {
    printk("Device not ready");
    return -ENODEV;
  }
  int rc = readRegister(LTR390_PART_ID, &ltrId);
  if (rc < 0)
  {
    return rc;
  }
  // check part ID!
  if ((ltrId >> 4) != 0x0B)
  {
    return -EIO;
  }
  // successfully enabled sensor
  if (LTR390_enable(true))
  {
    return -EIO;
  }
  return 0;
}