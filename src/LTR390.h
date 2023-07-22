/** Based on Adafruit LTR390 Arduino Library **/
/** Changed to work with Zephyr **/
/*!
 *  @file LTR390.h
 *
 * 	I2C Driver for LTR390 UV and light sensor
 *
 * 	This is a library for the Adafruit LTR390 breakout:
 * 	http://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#include <zephyr/device.h>

#define LTR390_SENSITIVITY 1400

#define WFAC 1
/*
   For device under tinted window with coated-ink of flat transmission rate at 400-600nm wavelength,
   window factor  is to  compensate light  loss due to the  lower  transmission rate from the coated-ink.
      a. WFAC = 1 for NO window / clear window glass.
      b. WFAC >1 device under tinted window glass. Calibrate under white LED.
*/

#define LTR390_ADDRESS 0x53     ///< I2C address
#define LTR390_MAIN_CTRL 0x00   ///< Main control register
#define LTR390_MEAS_RATE 0x04   ///< Resolution and data rate
#define LTR390_GAIN 0x05        ///< ALS and UVS gain range
#define LTR390_PART_ID 0x06     ///< Part id/revision register
#define LTR390_MAIN_STATUS 0x07 ///< Main status register
#define LTR390_ALSDATA_LSB 0x0D ///< ALS data lowest byte
#define LTR390_ALSDATA_MSB 0x0E ///< ALS data middle byte
#define LTR390_ALSDATA_HSB 0x0F ///< ALS data highest byte
#define LTR390_UVSDATA_LSB 0x10 ///< UVS data lowest byte
#define LTR390_UVSDATA_MSB 0x11 ///< UVS data middle byte
#define LTR390_UVSDATA_HSB 0x12 ///< UVS data highest byte
#define LTR390_INT_CFG 0x19     ///< Interrupt configuration
#define LTR390_INT_PST 0x1A     ///< Interrupt persistance config
#define LTR390_THRESH_UP 0x21   ///< Upper threshold, low byte
#define LTR390_THRESH_LOW 0x24  ///< Lower threshold, low byte


/*!    @brief  Whether we are measuring ambient or UV light  */
typedef enum
{
  LTR390_MODE_ALS,
  LTR390_MODE_UVS,
} ltr390_mode_t;

/*!    @brief  Sensor gain for UV or ALS  */
typedef enum
{
  LTR390_GAIN_1 = 0,
  LTR390_GAIN_3,
  LTR390_GAIN_6,
  LTR390_GAIN_9,
  LTR390_GAIN_18,
} ltr390_gain_t;

/*!    @brief Measurement resolution (higher res means slower reads!)  */
typedef enum
{
  LTR390_RESOLUTION_20BIT,
  LTR390_RESOLUTION_19BIT,
  LTR390_RESOLUTION_18BIT,
  LTR390_RESOLUTION_17BIT,
  LTR390_RESOLUTION_16BIT,
  LTR390_RESOLUTION_13BIT,
} ltr390_resolution_t;

/*!    @brief Measurement rate (How often to take measurements) When measurement rate is set faster than conversion time 
      for programmed resoloution rate will be lower than programmed - See datasheet.- 100ms by default
  */
typedef enum
{
  LTR390_RATE_25ms,
  LTR390_RATE_50ms,
  LTR390_RATE_100ms,
  LTR390_RATE_200ms,
  LTR390_RATE_500ms,
  LTR390_RATE_1000ms,
  LTR390_RATE_2000ms,
} ltr390_rate_t;

int LTR390_init(const struct device *dev);

bool LTR390_reset(void);

int LTR390_enable(bool en);
bool LTR390_enabled(void);

int LTR390_setMode(ltr390_mode_t mode);
ltr390_mode_t LTR390_getMode(void);

int LTR390_setGain(ltr390_gain_t gain);
ltr390_gain_t LTR390_getGain(void);

int LTR390_setResolution(ltr390_resolution_t res);
ltr390_resolution_t LTR390_getResolution(void);

int LTR390_setRate(ltr390_rate_t res);
ltr390_rate_t LTR390_getRate(void);

int LTR390_setThresholds(uint32_t lower, uint32_t higher);

int LTR390_configInterrupt(bool enable, ltr390_mode_t source, uint8_t persistance);

bool LTR390_newDataAvailable(void);
int LTR390_readDataReg(bool reg, uint32_t *readData);

int LTR390_getLUX(float *lux);

int LTR390_getUVI(float *uvi);


