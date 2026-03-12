/**
 ******************************************************************************
 * @file    LSM6DSO32Sensor.cpp
 * @author  NG
 * @version V1.0.0
 * @date    November 2025
 * @brief   Implementation of an LSM6DSO32 Inertial Measurement Unit (IMU) 3 axes
 *          sensor.
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LSM6DSO32Sensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LSM6DSO32Sensor::LSM6DSO32Sensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  reg_ctx.write_reg = LSM6DSO32_io_write;
  reg_ctx.read_reg = LSM6DSO32_io_read;
  reg_ctx.handle = (void *)this;
  acc_is_enabled = 0;
  gyro_is_enabled = 0;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LSM6DSO32Sensor::LSM6DSO32Sensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LSM6DSO32_io_write;
  reg_ctx.read_reg = LSM6DSO32_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    /* Disable I2C */
    if (lsm6dso32_i2c_interface_set(&reg_ctx, LSM6DSO32_I2C_DISABLE) != LSM6DSO32_OK) {
      return LSM6DSO32_ERROR;
    }
  }

  /* Disable I3C */
  if (lsm6dso32_i3c_disable_set(&reg_ctx, LSM6DSO32_I3C_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lsm6dso32_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable BDU */
  if (lsm6dso32_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* FIFO mode selection */
  if (lsm6dso32_fifo_mode_set(&reg_ctx, LSM6DSO32_BYPASS_MODE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Select default output data rate. */
  acc_odr = LSM6DSO32_XL_ODR_104Hz_HIGH_PERF;

  /* Output data rate selection - power down. */
  if (lsm6dso32_xl_data_rate_set(&reg_ctx, LSM6DSO32_XL_ODR_OFF) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dso32_xl_full_scale_set(&reg_ctx, LSM6DSO32_4g) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Select default output data rate. */
  gyro_odr = LSM6DSO32_GY_ODR_104Hz_HIGH_PERF;

  /* Output data rate selection - power down. */
  if (lsm6dso32_gy_data_rate_set(&reg_ctx, LSM6DSO32_GY_ODR_OFF) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dso32_gy_full_scale_set(&reg_ctx, LSM6DSO32_2000dps) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  acc_is_enabled = 0;
  gyro_is_enabled = 0;

  return LSM6DSO32_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::end()
{
  /* Disable both acc and gyro */
  if (Disable_X() != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (Disable_G() != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset CS configuration */
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, INPUT);
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Read component ID
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::ReadID(uint8_t *Id)
{
  if (lsm6dso32_device_id_get(&reg_ctx, Id) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable the LSM6DSO32 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_X()
{
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U) {
    return LSM6DSO32_OK;
  }

  /* Output data rate selection. */
  if (lsm6dso32_xl_data_rate_set(&reg_ctx, acc_odr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  acc_is_enabled = 1;

  return LSM6DSO32_OK;
}

/**
 * @brief  Disable the LSM6DSO32 accelerometer sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_X()
{
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U) {
    return LSM6DSO32_OK;
  }

  /* Get current output data rate. */
  if (lsm6dso32_xl_data_rate_get(&reg_ctx, &acc_odr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dso32_xl_data_rate_set(&reg_ctx, LSM6DSO32_XL_ODR_OFF) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  acc_is_enabled = 0;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 accelerometer sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_Sensitivity(float *Sensitivity)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_fs_xl_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso32_xl_full_scale_get(&reg_ctx, &full_scale) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSO32_4g:
      *Sensitivity = LSM6DSO32_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSO32_8g:
      *Sensitivity = LSM6DSO32_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSO32_16g:
      *Sensitivity = LSM6DSO32_ACC_SENSITIVITY_FS_16G;
      break;

    case LSM6DSO32_32g:
      *Sensitivity = LSM6DSO32_ACC_SENSITIVITY_FS_32G;
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSO32 accelerometer sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_ODR(float *Odr)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  LSM6DSO32_ACC_Operating_Mode_t unused_mode;

  ret = Get_X_ODR_With_Mode(Odr, &unused_mode);
  (void)unused_mode; //unused

  return ret;
}

/**
 * @brief  Get the LSM6DSO32 accelerometer sensor output data rate and mode
 * @param  Odr pointer where the output data rate is written
 * @param  Mode pointer where the output operating mode is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_ODR_With_Mode(float *Odr, LSM6DSO32_ACC_Operating_Mode_t *Mode)
{

  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_odr_xl_t odr_low_level;

  /* Get current output data rate and mode. */
  if (lsm6dso32_xl_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Extract mode byte */
  uint16_t mode_byte_odr_low_level = (odr_low_level & 0x30) >> 4;

  switch (mode_byte_odr_low_level) {
    case 0x0:
      *Mode = LSM6DSO32_ACC_HIGH_PERFORMANCE_MODE;
      break;
    case 0x1:
      *Mode = LSM6DSO32_ACC_LOW_POWER_NORMAL_MODE;
      break;
    case 0x2:
      *Mode = LSM6DSO32_ACC_ULTRA_LOW_POWER_MODE;
      break;
    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  /* Get ODR */
  switch (odr_low_level) {
    case LSM6DSO32_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSO32_XL_ODR_1Hz6_LOW_PW:
    case LSM6DSO32_XL_ODR_1Hz6_ULTRA_LOW_PW:
      *Odr = 1.6f;
      break;

    case LSM6DSO32_XL_ODR_12Hz5_LOW_PW:
    case LSM6DSO32_XL_ODR_12Hz5_HIGH_PERF:
    case LSM6DSO32_XL_ODR_12Hz5_ULTRA_LOW_PW:
      *Odr = 12.5f;
      break;

    case LSM6DSO32_XL_ODR_26Hz_LOW_PW:
    case LSM6DSO32_XL_ODR_26Hz_HIGH_PERF:
    case LSM6DSO32_XL_ODR_26Hz_ULTRA_LOW_PW:
      *Odr = 26.0f;
      break;

    case LSM6DSO32_XL_ODR_52Hz_LOW_PW:
    case LSM6DSO32_XL_ODR_52Hz_HIGH_PERF:
    case LSM6DSO32_XL_ODR_52Hz_ULTRA_LOW_PW:
      *Odr = 52.0f;
      break;

    case LSM6DSO32_XL_ODR_104Hz_NORMAL_MD:
    case LSM6DSO32_XL_ODR_104Hz_HIGH_PERF:
    case LSM6DSO32_XL_ODR_104Hz_ULTRA_LOW_PW:
      *Odr = 104.0f;
      break;

    case LSM6DSO32_XL_ODR_208Hz_NORMAL_MD:
    case LSM6DSO32_XL_ODR_208Hz_HIGH_PERF:
    case LSM6DSO32_XL_ODR_208Hz_ULTRA_LOW_PW:
      *Odr = 208.0f;
      break;

    case LSM6DSO32_XL_ODR_417Hz_HIGH_PERF:
      *Odr = 417.0f;
      break;

    case LSM6DSO32_XL_ODR_833Hz_HIGH_PERF:
      *Odr = 833.0f;
      break;

    case LSM6DSO32_XL_ODR_1667Hz_HIGH_PERF:
      *Odr = 1667.0f;
      break;

    case LSM6DSO32_XL_ODR_3333Hz_HIGH_PERF:
      *Odr = 3333.0f;
      break;

    case LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF:
      *Odr = 6667.0f;
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSO32 accelerometer sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_X_ODR(float Odr)
{
  return Set_X_ODR_With_Mode(Odr, LSM6DSO32_ACC_HIGH_PERFORMANCE_MODE);
}

/**
 * @brief  Set the LSM6DSO32 accelerometer sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the accelerometer operating mode
 * @note   This function switches off the gyroscope if Ultra Low Power Mode is set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_X_ODR_With_Mode(float Odr, LSM6DSO32_ACC_Operating_Mode_t Mode)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;

  switch (Mode) {
    case LSM6DSO32_ACC_HIGH_PERFORMANCE_MODE: {
        /* In HP mode ODR should be at least 12.5Hz */
        if (Odr < 12.5f) {
          Odr = 12.5f;
        }
        break;
      }
    case LSM6DSO32_ACC_LOW_POWER_NORMAL_MODE: {
        /* Now we need to limit the ODR to 208 Hz if it is higher */
        if (Odr > 208.0f) {
          Odr = 208.0f;
        }
        break;
      }
    case LSM6DSO32_ACC_ULTRA_LOW_POWER_MODE: {
        /* Disable Gyro */
        if (gyro_is_enabled == 1U) {
          if (Disable_G() != LSM6DSO32_OK) {
            return LSM6DSO32_ERROR;
          }
        }

        /* Now we need to limit the ODR to 208 Hz if it is higher */
        if (Odr > 208.0f) {
          Odr = 208.0f;
        }
        break;
      }
    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  /* Check if the component is enabled */
  if (acc_is_enabled == 1U) {
    ret = Set_X_ODR_When_Enabled(Odr, Mode);
  } else {
    ret = Set_X_ODR_When_Disabled(Odr, Mode);
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSO32 accelerometer sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_X_ODR_When_Enabled(float Odr, LSM6DSO32_ACC_Operating_Mode_t Mode)
{
  uint8_t new_odr;

  // Set the 4 ODR bits
  new_odr = (Odr <=    1.6f) ? LSM6DSO32_XL_ODR_1Hz6_LOW_PW
            : (Odr <=   12.5f) ? LSM6DSO32_XL_ODR_12Hz5_HIGH_PERF
            : (Odr <=   26.0f) ? LSM6DSO32_XL_ODR_26Hz_HIGH_PERF
            : (Odr <=   52.0f) ? LSM6DSO32_XL_ODR_52Hz_HIGH_PERF
            : (Odr <=  104.0f) ? LSM6DSO32_XL_ODR_104Hz_HIGH_PERF
            : (Odr <=  208.0f) ? LSM6DSO32_XL_ODR_208Hz_HIGH_PERF
            : (Odr <=  417.0f) ? LSM6DSO32_XL_ODR_417Hz_HIGH_PERF
            : (Odr <=  833.0f) ? LSM6DSO32_XL_ODR_833Hz_HIGH_PERF
            : (Odr <= 1667.0f) ? LSM6DSO32_XL_ODR_1667Hz_HIGH_PERF
            : (Odr <= 3333.0f) ? LSM6DSO32_XL_ODR_3333Hz_HIGH_PERF
            :                    LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF;

  // Reset and apply mode bits
  new_odr &= 0x0FU;
  switch (Mode) {
    case LSM6DSO32_ACC_HIGH_PERFORMANCE_MODE: {
        new_odr |= 0x00U;
        break;
      }
    case LSM6DSO32_ACC_LOW_POWER_NORMAL_MODE: {
        new_odr |= 0x10U;
        break;
      }
    case LSM6DSO32_ACC_ULTRA_LOW_POWER_MODE: {
        new_odr |= 0x20U;
        break;
      }
    default:
      return LSM6DSO32_ERROR;
      break;
  }

  /* Output data rate selection. */
  if (lsm6dso32_xl_data_rate_set(&reg_ctx, (lsm6dso32_odr_xl_t)new_odr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 accelerometer sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_X_ODR_When_Disabled(float Odr, LSM6DSO32_ACC_Operating_Mode_t Mode)
{
  // Set the 4 ODR bits
  uint8_t temp_acc_odr = 0;

  temp_acc_odr = (Odr <=    1.6f) ? LSM6DSO32_XL_ODR_1Hz6_LOW_PW
                 : (Odr <=   12.5f) ? LSM6DSO32_XL_ODR_12Hz5_HIGH_PERF
                 : (Odr <=   26.0f) ? LSM6DSO32_XL_ODR_26Hz_HIGH_PERF
                 : (Odr <=   52.0f) ? LSM6DSO32_XL_ODR_52Hz_HIGH_PERF
                 : (Odr <=  104.0f) ? LSM6DSO32_XL_ODR_104Hz_HIGH_PERF
                 : (Odr <=  208.0f) ? LSM6DSO32_XL_ODR_208Hz_HIGH_PERF
                 : (Odr <=  417.0f) ? LSM6DSO32_XL_ODR_417Hz_HIGH_PERF
                 : (Odr <=  833.0f) ? LSM6DSO32_XL_ODR_833Hz_HIGH_PERF
                 : (Odr <= 1667.0f) ? LSM6DSO32_XL_ODR_1667Hz_HIGH_PERF
                 : (Odr <= 3333.0f) ? LSM6DSO32_XL_ODR_3333Hz_HIGH_PERF
                 :                    LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF;

  // Reset and apply mode bits
  temp_acc_odr &= 0x0FU;
  switch (Mode) {
    case LSM6DSO32_ACC_HIGH_PERFORMANCE_MODE: {
        temp_acc_odr |= 0x00U;
        break;
      }
    case LSM6DSO32_ACC_LOW_POWER_NORMAL_MODE: {
        temp_acc_odr |= 0x10U;
        break;
      }
    case LSM6DSO32_ACC_ULTRA_LOW_POWER_MODE: {
        temp_acc_odr |= 0x20U;
        break;
      }
    default:
      return LSM6DSO32_ERROR;
      break;
  }

  acc_odr = (lsm6dso32_odr_xl_t)temp_acc_odr;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 accelerometer sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_FS(int32_t *FullScale)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_fs_xl_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso32_xl_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSO32_4g:
      *FullScale =  4;
      break;

    case LSM6DSO32_8g:
      *FullScale =  8;
      break;

    case LSM6DSO32_16g:
      *FullScale = 16;
      break;

    case LSM6DSO32_32g:
      *FullScale = 32;
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSO32 accelerometer sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_X_FS(int32_t FullScale)
{
  lsm6dso32_fs_xl_t new_fs;

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 4)  ? LSM6DSO32_4g
           : (FullScale <= 8)  ? LSM6DSO32_8g
           : (FullScale <= 16) ? LSM6DSO32_16g
           :                     LSM6DSO32_32g;

  if (lsm6dso32_xl_full_scale_set(&reg_ctx, new_fs) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}


/**
 * @brief  Get the LSM6DSO32 accelerometer sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_AxesRaw(int16_t *Value)
{

  /* Read raw data values. */
  if (lsm6dso32_acceleration_raw_get(&reg_ctx, Value) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}


/**
 * @brief  Get the LSM6DSO32 accelerometer sensor axes
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_Axes(int32_t *Acceleration)
{
  int16_t data_raw[3];
  float sensitivity = 0.0f;

  /* Read raw data values. */
  if (lsm6dso32_acceleration_raw_get(&reg_ctx, data_raw) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Get LSM6DSO32 actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float)((float)data_raw[0] * sensitivity));
  Acceleration[1] = (int32_t)((float)((float)data_raw[1] * sensitivity));
  Acceleration[2] = (int32_t)((float)((float)data_raw[2] * sensitivity));

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable the LSM6DSO32 gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_G()
{
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U) {
    return LSM6DSO32_OK;
  }

  /* Output data rate selection. */
  if (lsm6dso32_gy_data_rate_set(&reg_ctx, gyro_odr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  gyro_is_enabled = 1;

  return LSM6DSO32_OK;
}


/**
 * @brief  Disable the LSM6DSO32 gyroscope sensor
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_G()
{
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U) {
    return LSM6DSO32_OK;
  }

  /* Get current output data rate. */
  if (lsm6dso32_gy_data_rate_get(&reg_ctx, &gyro_odr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dso32_gy_data_rate_set(&reg_ctx, LSM6DSO32_GY_ODR_OFF) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  gyro_is_enabled = 0;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 gyroscope sensor sensitivity
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_G_Sensitivity(float *Sensitivity)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_fs_g_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso32_gy_full_scale_get(&reg_ctx, &full_scale) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSO32_125dps:
      *Sensitivity = LSM6DSO32_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSO32_250dps:
      *Sensitivity = LSM6DSO32_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSO32_500dps:
      *Sensitivity = LSM6DSO32_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSO32_1000dps:
      *Sensitivity = LSM6DSO32_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSO32_2000dps:
      *Sensitivity = LSM6DSO32_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSO32 gyroscope sensor mode and output data rate
 * @param  Odr and Mode pointer where the output data rate and sensor mode is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_G_ODR_With_Mode(float *Odr, LSM6DSO32_GYRO_Operating_Mode_t *Mode)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_odr_g_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dso32_gy_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Extract mode byte */
  uint16_t mode_byte_odr_low_level = (odr_low_level & 0x10) >> 4;

  switch (mode_byte_odr_low_level) {
    case 0x0:
      *Mode = LSM6DSO32_GYRO_HIGH_PERFORMANCE_MODE;
      break;
    case 0x1:
      *Mode = LSM6DSO32_GYRO_LOW_POWER_NORMAL_MODE;
      break;
    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  switch (odr_low_level) {
    case LSM6DSO32_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSO32_GY_ODR_12Hz5_HIGH_PERF:
    case LSM6DSO32_GY_ODR_12Hz5_LOW_PW:
      *Odr = 12.5f;
      break;

    case LSM6DSO32_GY_ODR_26Hz_HIGH_PERF:
    case LSM6DSO32_GY_ODR_26Hz_LOW_PW:
      *Odr = 26.0f;
      break;

    case LSM6DSO32_GY_ODR_52Hz_HIGH_PERF:
    case LSM6DSO32_GY_ODR_52Hz_LOW_PW:
      *Odr = 52.0f;
      break;

    case LSM6DSO32_GY_ODR_104Hz_HIGH_PERF:
    case LSM6DSO32_GY_ODR_104Hz_NORMAL_MD:
      *Odr = 104.0f;
      break;

    case LSM6DSO32_GY_ODR_208Hz_HIGH_PERF:
    case LSM6DSO32_GY_ODR_208Hz_NORMAL_MD:
      *Odr = 208.0f;
      break;

    case LSM6DSO32_GY_ODR_417Hz_HIGH_PERF:
      *Odr = 417.0f;
      break;

    case LSM6DSO32_GY_ODR_833Hz_HIGH_PERF:
      *Odr = 833.0f;
      break;

    case LSM6DSO32_GY_ODR_1667Hz_HIGH_PERF:
      *Odr =  1667.0f;
      break;

    case LSM6DSO32_GY_ODR_3333Hz_HIGH_PERF:
      *Odr =  3333.0f;
      break;

    case LSM6DSO32_GY_ODR_6667Hz_HIGH_PERF:
      *Odr =  6667.0f;
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSO32 gyroscope sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_G_ODR(float *Odr)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  LSM6DSO32_GYRO_Operating_Mode_t unused_mode;

  ret = Get_G_ODR_With_Mode(Odr, &unused_mode);
  (void)unused_mode; //unused

  return ret;
}

/**
 * @brief  Set the LSM6DSO32 gyroscope sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_G_ODR(float Odr)
{
  return Set_G_ODR_With_Mode(Odr, LSM6DSO32_GYRO_HIGH_PERFORMANCE_MODE);
}

/**
 * @brief  Set the LSM6DSO32 gyroscope sensor output data rate with operating mode
 * @param  Odr the output data rate value to be set
 * @param  Mode the gyroscope operating mode
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_G_ODR_With_Mode(float Odr, LSM6DSO32_GYRO_Operating_Mode_t Mode)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;

  switch (Mode) {
    case LSM6DSO32_GYRO_HIGH_PERFORMANCE_MODE: {
        break;
      }
    case LSM6DSO32_GYRO_LOW_POWER_NORMAL_MODE: {
        /* Now we need to limit the ODR to 208 Hz if it is higher */
        if (Odr > 208.0f) {
          Odr = 208.0f;
        }
        break;
      }
    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  /* Check if the component is enabled */
  if (gyro_is_enabled == 1U) {
    ret = Set_G_ODR_When_Enabled(Odr, Mode);
  } else {
    ret = Set_G_ODR_When_Disabled(Odr, Mode);
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSO32 gyroscope sensor output data rate when enabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_G_ODR_When_Enabled(float Odr, LSM6DSO32_GYRO_Operating_Mode_t Mode)
{
  uint8_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSO32_GY_ODR_12Hz5_HIGH_PERF
            : (Odr <=   26.0f) ? LSM6DSO32_GY_ODR_26Hz_HIGH_PERF
            : (Odr <=   52.0f) ? LSM6DSO32_GY_ODR_52Hz_HIGH_PERF
            : (Odr <=  104.0f) ? LSM6DSO32_GY_ODR_104Hz_HIGH_PERF
            : (Odr <=  208.0f) ? LSM6DSO32_GY_ODR_208Hz_HIGH_PERF
            : (Odr <=  417.0f) ? LSM6DSO32_GY_ODR_417Hz_HIGH_PERF
            : (Odr <=  833.0f) ? LSM6DSO32_GY_ODR_833Hz_HIGH_PERF
            : (Odr <= 1667.0f) ? LSM6DSO32_GY_ODR_1667Hz_HIGH_PERF
            : (Odr <= 3333.0f) ? LSM6DSO32_GY_ODR_3333Hz_HIGH_PERF
            :                    LSM6DSO32_GY_ODR_6667Hz_HIGH_PERF;

  // Reset and apply mode bits
  new_odr &= 0x0FU;
  switch (Mode) {
    case LSM6DSO32_GYRO_HIGH_PERFORMANCE_MODE: {
        new_odr |= 0x00U;
        break;
      }
    case LSM6DSO32_GYRO_LOW_POWER_NORMAL_MODE: {
        new_odr |= 0x10U;
        break;
      }
    default:
      return LSM6DSO32_ERROR;
      break;
  }

  /* Output data rate selection. */
  if (lsm6dso32_gy_data_rate_set(&reg_ctx, (lsm6dso32_odr_g_t)new_odr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 gyroscope sensor output data rate when disabled
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_G_ODR_When_Disabled(float Odr, LSM6DSO32_GYRO_Operating_Mode_t Mode)
{

  uint8_t temp_gyro_odr = 0;

  temp_gyro_odr = (Odr <=   12.5f) ? LSM6DSO32_GY_ODR_12Hz5_HIGH_PERF
                  : (Odr <=   26.0f) ? LSM6DSO32_GY_ODR_26Hz_HIGH_PERF
                  : (Odr <=   52.0f) ? LSM6DSO32_GY_ODR_52Hz_HIGH_PERF
                  : (Odr <=  104.0f) ? LSM6DSO32_GY_ODR_104Hz_HIGH_PERF
                  : (Odr <=  208.0f) ? LSM6DSO32_GY_ODR_208Hz_HIGH_PERF
                  : (Odr <=  417.0f) ? LSM6DSO32_GY_ODR_417Hz_HIGH_PERF
                  : (Odr <=  833.0f) ? LSM6DSO32_GY_ODR_833Hz_HIGH_PERF
                  : (Odr <= 1667.0f) ? LSM6DSO32_GY_ODR_1667Hz_HIGH_PERF
                  : (Odr <= 3333.0f) ? LSM6DSO32_GY_ODR_3333Hz_HIGH_PERF
                  :                    LSM6DSO32_GY_ODR_6667Hz_HIGH_PERF;
  // Reset and apply mode bits
  temp_gyro_odr &= 0x0FU;
  switch (Mode) {
    case LSM6DSO32_GYRO_HIGH_PERFORMANCE_MODE: {
        temp_gyro_odr |= 0x00U;
        break;
      }
    case LSM6DSO32_GYRO_LOW_POWER_NORMAL_MODE: {
        temp_gyro_odr |= 0x10U;
        break;
      }
    default:
      return LSM6DSO32_ERROR;
      break;
  }

  gyro_odr = (lsm6dso32_odr_g_t)temp_gyro_odr;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 gyroscope sensor full scale
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_G_FS(int32_t  *FullScale)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_fs_g_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso32_gy_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSO32_125dps:
      *FullScale =  125;
      break;

    case LSM6DSO32_250dps:
      *FullScale =  250;
      break;

    case LSM6DSO32_500dps:
      *FullScale =  500;
      break;

    case LSM6DSO32_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSO32_2000dps:
      *FullScale = 2000;
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSO32 gyroscope sensor full scale
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_G_FS(int32_t FullScale)
{
  lsm6dso32_fs_g_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSO32_125dps
           : (FullScale <= 250)  ? LSM6DSO32_250dps
           : (FullScale <= 500)  ? LSM6DSO32_500dps
           : (FullScale <= 1000) ? LSM6DSO32_1000dps
           :                       LSM6DSO32_2000dps;

  if (lsm6dso32_gy_full_scale_set(&reg_ctx, new_fs) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 gyroscope sensor raw axes
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_G_AxesRaw(int16_t *Value)
{

  /* Read raw data values. */
  if (lsm6dso32_angular_rate_raw_get(&reg_ctx, Value) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 gyroscope sensor axes
 * @param  AngularRate pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_G_Axes(int32_t *AngularRate)
{
  int16_t data_raw[3];
  float sensitivity;

  /* Read raw data values. */
  if (lsm6dso32_angular_rate_raw_get(&reg_ctx, data_raw) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Get LSM6DSO32 actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float)((float)data_raw[0] * sensitivity));
  AngularRate[1] = (int32_t)((float)((float)data_raw[1] * sensitivity));
  AngularRate[2] = (int32_t)((float)((float)data_raw[2] * sensitivity));

  return LSM6DSO32_OK;
}


/**
 * @brief  Get the LSM6DSO32 register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (lsm6dso32_read_reg(&reg_ctx, Reg, Data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}


/**
 * @brief  Set the LSM6DSO32 register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (lsm6dso32_write_reg(&reg_ctx, Reg, &Data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the interrupt latch
 * @param  Status value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Interrupt_Latch(uint8_t Status)
{
  if (Status > 1U) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_int_notification_set(&reg_ctx, (lsm6dso32_lir_t)Status) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the interrupt polarity
 * @param  Status value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Interrupt_Polarity(uint8_t Status)
{
  if (Status > 1U) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_polarity_set(&reg_ctx, (lsm6dso32_h_lactive_t)Status) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the interrupt pin mode
 * @param  Status value to be written
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Interrupt_PinMode_OpenDrain(uint8_t Status)
{
  if (Status > 1U) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_mode_set(&reg_ctx, (lsm6dso32_pp_od_t)Status) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable free fall detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_Free_Fall_Detection(LSM6DSO32_SensorIntPin_t IntPin)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(4) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* FF_DUR setting */
  if (lsm6dso32_ff_dur_set(&reg_ctx, 0x06) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* FF_THS setting */
  if (lsm6dso32_ff_threshold_set(&reg_ctx, LSM6DSO32_FF_TSH_312mg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSO32_INT1_PIN:
      if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val1.md1_cfg.int1_ff = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    case LSM6DSO32_INT2_PIN:
      if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val2.md2_cfg.int2_ff = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable free fall detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_Free_Fall_Detection()
{
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Disable free fall event on both INT1 and INT2 pins */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val1.md1_cfg.int1_ff = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val2.md2_cfg.int2_ff = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* FF_DUR setting */
  if (lsm6dso32_ff_dur_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* FF_THS setting */
  if (lsm6dso32_ff_threshold_set(&reg_ctx, LSM6DSO32_FF_TSH_312mg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set free fall threshold
 * @param  Threshold free fall detection threshold (values from lsm6dso32_ff_ths_t)
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Free_Fall_Threshold(uint8_t Threshold)
{
  if (lsm6dso32_ff_threshold_set(&reg_ctx, (lsm6dso32_ff_ths_t)Threshold) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get free fall threshold
 * @param  Pointer to free fall detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_Free_Fall_Threshold(uint8_t *Threshold)
{
  if (lsm6dso32_ff_threshold_get(&reg_ctx, (lsm6dso32_ff_ths_t *)Threshold) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set free fall duration
 * @param  Duration free fall detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Free_Fall_Duration(uint8_t Duration)
{
  if (lsm6dso32_ff_dur_set(&reg_ctx, Duration) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get free fall duration
 * @param  pointer to free fall detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_Free_Fall_Duration(uint8_t *Duration)
{
  if (lsm6dso32_ff_dur_get(&reg_ctx, Duration) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_Pedometer()
{
  lsm6dso32_pin_int1_route_t val;

  /* Output Data Rate selection */
  if (Set_X_ODR(26.0f) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(4) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pedo_sens_set(&reg_ctx, LSM6DSO32_FALSE_STEP_REJ_ADV_MODE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable step detector on INT1 pin */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val.emb_func_int1.int1_step_detector = PROPERTY_ENABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Disable pedometer
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_Pedometer()
{
  lsm6dso32_pin_int1_route_t val1;

  /* Disable step detector on INT1 pin */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val1.emb_func_int1.int1_step_detector = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pedo_sens_set(&reg_ctx, LSM6DSO32_PEDO_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get step count
 * @param  StepCount step counter
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_Step_Count(uint16_t *StepCount)
{
  if (lsm6dso32_number_of_steps_get(&reg_ctx, StepCount) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable step counter reset
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Step_Counter_Reset()
{
  if (lsm6dso32_steps_reset(&reg_ctx) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable tilt detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_Tilt_Detection(LSM6DSO32_SensorIntPin_t IntPin)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(26.0f) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(4) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Turn on tilt detection features */
  if (lsm6dso32_tilt_sens_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable tilt event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSO32_INT1_PIN:
      if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val1.emb_func_int1.int1_tilt = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    case LSM6DSO32_INT2_PIN:
      if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val2.emb_func_int2.int2_tilt = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable tilt detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_Tilt_Detection()
{
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Disable tilt event on both INT1 and INT2 pins */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val1.emb_func_int1.int1_tilt = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val2.emb_func_int2.int2_tilt = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Turn off tilt detection features */
  if (lsm6dso32_tilt_sens_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable wake up detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_Wake_Up_Detection(LSM6DSO32_SensorIntPin_t IntPin)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(4) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* WAKE_DUR setting */
  if (lsm6dso32_wkup_dur_set(&reg_ctx, 0x01) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set wake up threshold. */
  if (lsm6dso32_wkup_threshold_set(&reg_ctx, 0x02) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable wake up event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSO32_INT1_PIN:
      if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val1.md1_cfg.int1_wu = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    case LSM6DSO32_INT2_PIN:
      if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val2.md2_cfg.int2_wu = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}


/**
 * @brief  Disable wake up detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_Wake_Up_Detection()
{
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Disable wake up event on both INT1 and INT2 pins */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val1.md1_cfg.int1_wu = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val2.md2_cfg.int2_wu = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset wake up threshold. */
  if (lsm6dso32_wkup_threshold_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset WAKE_DUR setting */
  if (lsm6dso32_wkup_dur_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set wake up threshold
 * @param  Threshold wake up detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Wake_Up_Threshold(uint8_t Threshold)
{
  /* Set wake up threshold. */
  if (lsm6dso32_wkup_threshold_set(&reg_ctx, Threshold) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get wake up threshold
 * @param  Pointer to wake up detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_Wake_Up_Threshold(uint8_t *Threshold)
{
  /* Get wake up threshold. */
  if (lsm6dso32_wkup_threshold_get(&reg_ctx, Threshold) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set wake up duration
 * @param  Duration wake up detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Wake_Up_Duration(uint8_t Duration)
{
  /* Set wake up duration. */
  if (lsm6dso32_wkup_dur_set(&reg_ctx, Duration) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get wake up duration
 * @param  Pointer to wake up detection duration
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_Wake_Up_Duration(uint8_t *Duration)
{
  /* Get wake up duration. */
  if (lsm6dso32_wkup_dur_get(&reg_ctx, Duration) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable single tap detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_Single_Tap_Detection(LSM6DSO32_SensorIntPin_t IntPin)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(4) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set tap threshold. */
  if (lsm6dso32_tap_threshold_x_set(&reg_ctx, 0x08) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set tap shock time window. */
  if (lsm6dso32_tap_shock_set(&reg_ctx, 0x02) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set tap quiet time window. */
  if (lsm6dso32_tap_quiet_set(&reg_ctx, 0x01) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSO32_INT1_PIN:
      if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val1.md1_cfg.int1_single_tap = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    case LSM6DSO32_INT2_PIN:
      if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val2.md2_cfg.int2_single_tap = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable single tap detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_Single_Tap_Detection()
{
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Disable single tap event on both INT1 and INT2 pins */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val1.md1_cfg.int1_single_tap = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val2.md2_cfg.int2_single_tap = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset tap quiet time window. */
  if (lsm6dso32_tap_quiet_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset tap shock time window. */
  if (lsm6dso32_tap_shock_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset tap threshold. */
  if (lsm6dso32_tap_threshold_x_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable double tap detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_Double_Tap_Detection(LSM6DSO32_SensorIntPin_t IntPin)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(4) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_x_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_y_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_z_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set tap threshold. */
  if (lsm6dso32_tap_threshold_x_set(&reg_ctx, 0x08) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set tap shock time window. */
  if (lsm6dso32_tap_shock_set(&reg_ctx, 0x03) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set tap quiet time window. */
  if (lsm6dso32_tap_quiet_set(&reg_ctx, 0x03) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Set tap duration time window. */
  if (lsm6dso32_tap_dur_set(&reg_ctx, 0x08) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Single and double tap enabled. */
  if (lsm6dso32_tap_mode_set(&reg_ctx, LSM6DSO32_BOTH_SINGLE_DOUBLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSO32_INT1_PIN:
      if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val1.md1_cfg.int1_double_tap = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    case LSM6DSO32_INT2_PIN:
      if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val2.md2_cfg.int2_double_tap = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable double tap detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_Double_Tap_Detection()
{
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Disable double tap event on both INT1 and INT2 pins */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val1.md1_cfg.int1_double_tap = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val2.md2_cfg.int2_double_tap = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Only single tap enabled. */
  if (lsm6dso32_tap_mode_set(&reg_ctx, LSM6DSO32_ONLY_SINGLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset tap duration time window. */
  if (lsm6dso32_tap_dur_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset tap quiet time window. */
  if (lsm6dso32_tap_quiet_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset tap shock time window. */
  if (lsm6dso32_tap_shock_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset tap threshold. */
  if (lsm6dso32_tap_threshold_x_set(&reg_ctx, 0x00) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_z_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_y_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (lsm6dso32_tap_detection_on_x_set(&reg_ctx, PROPERTY_DISABLE) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set tap threshold
 * @param  Threshold tap threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Tap_Threshold(uint8_t Threshold)
{
  /* Set tap threshold. */
  if (lsm6dso32_tap_threshold_x_set(&reg_ctx, Threshold) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set tap shock time
 * @param  Time tap shock time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Tap_Shock_Time(uint8_t Time)
{
  /* Set tap shock time window. */
  if (lsm6dso32_tap_shock_set(&reg_ctx, Time) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set tap quiet time
 * @param  Time tap quiet time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Tap_Quiet_Time(uint8_t Time)
{
  /* Set tap quiet time window. */
  if (lsm6dso32_tap_quiet_set(&reg_ctx, Time) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set tap duration time
 * @param  Time tap duration time
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_Tap_Duration_Time(uint8_t Time)
{
  /* Set tap duration time window. */
  if (lsm6dso32_tap_dur_set(&reg_ctx, Time) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Enable 6D orientation detection
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Enable_6D_Orientation(LSM6DSO32_SensorIntPin_t IntPin)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(4) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* 6D orientation enabled. */
  if (lsm6dso32_6d_threshold_set(&reg_ctx, LSM6DSO32_DEG_47) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Enable 6D orientation event on either INT1 or INT2 pin */
  switch (IntPin) {
    case LSM6DSO32_INT1_PIN:
      if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val1.md1_cfg.int1_6d = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    case LSM6DSO32_INT2_PIN:
      if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }

      val2.md2_cfg.int2_6d = PROPERTY_ENABLE;

      if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
        return LSM6DSO32_ERROR;
      }
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable 6D orientation detection
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Disable_6D_Orientation()
{
  lsm6dso32_pin_int1_route_t val1;
  lsm6dso32_pin_int2_route_t val2;

  /* Disable 6D orientation event on both INT1 and INT2 pins */
  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val1.md1_cfg.int1_6d = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &val1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int2_route_get(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  val2.md2_cfg.int2_6d = PROPERTY_DISABLE;

  if (lsm6dso32_pin_int2_route_set(&reg_ctx, &val2) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  /* Reset 6D orientation. */
  if (lsm6dso32_6d_threshold_set(&reg_ctx, LSM6DSO32_DEG_68) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set 6D orientation threshold
 * @param  Threshold 6D Orientation detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_6D_Orientation_Threshold(uint8_t Threshold)
{
  if (lsm6dso32_6d_threshold_set(&reg_ctx, (lsm6dso32_sixd_ths_t)Threshold) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the status of XLow orientation
 * @param  XLow the status of XLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_6D_Orientation_XL(uint8_t *XLow)
{
  lsm6dso32_d6d_src_t data;

  if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *XLow = data.xl;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the status of XHigh orientation
 * @param  XHigh the status of XHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_6D_Orientation_XH(uint8_t *XHigh)
{
  lsm6dso32_d6d_src_t data;

  if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *XHigh = data.xh;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the status of YLow orientation
 * @param  YLow the status of YLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_6D_Orientation_YL(uint8_t *YLow)
{
  lsm6dso32_d6d_src_t data;

  if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *YLow = data.yl;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the status of YHigh orientation
 * @param  YHigh the status of YHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_6D_Orientation_YH(uint8_t *YHigh)
{
  lsm6dso32_d6d_src_t data;

  if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *YHigh = data.yh;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the status of ZLow orientation
 * @param  ZLow the status of ZLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_6D_Orientation_ZL(uint8_t *ZLow)
{
  lsm6dso32_d6d_src_t data;

  if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *ZLow = data.zl;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the status of ZHigh orientation
 * @param  ZHigh the status of ZHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_6D_Orientation_ZH(uint8_t *ZHigh)
{
  lsm6dso32_d6d_src_t data;

  if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *ZHigh = data.zh;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 ACC data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_DRDY_Status(uint8_t *Status)
{
  if (lsm6dso32_xl_flag_data_ready_get(&reg_ctx, Status) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the status of all hardware events
 * @param  pObj the device pObj
 * @param  Status the status of all hardware events
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_X_Event_Status(LSM6DSO32_Event_Status_t *Status)
{

  lsm6dso32_all_sources_t all_src;
  lsm6dso32_pin_int1_route_t int1_cfg;
  lsm6dso32_pin_int2_route_t int2_cfg;


  (void)memset((void *)Status, 0x0, sizeof(LSM6DSO32_Event_Status_t));

  if (lsm6dso32_all_sources_get(&reg_ctx, &all_src) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &int1_cfg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if (lsm6dso32_pin_int2_route_get(&reg_ctx, &int2_cfg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  if ((int1_cfg.md1_cfg.int1_ff == 1U) || (int2_cfg.md2_cfg.int2_ff == 1U)) {
    if (all_src.all_int_src.ff_ia == 1U) {
      Status->FreeFallStatus = 1;
    }
  }

  if ((int1_cfg.md1_cfg.int1_wu == 1U) || (int2_cfg.md2_cfg.int2_wu == 1U)) {
    if (all_src.all_int_src.wu_ia == 1U) {
      Status->WakeUpStatus = 1;
    }
  }

  if ((int1_cfg.md1_cfg.int1_single_tap == 1U) || (int2_cfg.md2_cfg.int2_single_tap == 1U)) {
    if (all_src.all_int_src.single_tap == 1U) {
      Status->TapStatus = 1;
    }
  }

  if ((int1_cfg.md1_cfg.int1_double_tap == 1U) || (int2_cfg.md2_cfg.int2_double_tap == 1U)) {
    if (all_src.all_int_src.double_tap == 1U) {
      Status->DoubleTapStatus = 1;
    }
  }

  if ((int1_cfg.md1_cfg.int1_6d == 1U) || (int2_cfg.md2_cfg.int2_6d == 1U)) {
    if (all_src.all_int_src.d6d_ia == 1U) {
      Status->D6DOrientationStatus = 1;
    }
  }

  if ((int1_cfg.emb_func_int1.int1_step_detector == 1U) || (int2_cfg.emb_func_int2.int2_step_detector == 1U)) {
    if (all_src.emb_func_status.is_step_det == 1U) {
      Status->StepStatus = 1;
    }
  }

  if ((int1_cfg.emb_func_int1.int1_tilt == 1U) || (int2_cfg.emb_func_int2.int2_tilt == 1U)) {
    if (all_src.emb_func_status.is_tilt == 1U) {
      Status->TiltStatus = 1;
    }
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set self test
 * @param  val the value of st_xl in reg CTRL5_C
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_X_SelfTest(uint8_t val)
{
  lsm6dso32_st_xl_t reg;

  reg = (val == 0U)  ? LSM6DSO32_XL_ST_DISABLE
        : (val == 1U)  ? LSM6DSO32_XL_ST_POSITIVE
        : (val == 2U)  ? LSM6DSO32_XL_ST_NEGATIVE
        :                LSM6DSO32_XL_ST_DISABLE;

  if (lsm6dso32_xl_self_test_set(&reg_ctx, reg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 GYRO data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_G_DRDY_Status(uint8_t *Status)
{
  if (lsm6dso32_gy_flag_data_ready_get(&reg_ctx, Status) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set self test
 * @param  val the value of st_g in reg CTRL5_C
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_G_SelfTest(uint8_t val)
{
  lsm6dso32_st_g_t reg;

  reg = (val == 0U)  ? LSM6DSO32_GY_ST_DISABLE
        : (val == 1U)  ? LSM6DSO32_GY_ST_POSITIVE
        : (val == 3U)  ? LSM6DSO32_GY_ST_NEGATIVE
        :                LSM6DSO32_GY_ST_DISABLE;


  if (lsm6dso32_gy_self_test_set(&reg_ctx, reg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 FIFO number of samples
 * @param  NumSamples number of samples
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_FIFO_Num_Samples(uint16_t *NumSamples)
{
  if (lsm6dso32_fifo_data_level_get(&reg_ctx, NumSamples) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 FIFO full status
 * @param  Status FIFO full status, >0 if fifo is full or full at next ODR
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_FIFO_Full_Status(uint8_t *Status)
{
  lsm6dso32_fifo_status2_t reg;

  if (lsm6dso32_fifo_status_get(&reg_ctx, &reg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *Status = reg.fifo_ovr_ia || reg.fifo_full_ia;

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 FIFO full interrupt on INT1 pin
 * @param  Status FIFO full interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_FIFO_INT1_FIFO_Full(uint8_t Status)
{
  lsm6dso32_pin_int1_route_t int1_cfg;

  if (lsm6dso32_pin_int1_route_get(&reg_ctx, &int1_cfg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  int1_cfg.int1_ctrl.int1_fifo_full = Status;

  if (lsm6dso32_pin_int1_route_set(&reg_ctx, &int1_cfg) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 FIFO watermark level
 * @param  Watermark FIFO watermark level
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_FIFO_Watermark_Level(uint16_t Watermark)
{
  if (lsm6dso32_fifo_watermark_set(&reg_ctx, Watermark) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 FIFO stop on watermark
 * @param  Status FIFO stop on watermark status
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_FIFO_Stop_On_Fth(uint8_t Status)
{
  if (lsm6dso32_fifo_stop_on_wtm_set(&reg_ctx, Status) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 FIFO mode
 * @param  FIFO mode value (from FIFO_CTRL4 register)
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_FIFO_Mode(uint8_t Mode)
{
  LSM6DSO32StatusTypeDef ret = LSM6DSO32_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lsm6dso32_fifo_mode_t)Mode) {
    case LSM6DSO32_BYPASS_MODE:
    case LSM6DSO32_FIFO_MODE:
    case LSM6DSO32_STREAM_TO_FIFO_MODE:
    case LSM6DSO32_BYPASS_TO_STREAM_MODE:
    case LSM6DSO32_STREAM_MODE:
    case LSM6DSO32_BYPASS_TO_FIFO_MODE:
      break;

    default:
      ret = LSM6DSO32_ERROR;
      break;
  }

  if (ret == LSM6DSO32_ERROR) {
    return ret;
  }

  if (lsm6dso32_fifo_mode_set(&reg_ctx, (lsm6dso32_fifo_mode_t)Mode) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSO32 FIFO tag
 * @param  Tag FIFO tag
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_FIFO_Tag(uint8_t *Tag)
{
  lsm6dso32_fifo_tag_t tag_local;

  if (lsm6dso32_fifo_sensor_tag_get(&reg_ctx, &tag_local) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  *Tag = (uint8_t)tag_local;

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 FIFO raw data
 * @param  Data FIFO raw data array [6]
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_FIFO_Data(uint8_t *Data)
{
  if (lsm6dso32_fifo_out_raw_get(&reg_ctx, Data) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 FIFO accelero single sample (16-bit data per 3 axes) and calculate acceleration [mg]
 * @param  Acceleration FIFO accelero axes [mg]
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_FIFO_X_Axes(int32_t *Acceleration)
{
  uint8_t data[6];
  int16_t data_raw[3];
  float sensitivity = 0.0f;
  float acceleration_float[3];

  if (Get_FIFO_Data(data) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  if (Get_X_Sensitivity(&sensitivity) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  acceleration_float[0] = (float)data_raw[0] * sensitivity;
  acceleration_float[1] = (float)data_raw[1] * sensitivity;
  acceleration_float[2] = (float)data_raw[2] * sensitivity;

  Acceleration[0] = (int32_t)acceleration_float[0];
  Acceleration[1] = (int32_t)acceleration_float[1];
  Acceleration[2] = (int32_t)acceleration_float[2];

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 FIFO accelero BDR value
 * @param  Bdr FIFO accelero BDR value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_FIFO_X_BDR(float Bdr)
{
  lsm6dso32_bdr_xl_t new_bdr;

  new_bdr = (Bdr <=    0.0f) ? LSM6DSO32_XL_NOT_BATCHED
            : (Bdr <=   12.5f) ? LSM6DSO32_XL_BATCHED_AT_12Hz5
            : (Bdr <=   26.0f) ? LSM6DSO32_XL_BATCHED_AT_26Hz
            : (Bdr <=   52.0f) ? LSM6DSO32_XL_BATCHED_AT_52Hz
            : (Bdr <=  104.0f) ? LSM6DSO32_XL_BATCHED_AT_104Hz
            : (Bdr <=  208.0f) ? LSM6DSO32_XL_BATCHED_AT_208Hz
            : (Bdr <=  416.0f) ? LSM6DSO32_XL_BATCHED_AT_417Hz
            : (Bdr <=  833.0f) ? LSM6DSO32_XL_BATCHED_AT_833Hz
            : (Bdr <= 1660.0f) ? LSM6DSO32_XL_BATCHED_AT_1667Hz
            : (Bdr <= 3330.0f) ? LSM6DSO32_XL_BATCHED_AT_3333Hz
            :                    LSM6DSO32_XL_BATCHED_AT_6667Hz;

  if (lsm6dso32_fifo_xl_batch_set(&reg_ctx, new_bdr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

/**
 * @brief  Get the LSM6DSO32 FIFO gyro single sample (16-bit data per 3 axes) and calculate angular velocity [mDPS]
 * @param  AngularVelocity FIFO gyro axes [mDPS]
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Get_FIFO_G_Axes(int32_t *AngularVelocity)
{
  uint8_t data[6];
  int16_t data_raw[3];
  float sensitivity = 0.0f;
  float angular_velocity_float[3];

  if (Get_FIFO_Data(data) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  if (Get_G_Sensitivity(&sensitivity) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  angular_velocity_float[0] = (float)data_raw[0] * sensitivity;
  angular_velocity_float[1] = (float)data_raw[1] * sensitivity;
  angular_velocity_float[2] = (float)data_raw[2] * sensitivity;

  AngularVelocity[0] = (int32_t)angular_velocity_float[0];
  AngularVelocity[1] = (int32_t)angular_velocity_float[1];
  AngularVelocity[2] = (int32_t)angular_velocity_float[2];

  return LSM6DSO32_OK;
}

/**
 * @brief  Set the LSM6DSO32 FIFO gyro BDR value
 * @param  Bdr FIFO gyro BDR value
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO32StatusTypeDef LSM6DSO32Sensor::Set_FIFO_G_BDR(float Bdr)
{
  lsm6dso32_bdr_gy_t new_bdr;

  new_bdr = (Bdr <=    0.0f) ? LSM6DSO32_GY_NOT_BATCHED
            : (Bdr <=   12.5f) ? LSM6DSO32_GY_BATCHED_AT_12Hz5
            : (Bdr <=   26.0f) ? LSM6DSO32_GY_BATCHED_AT_26Hz
            : (Bdr <=   52.0f) ? LSM6DSO32_GY_BATCHED_AT_52Hz
            : (Bdr <=  104.0f) ? LSM6DSO32_GY_BATCHED_AT_104Hz
            : (Bdr <=  208.0f) ? LSM6DSO32_GY_BATCHED_AT_208Hz
            : (Bdr <=  416.0f) ? LSM6DSO32_GY_BATCHED_AT_417Hz
            : (Bdr <=  833.0f) ? LSM6DSO32_GY_BATCHED_AT_833Hz
            : (Bdr <= 1660.0f) ? LSM6DSO32_GY_BATCHED_AT_1667Hz
            : (Bdr <= 3330.0f) ? LSM6DSO32_GY_BATCHED_AT_3333Hz
            :                    LSM6DSO32_GY_BATCHED_AT_6667Hz;

  if (lsm6dso32_fifo_gy_batch_set(&reg_ctx, new_bdr) != LSM6DSO32_OK) {
    return LSM6DSO32_ERROR;
  }

  return LSM6DSO32_OK;
}

int32_t LSM6DSO32_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LSM6DSO32Sensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LSM6DSO32_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LSM6DSO32Sensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
