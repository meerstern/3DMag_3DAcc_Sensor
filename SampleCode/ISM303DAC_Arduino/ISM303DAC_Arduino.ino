/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdio.h>
#include "ism303dac_reg.h"


typedef struct {
  //void   *hbus;
  uint8_t i2c_address;
  //GPIO_TypeDef *cs_port;
  //uint16_t cs_pin;
} sensbus_t;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME          20 //ms
//#define TX_BUF_DIM          100

#define    WAIT_TIME_XL     200 //ms
#define    WAIT_TIME_01     20 //ms
#define    WAIT_TIME_02     60 //ms

#define    SAMPLES_XL        5 //number of samples
#define    SAMPLES_MG       50 //number of samples
/* Self test limits. */
#define    MIN_ST_LIMIT_mg         70.0f
#define    MAX_ST_LIMIT_mg       1500.0f
#define    MIN_ST_LIMIT_mG         15.0f
#define    MAX_ST_LIMIT_mG        500.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Private variables ---------------------------------------------------------*/
static sensbus_t xl_bus =  {//&SENSOR_BUS,
                            (ISM303DAC_I2C_ADD_XL>>1)//,
                            //0,
                            //0
                           };
static sensbus_t mag_bus = {//&SENSOR_BUS,
                            (ISM303DAC_I2C_ADD_MG>>1)//,
                           // 0,
                           // 0
                           };


static int16_t data_raw_acceleration[3];
static int16_t data_raw_magnetic[3];
static float_t acceleration_mg[3];
static float_t magnetic_mG[3];
static uint8_t whoamI, rst;
//static uint8_t tx_buffer[TX_BUF_DIM];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);
void ism303dac_read_data_polling(void);
void ism303dac_self_test(void);

/* Example --------------------------------------------------------------*/
void setup() 
{

  Serial.begin(9600);
  delay(10);
  Serial.println("ISM303DAC Demo");
  delay(10);
  ism303dac_self_test();
  ism303dac_read_data_polling();
}


void loop() 
{
  

}

/* Main Example --------------------------------------------------------------*/
void ism303dac_read_data_polling(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx_xl;
  stmdev_ctx_t dev_ctx_mg;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_xl.write_reg = platform_write;
  dev_ctx_xl.read_reg = platform_read;
  dev_ctx_xl.handle = (void *)&xl_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void *)&mag_bus;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  whoamI = 0;
  ism303dac_xl_device_id_get(&dev_ctx_xl, &whoamI);

  if ( whoamI != ISM303DAC_ID_XL )
    while (1); /*manage here device not found */

  whoamI = 0;
  ism303dac_mg_device_id_get(&dev_ctx_mg, &whoamI);

  if ( whoamI != ISM303DAC_ID_MG )
    while (1); /*manage here device not found */

  /* Restore default configuration */
  ism303dac_xl_reset_set(&dev_ctx_xl, PROPERTY_ENABLE);

  do {
    ism303dac_xl_reset_get(&dev_ctx_xl, &rst);
  } while (rst);

  ism303dac_mg_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

  do {
    ism303dac_mg_reset_get(&dev_ctx_mg, &rst);
  } while (rst);

  /* Enable Block Data Update */
  ism303dac_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  ism303dac_mg_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set full scale */
  ism303dac_xl_full_scale_set(&dev_ctx_xl, ISM303DAC_XL_2g);
  /* Configure filtering chain */
  /* Accelerometer - High Pass / Slope path */
  //ism303dac_xl_hp_path_set(&dev_ctx_xl, ISM303DAC_HP_ON_OUTPUTS);
  /* Set / Reset magnetic sensor mode */
  ism303dac_mg_set_rst_mode_set(&dev_ctx_mg,
                                ISM303DAC_MG_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation on mag sensor */
  ism303dac_mg_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  ism303dac_xl_data_rate_set(&dev_ctx_xl, ISM303DAC_XL_ODR_100Hz_LP);
  ism303dac_mg_data_rate_set(&dev_ctx_mg, ISM303DAC_MG_ODR_10Hz);
  /* Set magnetometer in continuous mode */
  ism303dac_mg_operating_mode_set(&dev_ctx_mg,
                                  ISM303DAC_MG_CONTINUOUS_MODE);

  /* Read samples in polling mode (no int) */
  while (1) {
    /* Read output only if new value is available */
    ism303dac_reg_t reg;
    ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);

    if (reg.status_a.drdy) {
      /* Read acceleration data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw_acceleration);
      acceleration_mg[0] = ism303dac_from_fs2g_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = ism303dac_from_fs2g_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = ism303dac_from_fs2g_to_mg(
                             data_raw_acceleration[2]);
      //Arduino Not Supportted snprintf float
      //snprintf((char *)tx_buffer, sizeof(tx_buffer),
      //        "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
      //        acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      //tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
      Serial.print("Acceleration [mg]:\t\t");    
      Serial.print(acceleration_mg[0]);
      Serial.print(",\t\t");    
      Serial.print(acceleration_mg[1]);
      Serial.print(",\t\t");    
      Serial.print(acceleration_mg[2]);
      Serial.println();
    }

    ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);

    if (reg.status_reg_m.zyxda) {
      /* Read magnetic field data */
      memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
      ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic);
      magnetic_mG[0] = ism303dac_from_lsb_to_mG( data_raw_magnetic[0]);
      magnetic_mG[1] = ism303dac_from_lsb_to_mG( data_raw_magnetic[1]);
      magnetic_mG[2] = ism303dac_from_lsb_to_mG( data_raw_magnetic[2]);
      //Arduino Not Supportted snprintf float
      //snprintf((char *)tx_buffer, sizeof(tx_buffer),
      //        "Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
      //       magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
      //tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
      Serial.print("Magnetic field [mG]:\t\t");    
      Serial.print(magnetic_mG[0]);
      Serial.print(",\t\t");    
      Serial.print(magnetic_mG[1]);
      Serial.print(",\t\t");    
      Serial.print(magnetic_mG[2]);
      Serial.println();
    }
  }
}

void ism303dac_self_test(void)
{
  stmdev_ctx_t dev_ctx_xl;
  stmdev_ctx_t dev_ctx_mg;
  uint8_t tx_buffer[1000];
  float_t meas_st_off[3];
  int16_t data_raw[3];
  float_t meas_st_on[3];
  ism303dac_reg_t reg;
  float_t test_val[3];
  uint8_t st_result;
  uint8_t i, j;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_xl.write_reg = platform_write;
  dev_ctx_xl.read_reg = platform_read;
  dev_ctx_xl.handle = (void *)&xl_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void *)&mag_bus;
  /* Initialize self test results */
  st_result = ST_PASS;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  reg.byte = 0;
  ism303dac_xl_device_id_get(&dev_ctx_xl, &reg.byte);

  if ( reg.byte != ISM303DAC_ID_XL )
    while (1); /*manage here device not found */

  reg.byte = 0;
  ism303dac_mg_device_id_get(&dev_ctx_mg, &reg.byte);

  if ( reg.byte != ISM303DAC_ID_MG )
    while (1); /*manage here device not found */

  /* Restore default configuration */
  ism303dac_xl_reset_set(&dev_ctx_xl, PROPERTY_ENABLE);

  do {
    ism303dac_xl_reset_get(&dev_ctx_xl, &reg.byte);
  } while (reg.byte);

  ism303dac_mg_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

  do {
    ism303dac_mg_reset_get(&dev_ctx_mg, &reg.byte);
  } while (reg.byte);

  /* Enable Block Data Update */
  ism303dac_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  ism303dac_mg_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /*
   * START ACCELEROMETER SELF TEST PROCEDURE
   */
  /* Set full scale */
  ism303dac_xl_full_scale_set(&dev_ctx_xl, ISM303DAC_XL_2g);
  /* Set Output Data Rate. */
  ism303dac_xl_data_rate_set(&dev_ctx_xl, ISM303DAC_XL_ODR_50Hz_HR);
  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
  } while (!reg.status_a.drdy);

  /* Read dummy data and discard it */
  ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read samples and get the average vale for each axis */
  for (i = 0; i < SAMPLES_XL; i++) {
    /* Check if new value available */
    do {
      ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
    } while (!reg.status_a.drdy);

    /* Read data and accumulate the mg value */
    ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_off[j] += ism303dac_from_fs2g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_off[i] /= SAMPLES_XL;
  }

  /* Enable Self Test positive (or negative) */
  ism303dac_xl_self_test_set(&dev_ctx_xl, ISM303DAC_XL_ST_POSITIVE);
  //ism303dac_xl_self_test_set(&dev_ctx_xl, ISM303DAC_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
  } while (!reg.status_a.drdy);

  /* Read dummy data and discard it */
  ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read samples and get the average vale for each axis */
  for (i = 0; i < SAMPLES_XL; i++) {
    /* Check if new value available */
    do {
      ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
    } while (!reg.status_a.drdy);

    /* Read data and accumulate the mg value */
    ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_on[j] += ism303dac_from_fs2g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_on[i] /= SAMPLES_XL;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((meas_st_on[i] - meas_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }

    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }

  /* Disable Self Test */
  ism303dac_xl_self_test_set(&dev_ctx_xl, ISM303DAC_XL_ST_DISABLE);
  /* Disable sensor. */
  ism303dac_xl_data_rate_set(&dev_ctx_xl, ISM303DAC_XL_ODR_OFF);
  /*
   * END ACCELEROMETER SELF TEST PROCEDURE
   */
  /*
   * START MAGNETOMETER SELF TEST PROCEDURE
   */
  /* Temperature compensation enable */
  ism303dac_mg_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set restore magnetic condition policy */
  ism303dac_mg_set_rst_mode_set(&dev_ctx_mg,
                                ISM303DAC_MG_SET_SENS_ODR_DIV_63);
  /* Set power mode */
  ism303dac_mg_power_mode_set(&dev_ctx_mg,
                              ISM303DAC_MG_HIGH_RESOLUTION);
  /* Set Output Data Rate */
  ism303dac_mg_data_rate_set(&dev_ctx_mg, ISM303DAC_MG_ODR_100Hz);
  /* Set Operating mode */
  ism303dac_mg_operating_mode_set(&dev_ctx_mg,
                                  ISM303DAC_MG_CONTINUOUS_MODE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_01);

  /* Check if new value available */
  do {
    ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it */
  ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(meas_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES_MG; i++) {
    /* Check if new value available */
    do {
      ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_off[j] += ism303dac_from_lsb_to_mG(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_off[i] /= SAMPLES_MG;
  }

  /* Enable Self Test */
  ism303dac_mg_self_test_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_02);

  /* Check if new value available */
  do {
    ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it */
  ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(meas_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES_MG; i++) {
    /* Check if new value available */
    do {
      ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_on[j] += ism303dac_from_lsb_to_mG(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_on[i] /= SAMPLES_MG;
  }

  st_result = ST_PASS;

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((meas_st_on[i] - meas_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mG > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mG)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  ism303dac_mg_self_test_set(&dev_ctx_mg, PROPERTY_DISABLE);
  /* Disable sensor. */
  ism303dac_mg_operating_mode_set(&dev_ctx_mg, ISM303DAC_MG_POWER_DOWN);

  /*
   * END MAGNETOMETER SELF TEST PROCEDURE
   */

  /* Print self test result */
  if (st_result == ST_PASS) {
    //snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - PASS\r\n" );
    Serial.println("Self Test - PASS");
  }

  else {
    //snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - FAIL\r\n" );
    Serial.println("Self Test - FAIL");
  }

  //tx_com(tx_buffer, strlen((char const *)tx_buffer));
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  Wire.beginTransmission(sensbus->i2c_address);
  Wire.write(reg);
  Wire.write(bufp, len);
  Wire.endTransmission();
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  
  Wire.beginTransmission(sensbus->i2c_address);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom((uint8_t)sensbus->i2c_address, (uint8_t)len);

  while (Wire.available()) {
    *bufp = Wire.read();
    bufp++;
  }
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{

}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
  Wire.begin();
  Wire.setClock(100000);
}
