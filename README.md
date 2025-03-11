# LIS2DH12_STM32_C
## To Implement LIS2HD12 into STM32 Project, the following can be followed to allow FIFO readout of accelerometer data.

```c
/**
  ******************************************************************************
  * @file    lis2dh12_reg.c
  * @author  Sensors Software Solution Team
  * @brief   LIS2DH12 driver file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
```
### Defines, Variables, Functions
```c
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
#define FIFO_WATERMARK     10
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static float_t acceleration_mg[3];
static uint8_t whoamI;
static uint8_t tx_buffer[1000];
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
void lis2dh12_read_fifo(void);
// void CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */
```

### HAL I2C Platform dependent code DMA based
```c
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
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write_DMA(handle, LIS2DH12_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len);
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
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
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read_DMA(handle, LIS2DH12_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len);
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  return 0;
}

static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}
```

### Read FIFO
```c
void lis2dh12_read_fifo(void)
{
  stmdev_ctx_t dev_ctx;
  uint8_t dummy;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  platform_delay(100);
  /* Check device ID */
  lis2dh12_device_id_get(&dev_ctx, &whoamI);

  while (whoamI != LIS2DH12_ID) {
	  lis2dh12_device_id_get(&dev_ctx, &whoamI);
	  platform_delay(100);
  }

  /* Set FIFO watermark to FIFO_WATERMARK */
  lis2dh12_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK - 1);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lis2dh12_fifo_mode_set(&dev_ctx, LIS2DH12_DYNAMIC_STREAM_MODE);
  /* Enable FIFO */
  lis2dh12_fifo_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable Block Data Update. */
  lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate to 1Hz. */
  lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_25Hz);
  /* Set full scale to 2g. */
  lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
  /* Set device in continuous mode with 12 bit resol. */
  lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);

  /* Wait Events */
  while (1) {
    /* Check if FIFO level over threshold */
    lis2dh12_fifo_fth_flag_get(&dev_ctx, &dummy);

    if (dummy) {
      /* Read number of sample in FIFO */
      lis2dh12_fifo_data_level_get(&dev_ctx, &dummy);

      while ( dummy > 0) {
        /* Read XL samples */
        lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        acceleration_mg[0] =
          lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[0]);
        acceleration_mg[1] =
          lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] =
          lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration[2]);


        snprintf((char *)tx_buffer, sizeof(tx_buffer),
                "%d - Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                FIFO_WATERMARK - dummy,
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
        CDC_Transmit_FS(tx_buffer, 54);
        dummy--;
      }
    }
  }
}
```
