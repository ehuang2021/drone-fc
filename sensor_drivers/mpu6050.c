#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "mpu6050.h"

/*The outputs here will return the acceleration*/
// Accelerometer Measurements
#define ACCEL_XOUT_7_0  0x3B
// Gyroscope Measurements
#define GYRO_XOUT_7_0 0x43
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B

#define I2C_NODE DT_NODELABEL(mpu6050)

// GYRO Deadzone Fix
#define GYRO_DZ 25
#define LSB_SENS 16384


LOG_MODULE_REGISTER(mpu6050_driver, LOGLEVEL_DBG);

static int16_t accel_bias_x = 0;
static int16_t accel_bias_y = 0;
static int16_t accel_bias_z = 0;
static int16_t gyro_bias_x = 0;
static int16_t gyro_bias_y = 0;
static int16_t gyro_bias_z = 0;

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

int init_mpu6050() {
    
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0, gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    struct imu_data acc;

    // Check I2C bus
    if (!device_is_ready(dev_i2c.bus)) {
        return -1;
    }

    // Start-up register config to Sensor
    uint8_t write_buf1[] = {PWR_MGMT_1, 0x00};
    int ret = i2c_write_dt(&dev_i2c, write_buf1, sizeof(write_buf1));
    if (ret != 0) {
        return ret;
    }
    uint8_t write_buf2[] = {GYRO_CONFIG, 0b00001000};
    ret = i2c_write_dt(&dev_i2c, write_buf2, sizeof(write_buf2));
    if (ret != 0) {
        return ret;
    }

    int num = 1000;
    // Perform a baseline calibration of the sensor. 
    for (int i = 0; i < num; i++) {
        read_data(&acc);
        accel_sum_x += acc.ax;
        accel_sum_y += acc.ay;
        accel_sum_z += acc.az;
        gyro_sum_x += acc.gx;
        gyro_sum_y += acc.gy;
        gyro_sum_z += acc.gz;
    }
    accel_bias_x = 0-accel_sum_x/num;
    accel_bias_y = 0-accel_sum_y/num;
    accel_bias_z = 16384-accel_sum_z/num;

    gyro_bias_x = gyro_sum_x/num;
    gyro_bias_y = gyro_sum_y/num;
    gyro_bias_z = gyro_sum_z/num;
    LOG_INF("Accel Bias: %d, %d, %d\nGyro Bias: %d, %d, %d", accel_bias_x, accel_bias_y, accel_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z);



    return 0;


}

int read_data(struct imu_data *data) {

    uint8_t vals[14];
    // Read x,y,z vals at once
    int ret = i2c_burst_read_dt(&dev_i2c, ACCEL_XOUT_7_0, vals, 14);
    if (ret != 0) {
        return ret;
    }

    data->ax = (vals[0] << 8 | vals[1]) + accel_bias_x;
    data->ay = (vals[2] << 8 | vals[3]) + accel_bias_y;
    data->az = (vals[4] << 8 | vals[5]) + accel_bias_z;
    data->temp = (vals[6] << 8 | vals[7]);
    data->gx = (vals[8] << 8 | vals[9]) - gyro_bias_x;
    data->gy = (vals[10] << 8 | vals[11]) - gyro_bias_y;
    data->gz = (vals[12] << 8 | vals[13]) - gyro_bias_z;


    return 0;

}



