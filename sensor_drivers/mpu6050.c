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
#define CONFIGURATION 0x1A
#define SAMPLE_RATE_DIVIDER 0x19
#define INT_PIN_CONFIG 0x37
#define INT_ENABLE 0x38

#define I2C_NODE DT_NODELABEL(mpu6050)

// GYRO Deadzone Fix
#define GYRO_DZ 25
#define LSB_SENS 16384


LOG_MODULE_REGISTER(mpu6050_driver, LOG_LEVEL_DBG);

static int16_t accel_bias_x = 0;
static int16_t accel_bias_y = 0;
static int16_t accel_bias_z = 0;
static int16_t gyro_bias_x = 0;
static int16_t gyro_bias_y = 0;
static int16_t gyro_bias_z = 0;

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);


int read_data(struct imu_data *data) {

    uint8_t vals[14];
    // Read x,y,z vals at once
    int ret = i2c_burst_read_dt(&dev_i2c, ACCEL_XOUT_7_0, vals, 14);
    if (ret != 0) {
        LOG_ERR("read_data issue. Error code: %d", ret);
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
        LOG_ERR("i2c_write write_buf1 issue. Error code: %d", ret);
        return ret;
    }

    // Configures the Gyro full scale range to 500 deg/s
    uint8_t write_buf2[] = {GYRO_CONFIG, 0b00001000};
    ret = i2c_write_dt(&dev_i2c, write_buf2, sizeof(write_buf2));
    if (ret != 0) {
        LOG_ERR("i2c_write write_buf2 issue. Error code: %d", ret);
        return ret;
    }

    // Configures the on board digital low pass filter for cutoffs of (184 hz for accelerometer, 188 hz for gyroscope, with a 2.0 ms and 1.9 ms delay respectively)
    uint8_t write_buf3[] = {CONFIGURATION, 0b0000001};
    ret = i2c_write_dt(&dev_i2c, write_buf3, sizeof(write_buf3));
    if (ret != 0) {
        LOG_ERR("i2c_write dlpf issue. Error code: %d", ret);
        return ret;
    }

    // sample rate = gyroscope output rate / (1 + div), where gyro output is 1000 hz from the low pass filter
    // Aiming for 500 hz
    uint8_t write_buf4[] = {SAMPLE_RATE_DIVIDER, 0b0000001};
    ret = i2c_write_dt(&dev_i2c, write_buf4, sizeof(write_buf4));
    if (ret != 0) {
        LOG_ERR("i2c_write sample divider issue. Error code: %d", ret);
        return ret;
    }

    // Inturrupt Pin configuration. It is active low, as a push-pull. Int will also emit a 50us pulse, and will clear on any read opperation
    uint8_t write_buf5[] = {INT_PIN_CONFIG, 0b11010000};
    ret = i2c_write_dt(&dev_i2c, write_buf5, sizeof(write_buf5));
    if (ret != 0) {
        LOG_ERR("i2c_write sample divider issue. Error code: %d", ret);
        return ret;
    }

    // Inturrupt Pin enable, and sets it for data ready interrupt
    uint8_t write_buf6[] = {INT_ENABLE, 0b00000001};
    ret = i2c_write_dt(&dev_i2c, write_buf6, sizeof(write_buf6));
    if (ret != 0) {
        LOG_ERR("i2c_write sample divider issue. Error code: %d", ret);
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



