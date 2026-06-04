#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "mpu6050.h"
/*The outputs here will return the acceleration*/
// Accelerometer Measurements
#define ACCEL_XOUT_7_0  0x3B
// Gyroscope Measurements
#define GYRO_XOUT_7_0 0x43
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B


// GYRO Deadzone Fix
#define GYRO_DZ 25
#define LSB_SENS 16384


static int16_t accel_bias_x = 0;
static int16_t accel_bias_y = 0;
static int16_t accel_bias_z = 0;
static int16_t gyro_bias_x = 0;
static int16_t gyro_bias_y = 0;
static int16_t gyro_bias_z = 0;

int init_sensor(const struct i2c_dt_spec *dev_i2c) {
    
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0, gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    struct accelrometer_data acc;
    struct gyro_data gyro;

    // Check I2C bus
    if (!device_is_ready(dev_i2c->bus)) {
        return -1;
    }

    // Start-up register config to Sensor
    uint8_t write_buf1[] = {PWR_MGMT_1, 0x00};
    int ret = i2c_write_dt(dev_i2c, write_buf1, sizeof(write_buf1));
    if (ret != 0) {
        return ret;
    }
    uint8_t write_buf2[] = {GYRO_CONFIG, 0b00001000};
    ret = i2c_write_dt(dev_i2c, write_buf2, sizeof(write_buf2));
    if (ret != 0) {
        return ret;
    }

    int num = 1000;
    // Perform a baseline calibration of the sensor. 
    for (int i = 0; i < num; i++) {
        read_acclerometer(dev_i2c, &acc);
        accel_sum_x += acc.x;
        accel_sum_y += acc.y;
        accel_sum_z += acc.z;

        read_gyro(dev_i2c, &gyro);
        gyro_sum_x += gyro.x;
        gyro_sum_y += gyro.y;
        gyro_sum_z += gyro.z;
    }
    accel_bias_x = 0-accel_sum_x/num;
    accel_bias_y = 0-accel_sum_y/num;
    accel_bias_z = 16384-accel_sum_z/num;

    gyro_bias_x = gyro_sum_x/num;
    gyro_bias_y = gyro_sum_y/num;
    gyro_bias_z = gyro_sum_z/num;
    printk("Accel Bias: %d, %d, %d\n", accel_bias_x, accel_bias_y, accel_bias_z);
    printk("Gyro Bias: %d, %d, %d\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);



    return 0;


}

int read_acclerometer(const struct i2c_dt_spec *dev_i2c, struct accelrometer_data *data) {

    uint8_t vals[6];
    // Read x,y,z vals at once
    int ret = i2c_burst_read_dt(dev_i2c, ACCEL_XOUT_7_0, vals, 6);
    if (ret != 0) {
        return ret;
    }

    data->x = (vals[0] << 8 | vals[1]) + accel_bias_x;
    data->y = (vals[2] << 8 | vals[3]) + accel_bias_y;
    data->z = (vals[4] << 8 | vals[5]) + accel_bias_z;

    return 0;

}

int read_gyro(const struct i2c_dt_spec *dev_i2c, struct gyro_data *data) {

    uint8_t vals[6];
    // Read x,y,z vals at once
    int ret = i2c_burst_read_dt(dev_i2c, GYRO_XOUT_7_0, vals, 6);
    if (ret != 0) {
        return ret;
    }

    data->x = (vals[0] << 8 | vals[1]) - gyro_bias_x;
    data->y = (vals[2] << 8 | vals[3]) - gyro_bias_y;
    data->z = (vals[4] << 8 | vals[5]) - gyro_bias_z;

    return 0;

}

