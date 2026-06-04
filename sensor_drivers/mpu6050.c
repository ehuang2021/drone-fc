#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "mpu6050.h"

// Accelerometer Measurements
#define ACCEL_XOUT_7_0  0x3B

// Gyroscope Measurements
#define GYRO_XOUT_7_0 0x43


#define PWR_MGMT_1 0x6B





static struct i2c_dt_spec dev_i2c;

int init_sensor(const struct i2c_dt_spec *pass) {
    if (!device_is_ready(dev_i2c.bus)) {
        return -1;
    }

    uint8_t write_buf[] = {PWR_MGMT_1, 0x00};
    int ret = i2c_write_dt(&dev_i2c, write_buf, sizeof(write_buf));
    if (ret != 0) {
        return ret;
    }

    return 0;

}

int read_acclerometer(const struct i2c_dt_spec *dev_i2c, struct accelrometer_data *data) {

    int8_t vals[6];
    // Read x,y,z vals at once
    int ret = i2c_burst_read_dt(dev_i2c, ACCEL_XOUT_7_0, vals, 6);
    if (ret != 0) {
        return ret;
    }

    data->accel_x = (vals[0] << 8 | vals[1]);
    data->accel_y = (vals[2] << 8 | vals[3]);
    data->accel_z = (vals[4] << 8 | vals[5]);

    return 0;

}

int read_gyro(const struct i2c_dt_spec *dev_i2c, struct gyro_data *data) {

    int8_t vals[6];
    // Read x,y,z vals at once
    int ret = i2c_burst_read_dt(dev_i2c, GYRO_XOUT_7_0, vals, 6);
    if (ret != 0) {
        return ret;
    }

    data->accel_x = (vals[0] << 8 | vals[1]);
    data->accel_y = (vals[2] << 8 | vals[3]);
    data->accel_z = (vals[4] << 8 | vals[5]);

    return 0;

}

