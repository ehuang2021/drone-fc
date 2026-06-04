#ifndef MPU6050_H
#define MPU6050_H

struct accelrometer_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
};

struct gyro_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
};

int init_sensor(const struct i2c_dt_spec *dev_i2c);


int read_acclerometer(const struct i2c_dt_spec *dev_i2c, struct accelrometer_data *data);


int read_gyro(const struct i2c_dt_spec *dev_i2c, struct gyro_data *data);

#endif