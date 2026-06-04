#ifndef MPU6050_H
#define MPU6050_H



static int16_t accel_bias_x;
static int16_t accel_bias_y;
static int16_t accel_bias_z;
static int16_t gyro_bias_x;
static int16_t gyro_bias_y;
static int16_t gyro_bias_z;

struct accelrometer_data {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct gyro_data {
    int16_t x;
    int16_t y;
    int16_t z;
};

int init_sensor(const struct i2c_dt_spec *dev_i2c);


int read_acclerometer(const struct i2c_dt_spec *dev_i2c, struct accelrometer_data *data);


int read_gyro(const struct i2c_dt_spec *dev_i2c, struct gyro_data *data);

#endif