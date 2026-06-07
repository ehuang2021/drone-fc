#ifndef MPU6050_H
#define MPU6050_H





struct imu_data {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;    
    int16_t temp;
};



int init_mpu6050();


int read_data(struct imu_data *data);



#endif