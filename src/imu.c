#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "mpu6050.h"
#include "global_objects.h"


#define I2C_NODE DT_NODELABEL(mpu6050)
#define ACC_LSB_SENS 16384
#define GYRO_LSB_SENS 655

// Logger module for logs
LOG_MODULE_REGISTER(imu_task, LOG_LEVEL_DBG);


/* Semaphore for a hardware inturrupt given by the mpu6050 sensor. basically, every 2 ms (500hz), the imu sends a data ready ping
and that adds a sem to instance monitor, which causes imu thread to wake up and to pass information to the altitude thread */
K_SEM_DEFINE(instance_monitor, 0, 1);

void mpu_inturrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
        k_sem_give(&instance_monitor);
}

int mpu_thread(void *p1, void *p2, void *p3)
{

        struct imu_data acc;
        struct imu_packet packet;

        while (1) {
                
                k_sem_take(&instance_monitor, K_FOREVER);

                int ret = read_data(&acc);
                if (ret) {
                        LOG_ERR("read_data error, something is wrong within the mpu driver. Error code: %d", ret);
                        return ret;
                }

                packet.ax = acc.ax;
                packet.ay = acc.ay;
                packet.az = acc.az;
                packet.gx = acc.gx;
                packet.gy = acc.gy;
                packet.gz = acc.gz;
                packet.timestamp = k_uptime_get();
                
                

        }



}

/*
        packet.ax = acc.acc_x*1000/ACC_LSB_SENS;
        packet.ax = acc.acc_y*1000/ACC_LSB_SENS;
        packet.ax = acc.acc_z*1000/ACC_LSB_SENS;

        packet.gx = (int32_t)acc.gyro_x*1000/GYRO_LSB_SENS;
        packet.gy = (int32_t)acc.gyro_y*1000/GYRO_LSB_SENS;
        packet.gz = (int32_t)acc.gyro_z*1000/GYRO_LSB_SENS;

*/