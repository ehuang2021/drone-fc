#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "mpu6050.h"
#include "imu.h"
#include "global_objects.h"



#define I2C_NODE DT_NODELABEL(mpu6050)
#define ACC_LSB_SENS 16384.0f
#define GYRO_LSB_SENS 65.5f

// Logger module for logs
LOG_MODULE_REGISTER(imu_task, LOG_LEVEL_DBG);


/* Semaphore for a hardware inturrupt given by the mpu6050 sensor. basically, every 2 ms (500hz), the imu sends a data ready ping
and that adds a sem to instance monitor, which causes imu thread to wake up and to pass information to the altitude thread */
K_SEM_DEFINE(instance_monitor, 0, 1);

void mpu_inturrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
        k_sem_give(&instance_monitor);
}

void mpu_thread(void *p1, void *p2, void *p3)
{

        struct imu_data acc;
        struct imu_packet packet;

        while (1) {
                
                k_sem_take(&instance_monitor, K_FOREVER);
                packet.timestamp = k_uptime_ticks();
                int ret = read_data(&acc);
                if (ret) {
                        continue;
                }

                packet.ax = (float) acc.ax/ACC_LSB_SENS;
                packet.ay = (float)acc.ay/ACC_LSB_SENS;
                packet.az = (float)acc.az/ACC_LSB_SENS;
                packet.gx = (float)acc.gx/GYRO_LSB_SENS;
                packet.gy = (float)acc.gy/GYRO_LSB_SENS;
                packet.gz = (float)acc.gz/GYRO_LSB_SENS;
                
                ret = k_msgq_put(&attitude_message_queue, &packet, K_NO_WAIT);

                if (ret) {
                        LOG_ERR("imu -> attitude thread message queue issue, attitude is lagging. error code %d", ret);
                }

        }



}

/*
        packet.ax = acc.acc_x/ACC_LSB_SENS;
        packet.ax = acc.acc_y*1000/ACC_LSB_SENS;
        packet.ax = acc.acc_z*1000/ACC_LSB_SENS;

        packet.gx = (int32_t)acc.gyro_x*1000/GYRO_LSB_SENS;
        packet.gy = (int32_t)acc.gyro_y*1000/GYRO_LSB_SENS;
        packet.gz = (int32_t)acc.gyro_z*1000/GYRO_LSB_SENS;

*/