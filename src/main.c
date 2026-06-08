#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>


#include "mpu6050.h"
#include "global_objects.h"
#include "imu.h"

#define I2C_NODE DT_NODELABEL(mpu6050)

// Thread Definitions
#define IMU_STACKSIZE 1024
#define IMU_THREAD_PRIORITY 0


// Initalize GPIO
static struct gpio_dt_spec mpu_int_gpio = GPIO_DT_SPEC_GET(I2C_NODE, int_gpios);
static struct gpio_callback mpu_int_cb_data;

LOG_MODULE_REGISTER(main_task, LOG_LEVEL_DBG);


// This function init's the imu inturupt, which is responsible for thread unblocking for imu.c
int initalize_imu_inturrupts() {
    int ret = gpio_pin_configure_dt(&mpu_int_gpio, GPIO_INPUT);
    if (ret) {
            LOG_ERR("gpio_pin_configure_dt Error code: %d", ret);
            return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&mpu_int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
            LOG_ERR("gpio_pin_interrupt_configure_dt error: Error code: %d", ret);
            return ret;
    }
    gpio_init_callback(&mpu_int_cb_data, mpu_inturrupt, BIT(mpu_int_gpio.pin)); 	
    gpio_add_callback(mpu_int_gpio.port, &mpu_int_cb_data);

    return 0;


}

K_MSGQ_DEFINE(altitude_message_queue, sizeof(struct imu_packet), 1, sizeof(void *));

K_THREAD_DEFINE(imu_thread, IMU_STACKSIZE, mpu_thread, NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 0);

int main() {
    init_mpu6050();
    initalize_imu_inturrupts();
    k_msleep(500000);
    return 0;
}