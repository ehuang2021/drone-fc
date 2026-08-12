#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>


#include "mpu6050.h"
#include "global_objects.h"
#include "imu.h"
#include "ble.h"

#define I2C_NODE DT_NODELABEL(mpu6050)
#define ESC_PWM_NODE DT_NODELABEL(pwm1)

// Thread Definitions
#define IMU_STACKSIZE 1024
#define IMU_THREAD_PRIORITY 0

#define ATTITUDE_STACKSIZE 2048
#define ATTITUDE_THREAD_PRIORITY 1

#define PID_STACKSIZE 1024
#define PID_THREAD_PRIORITY 2

#define PWM_STACKSIZE 1024
#define PWM_THREAD_PRIORITY 3

#define DRONE_SYSTEMS_STACKSIZE 1024
#define DRONE_SYSTEMS_THREAD_PRIORITY 4

#define BLE_STACKSIZE 2048
#define BLE_THREAD_PRIORITY 5

// Initalize GPIO
static struct gpio_dt_spec mpu_int_gpio = GPIO_DT_SPEC_GET(I2C_NODE, int_gpios);
static struct gpio_callback mpu_int_cb_data;

static const struct device *esc_pwm = DEVICE_DT_GET(ESC_PWM_NODE);

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

// Initalizes the imu -> attitude message queue
K_MSGQ_DEFINE(attitude_message_queue, sizeof(struct imu_packet), 1, sizeof(void *));
// Initlaize the attiude -> pid thread
K_MSGQ_DEFINE(controls_message_queue, sizeof(attitude_data), 1, sizeof(void *));
// initlaize pid -> pwm thread
K_MSGQ_DEFINE(pwm_message_queue, sizeof(float), 1, sizeof(void *));


K_THREAD_DEFINE(imu_tasks, IMU_STACKSIZE, mpu_thread, NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(attitude_tasks, ATTITUDE_STACKSIZE, attitude_thread, NULL, NULL, NULL, ATTITUDE_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(pid_tasks, PID_STACKSIZE, pid_thread, NULL, NULL, NULL, PID_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(pwm_tasks, PWM_STACKSIZE, pwm_thread, NULL, NULL, NULL, PWM_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(drone_systems_tasks, DRONE_SYSTEMS_STACKSIZE, drone_systems_thread, NULL, NULL, NULL, DRONE_SYSTEMS_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_tasks, BLE_STACKSIZE, ble_thread, NULL, NULL, NULL, BLE_THREAD_PRIORITY, 0, 0);



int main() {
    int ret = ble_init();
    if (ret) {
        LOG_ERR("BLE init failed, continuing without BLE");
    }

    init_mpu6050();
    initalize_imu_inturrupts();


    if (!device_is_ready(esc_pwm)) {
        LOG_ERR("PWM device not ready");
        return -1;
    }

    pwm_set(esc_pwm, 0, PWM_MSEC(20), PWM_USEC(1000), PWM_POLARITY_NORMAL);
    return 0;
}
