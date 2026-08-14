#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>


#include "global_objects.h"
#include "ble.h"
#include "safety.h"

#define ESC_PWM_NODE DT_NODELABEL(pwm1)

static const struct device *esc_pwm =
    DEVICE_DT_GET(ESC_PWM_NODE);


LOG_MODULE_REGISTER(pwm_task, LOG_LEVEL_DBG);


// maps throttle from 0-100 to 1ms -> 2ms
int throttle_mapper(float throttle) {
    int adder = (throttle * 10);
    return adder + 1000;

}

void pwm_thread(void *p1, void *p2, void *p3) {

    float throttle;
    uint8_t ble_push = 0;

    if (!device_is_ready(esc_pwm)) {
        LOG_ERR("PWM device not ready");
        return;
    }

    while (1) {
        k_msgq_get(&pwm_message_queue, &throttle, K_FOREVER);

        if (!check_flight_state()) {
    
        pwm_set(esc_pwm, 0, PWM_MSEC(20), PWM_USEC(throttle_mapper(throttle)), PWM_POLARITY_NORMAL);

        // Downsample actuator telemetry from 250hz to 25hz
        ble_push++;
        if (ble_push >= 10) {
            ble_publish_actuator(throttle);
            ble_push = 0;
        }
    }

    }
}
