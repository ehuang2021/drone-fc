#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>


#include "global_objects.h"
#include "pwm.h"

#define ESC_PWM_NODE DT_NODELABEL(pwm1)

static const struct device *esc_pwm =
    DEVICE_DT_GET(ESC_PWM_NODE);


LOG_MODULE_REGISTER(pwm_task, LOG_LEVEL_DBG);

void pwm_thread(void *p1, void *p2, void *p3) {

    float throttle;

    if (!device_is_ready(esc_pwm)) {
        LOG_ERR("PWM device not ready");
        return;
    }

    while (1) {
        k_msgq_get(&pwm_message_queue, &throttle, K_FOREVER);

    
        pwm_set(esc_pwm, 0, PWM_MSEC(20), PWM_USEC(throttle_mapper(throttle)), PWM_POLARITY_NORMAL);

    }
}

// maps throttle from 0-100 to 1ms -> 2ms
int throttle_mapper(float throttle) {
    int adder = (throttle * 10);
    return adder + 1000;

}