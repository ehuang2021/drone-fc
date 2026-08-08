#include <zephyr/kernel.h>

#include "global_objects.h"


#define SETPOINT 0
#define EQULIBIRUM 35



void pid_thread(void *p1, void *p2, void *p3) {
    float delta, result;
    float output; // 0 to 100
    float input, setpoint = SETPOINT;
    float total_err = 0, last_input = 0;
    uint32_t prev_time = 0;
    struct attitude_data attitude_message;
    float iError = 0;

    float ki = 0, kp = 0, kd = 0;

    int prev_clamped = 0; // -1 for negatively clamped, 1 for positively clampled.

    while (1) {

        k_msgq_get(&controls_message_queue, &attitude_message, K_FOREVER);
        input = attitude_message.roll;
        // Ensures first run is proper
        if (prev_time == 0) {
            delta = 0.004f;
            prev_time = k_cycle_get_32();
        }
        else {
            // Converts the ticks to us, then to seconds
            uint32_t now = k_cycle_get_32();
            delta = (float) k_ticks_to_us_floor32(now - prev_time) / 1000000.0f;
            prev_time = now;
        }

        float error = setpoint - input;
        total_err += error;
        if (!prev_clamped) {
        iError += ki * (error * delta);
        }
        float dError = (input - last_input) / delta;
        last_input = input;
        result = (kp * error) + (iError) - (kd * dError);

        if (result > 100) {
            result = 100;
            prev_clamped = 1;
        }
        else if (result < -100) {
            result = -100;
            prev_clamped = -1;
        }
        else {
            prev_clamped = 0;
        }

        output = result + EQULIBIRUM;
        if (output > 100) {
            result = 100;
        }
        else if (output < 0) {
            result = 0;
        }
        else {
            prev_clamped = 0;
        }

        while ((k_msgq_put(&pwm_message_queue, &output, K_NO_WAIT))) {
            k_msgq_purge(&pwm_message_queue);
        }
    }



}




