#include <zephyr/kernel.h>

#include "global_objects.h"
#include "pid.h"
#include "ble.h"


#define SETPOINT 0
#define EQULIBIRUM 35

K_MSGQ_DEFINE(pid_command_queue, sizeof(struct pid_command), 4, sizeof(void *));

int pid_submit_command(const struct pid_command *command) {
    return k_msgq_put(&pid_command_queue, command, K_NO_WAIT);
}


void pid_thread(void *p1, void *p2, void *p3) {
    float delta, result;
    float output; // 0 to 100
    float input, setpoint = SETPOINT;
    float total_err = 0, last_input = 0;
    uint32_t prev_time = 0;
    struct attitude_data attitude_message;
    float iError = 0;

    float ki = 0, kp = 0, kd = 0;
    struct pid_command command;
    uint8_t ble_push = 0;

    int prev_clamped = 0; // -1 for negatively clamped, 1 for positively clampled.

    while (1) {

        k_msgq_get(&controls_message_queue, &attitude_message, K_FOREVER);

        while (k_msgq_get(&pid_command_queue, &command, K_NO_WAIT) == 0) {
            if (command.type == PID_COMMAND_SET_GAINS) {
                kp = command.kp;
                ki = command.ki;
                kd = command.kd;
            }
        }

        input = attitude_message.roll;
        // Ensures first run is proper
        if (prev_time == 0) {
            delta = 0.004f;
            prev_time = k_cycle_get_32();
        }
        else {
            // Converts the ticks to us, then to seconds
            uint32_t now = k_cycle_get_32();
            delta = (float) k_cyc_to_us_floor32(now - prev_time) / 1000000.0f;
            prev_time = now;
        }

        float error = setpoint - input;
        total_err += error;
        
        // Anti-windup
        if (!prev_clamped) {
            // does not allow for integral to be added if output is already clamped
        iError += ki * (error * delta);
        }
        else if (prev_clamped == -1 && (error*delta) > 0) {
            // UNLESS the integral is positive + prev_clamped was negative
            iError += ki * (error*delta);
        }
        else if (prev_clamped == 1 && (error*delta) < 0) {
            // same other way around
            iError += ki * (error*delta);
        }
        float dError = (input - last_input) / delta;
        last_input = input;
        result = (kp * error) + (iError) - (kd * dError);

        if (result > 100) {
            result = 100;
        }
        else if (result < -100) {
            result = -100;
        }
        else {
            prev_clamped = 0;
        }

        output = result + EQULIBIRUM;
        if (output > 100) {
            output = 100;
            prev_clamped = 1;
        }
        else if (output < 0) {
            output = 0;
            prev_clamped = -1;
        }
        else {
            prev_clamped = 0;
        }

        // Downsample PID telemetry from 250hz to 25hz
        ble_push++;
        if (ble_push >= 10) {
            ble_publish_pid(output);
            ble_push = 0;
        }

        while ((k_msgq_put(&pwm_message_queue, &output, K_NO_WAIT))) {
            k_msgq_purge(&pwm_message_queue);
        }
    }

    atomic_set(&pid_heartbeat, k_uptime_get_32());


}


