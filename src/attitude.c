#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "global_objects.h"
#include "magdwick_filter.h"

#define PI 3.14159265f
#define RAD_TO_DEG (57.2957795131f)


LOG_MODULE_REGISTER(attitude_task, LOG_LEVEL_DBG);

// This converts quanterion to degrees after the filter outputs
attitude_data quanterion_to_deg(Quanterion q);


void attitude_thread(void *p1, void *p2, void *p3) {
    bool push = 1;
    struct imu_packet imu_message;
    attitude_data am;
    Quanterion quants = {1.0f, 0.0f, 0.0f, 0.0f}; // This is the default 0 rotation setting
    uint64_t prev_time_ticks = 0;

    while (1) {
        k_msgq_get(&attitude_message_queue, &imu_message, K_FOREVER);
        float gx, gy, gz, ax, ay, az;
        float delta;
        
        // Ensures first run is proper
        if (prev_time_ticks == 0) {
            delta = 0.002f;
            prev_time_ticks = imu_message.timestamp;
        }
        else {
            // Converts the ticks to us, then to seconds
            delta = (float) k_ticks_to_us_floor64(imu_message.timestamp - prev_time_ticks) / 1000000.0f;
            prev_time_ticks = imu_message.timestamp;
        }


        // Convert to radians
        gx = imu_message.gx * (PI/180);
        gy = imu_message.gy * (PI/180);
        gz = imu_message.gz * (PI/180);

        ax = imu_message.ax;
        ay = imu_message.ay;
        az = imu_message.az;

        filterUpdate(gx, gy, gz, ax, ay, az, &quants, delta);

        // Reconverts the quanterions to radians, and finally degrees
        am = quanterion_to_deg(quants);


        // Downsamples from 500hz to 250hz for the control loop
        if (push) {
            int ret = k_msgq_put(&controls_message_queue, &am, K_NO_WAIT);
            if (ret) {
                attitude_data temp;
                // Push new data to message queue if control loop isn't keeping up by removing old and putting new
                LOG_ERR("controls thread not keeping up with the message pipeline, error code: %d", ret);
                k_msgq_get(&controls_message_queue, &temp, K_NO_WAIT);
                k_msgq_put(&controls_message_queue, &am, K_NO_WAIT);
                }
            push = !push;
        }
        else {
            push = !push;
        } 
    } 
}

attitude_data quanterion_to_deg(Quanterion q) {
    attitude_data ret;
    ret.roll = atan2f(2.0f*((q.q1 * q.q2) + (q.q3*q.q4)), 1.0f - 2.0f*(q.q2 * q.q2 + q.q3 * q.q3)) * RAD_TO_DEG;
    
    float sinp = 2.0f * (q.q1 * q.q3 - q.q4 * q.q2);

    // Clamp extreme values to prevent Gimbal Lock / NaN errors
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    ret.pitch = asinf(sinp) * RAD_TO_DEG;

    ret.yaw = atan2f(2.0f * (q.q1 * q.q4 + q.q2 * q.q3), 
            1.0f - 2.0f * (q.q3 * q.q3 + q.q4 * q.q4)) * RAD_TO_DEG;
    
    return ret;

}