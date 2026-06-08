#ifndef GLOBAL_OBJECTS_H
#define GLOBAL_OBJECTS_H

struct imu_packet {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int64_t timestamp;

};


extern struct k_msgq altitude_message_queue;

extern void mpu_thread(void *p1, void *p2, void *p3);

#endif