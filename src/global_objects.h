#ifndef GLOBAL_OBJECTS_H
#define GLOBAL_OBJECTS_H

// message packet for imu -> attitude threads
struct imu_packet   {
    float ax, ay, az;
    float gx, gy, gz; 
    int64_t timestamp; // Total ticks

};

// message packet for attitude -> control threads
typedef struct attitude_data {
    float roll;
    float pitch;
    float yaw;
} attitude_data;

extern struct k_msgq attitude_message_queue;
extern struct k_msgq controls_message_queue;
extern struct k_msgq pwm_message_queue;

extern void mpu_thread(void *p1, void *p2, void *p3);
extern void attitude_thread(void *p1, void *p2, void *p3);
extern void pid_thread(void *p1, void *p2, void *p3);
extern void pwm_thread(void *p1, void *p2, void *p3);
extern void drone_systems_thread(void *p1, void *p2, void *p3);
extern void safety_thread(void *p1, void *p2, void *p3);



typedef enum {
    FLIGHT_DISARMED,
    FLIGHT_ARMED,
    FLIGHT_FAULT
} flight_state_t;

typedef struct {
    flight_state_t flight_state;
    uint32_t flight_flags;

} safety_state_t;

extern atomic_t pid_heartbeat;
extern atomic_t attitude_heartbeat;
extern atomic_t imu_heartbeat;
extern atomic_t global_roll;

#endif
