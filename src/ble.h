#ifndef BLE_H
#define BLE_H

#include <stdint.h>

int ble_init(void);
void ble_thread(void *p1, void *p2, void *p3);

void ble_publish_attitude(float roll, float pitch, float yaw);
void ble_publish_pid(float output);
void ble_publish_actuator(float throttle);
void ble_publish_system(uint16_t battery_mv, uint32_t flags);

#endif
