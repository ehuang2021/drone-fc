#ifndef IMU_H
#define IMU_H

void mpu_inturrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
int mpu_thread(void *p1, void *p2, void *p3);


#endif